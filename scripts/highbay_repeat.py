"""Independent process restarts of a frozen simulation candidate; no hardware."""
import argparse,hashlib,json,os,signal,subprocess,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def digest(perception_config):
    paths=sorted((ROOT/'src/sly_lane_following').rglob('*.py'))+sorted((ROOT/'src/sly_lane_following/config').glob('*.yaml'))
    paths += [ROOT/'scripts/highbay_experiment.py',Path(__file__).resolve()]
    result={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if '__pycache__' not in str(p)}
    sim=Path('/workspace/gem_simulator')
    for relative in ['gem_gazebo/scripts/gem_control.py','gem_gazebo/worlds/highbay_testbed.world','gem_gazebo/models/highbay_track/meshes/highbay_track.dae','gem_description/urdf/gem.urdf.xacro','gem_description/urdf/gem.gazebo','gem_gazebo/config/gem_joint_control_params_ros2.yaml']:
        result['external/'+relative]=hashlib.sha256((sim/relative).read_bytes()).hexdigest()
    import yaml
    profile=yaml.safe_load(perception_config.read_text())
    result['perception_profile']=hashlib.sha256(perception_config.read_bytes()).hexdigest()
    if profile.get('mask_backend')=='enet':
        checkpoint=Path(profile['enet_checkpoint']);checkpoint=checkpoint if checkpoint.is_absolute() else ROOT/checkpoint
        result['neural_checkpoint']=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    return result

def terminate(process):
    if process is None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    # A launcher can exit before its child. Clear only this dedicated group.
    try:os.killpg(process.pid,signal.SIGTERM)
    except ProcessLookupError:pass

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--lookahead',type=float,default=3.5)
    p.add_argument('--adaptive-speed',action='store_true')
    p.add_argument('--speed',type=float,default=.2)
    p.add_argument('--repeats',type=int,choices=range(1,4),default=3)
    p.add_argument('--route',choices=['combination','full-lap'],default='combination')
    p.add_argument('--perception-config',type=Path,default=ROOT/'src/sly_lane_following/config/highbay.yaml')
    a=p.parse_args();a.perception_config=a.perception_config.resolve()
    if os.environ.get('ROS_DOMAIN_ID')!='84' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated local simulation domain 84')
    # Refuse to coexist with a previously running simulator or adapter.
    lines=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
    if any('gz sim -r' in l or 'ros2 launch sly_lane_following sim.launch.py' in l or '/lane_adapter ' in l or ('-m sly_lane_following.node' in l and 'highbay' in l) for l in lines):
        raise RuntimeError('Stop the existing simulator and adapter before independent repeats')
    output=a.output.resolve();output.mkdir(parents=True,exist_ok=False)
    frozen=digest(a.perception_config);(output/'frozen_sources.json').write_text(json.dumps(frozen,indent=2))
    results=[]
    try:
        for i in range(1,a.repeats+1):
            if digest(a.perception_config)!=frozen:raise RuntimeError('Source changed after freeze')
            sim=adapter=None
            with (output/f'sim_{i}.log').open('w') as sl,(output/f'adapter_{i}.log').open('w') as al:
                try:
                    env=dict(os.environ,GZ_PARTITION='sly_highbay_m3')
                    sim=subprocess.Popen(['bash','--noprofile','--norc','-c',
                        'source /workspace/gem_simulator/install/local_setup.bash; exec ros2 launch sly_lane_following sim.launch.py'],
                        env=env,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True)
                    adapter=subprocess.Popen(['ros2','run','sly_lane_following','lane_adapter','--ros-args',
                        '-p','use_sim_time:=true','-p','platform:=sim','-p','dry_run:=false','-p','adaptive_speed:='+str(a.adaptive_speed).lower()],
                        env=env,stdout=al,stderr=subprocess.STDOUT,start_new_session=True)
                    # Each launch gets a fresh Gazebo clock, vehicle and ROS nodes.
                    started=time.monotonic()
                    while time.monotonic()-started<10:
                        if sim.poll() is not None or adapter.poll() is not None:raise RuntimeError('Launch failed')
                        time.sleep(.2)
                    cmd=['python',str(ROOT/'scripts/highbay_experiment.py'),'pp','--name',f'repeat_{i}',
                         '--perception-config',str(a.perception_config),'--lookahead',str(a.lookahead),'--speed',str(a.speed),'--route',a.route,'--output',str(output)]
                    if a.adaptive_speed:cmd.append('--adaptive-speed')
                    subprocess.run(cmd,env=env,check=True)
                    s=json.loads((output/f'repeat_{i}/summary.json').read_text())
                    s.update(independent_process_restart=True,source_matches_freeze=digest(a.perception_config)==frozen,
                             simulator_launcher_pid=sim.pid,adapter_launcher_pid=adapter.pid)
                    results.append(s)
                    if not s['passed'] or not s['source_matches_freeze']:
                        raise RuntimeError(f'Repeat {i} failed; no further qualification runs')
                finally:
                    terminate(adapter);terminate(sim)
    finally:
        report=dict(passed=len(results)==a.repeats and all(s['passed'] and s['source_matches_freeze'] for s in results),
                    independent_repeats=len(results),requested_repeats=a.repeats,route=a.route,results=results)
        if report['passed']:
            for result in results:
                result['repeat_qualified']=True
                (output/result['name']/'summary.json').write_text(json.dumps(result,indent=2))
        (output/'qualification.json').write_text(json.dumps(report,indent=2))
        print('QUALIFICATION '+json.dumps(dict(passed=report['passed'],independent_repeats=len(results))),flush=True)

if __name__=='__main__':main()
