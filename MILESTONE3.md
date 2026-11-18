# Milestone 3：图像车道跟随操作

目标是实车根据前置相机完成车道跟随，**尚未完成实车验收**。电脑端此前通过三次独立49米组合弯验证；最新ENet Highbay独立整圈进度137.649米，到终点882.418秒，最小包络余量0.116米，零失效控制帧并在终点停稳。最新结果见本地`results/highbay_cache_lap_20261118/`；历史数据、视频和报告不随Git上传。

仿真使用自己的e2相机几何，e4使用官方名义投影，配置分开。Highbay实时缓存基于实测`/odom`，最多1.5秒/0.3米、续行速度上限0.1 m/s；明确冲突、反馈过期或缓存到期仍停车，图像/请求超时未放宽。离线分段质量与身份交接、有限续行尚未接入实车。实车入口和模型切换以 [E4_QUICKSTART_ZH.md](E4_QUICKSTART_ZH.md) 为准。

## 1. 电脑端：已有 Jazzy Docker

所有终端在宿主机进入同一仓库：

```bash
cd /home/xinweiyu/Workspace/ECE484_Program/fa26_ece484_final_project_code_team_SLY
```

首次构建：

```bash
./scripts/dev.sh up
./scripts/dev.sh build
```

不要同时运行 bag player 和仿真。它们使用同一 ROS 域 84 和 `/clock`。

| 终端 | 直接执行 |
|---|---|
| 1：Highbay 仿真 | `./scripts/dev.sh sim` |
| 2：感知及 Pure Pursuit 请求 | `./scripts/dev.sh lane-sim-pp` |
| 3：仿真执行器出口，初始禁用 | `./scripts/dev.sh adapter` |
| 4：实时调试图 | `./scripts/dev.sh lane-view` |
| 5：明确启用 | `./scripts/dev.sh enable` |
| 5：停止并锁定 | `./scripts/dev.sh stop` |

上表`lane-sim-pp`默认使用OpenCV感知；最新ENet整圈用下方专用复现命令。原图像控制对照使用 `./scripts/dev.sh lane-sim`，不能与 Pure Pursuit 节点同时运行。

仿真默认无 GUI，终端 4 显示真实模拟相机与识别结果。需要 Gazebo GUI 时，在终端 1 使用：

```bash
./scripts/dev.sh sim 'gz_args:=-r -v1 '
```

输入无效、过期、反馈中断、出现第二个执行器发布者时禁用控制。恢复图像不会自动恢复驾驶。排除原因、车辆停止并确认仍对准正确车道后，才可运行：

```bash
./scripts/dev.sh lane-reset
./scripts/dev.sh enable
```

重置感知会重新选择当前走廊，不应在车辆移动中执行。结束顺序：先 `stop`，看到车速归零，再在终端 2、3、1 分别 Ctrl+C。无需停止整个 Docker。

### 完整路线与三次独立验证

此命令自行启动各次仿真、感知及适配器，因此先关闭上表的项目进程。每次使用新的输出目录；脚本会冻结源码指纹，任一次失败即停止计数。

```bash
./scripts/dev.sh exec python scripts/highbay_repeat.py --lookahead 3.5 --output results/single_boundary/my_repeats
```

手动单次全程：只启动终端 1、3，由实验器启动感知、启用和终点停车：

```bash
./scripts/dev.sh exec python scripts/highbay_experiment.py pp --name full_route --lookahead 3.5 --output results/single_boundary/my_trials
```

调试图标注 `single_left` / `single_right` 时，中心线来自当前单侧观测和历史宽度。此时原始双边界状态可以是 `NO_SUPPORTED_EGO_PAIR`，应同时查看控制有效性与 `observation_mode`；它不等同于停车。当前观测不足时，Highbay仅在有限缓存和冲突检查均通过时短时续行；缓存不可用或存在明确冲突则发布空路径、请求停车。

### Highbay ENet整圈复现

整圈约137.53米，需要回到起点。当前ENet v3、前视3.5米，启用自适应速度（直线上限1 m/s，单侧及弯道降速、加速度/加加速度限制），可靠图像订阅与有限缓存已完成一次独立整圈。单次复现：

```bash
./scripts/dev.sh exec python scripts/highbay_repeat.py --route full-lap --repeats 1 --speed 0.2 --lookahead 3.5 --adaptive-speed --perception-config src/sly_lane_following/config/highbay_enet.yaml --output results/my_current_enet_lap
```

先关闭其他项目仿真/感知/适配器，使用新的输出目录。`--speed 0.2`为基础配置；启用`--adaptive-speed`后由预瞄调速决定速度。脚本不会在故障停车后重新启用，失败时保留记录并退出。

### 仅仿真：复现速度与方向参数对照

这些是失败基线的复现入口，不是已通过参数。先仅启动上表终端 1（仿真）和终端 3（适配器），不要运行 `lane-sim` 或自行 `enable`；实验器负责新建感知进程、静止重置、明确启用及结束停车。

```bash
./scripts/dev.sh exec python scripts/highbay_experiment.py speed --output results/highbay_speed_review
./scripts/dev.sh exec python scripts/highbay_experiment.py angle --speed 0.2 --output results/highbay_angle_review
```

两条命令顺序执行，每次使用新输出目录。录像、逐帧日志、参数与源码快照会一起保存。赛道真值只用于诊断与验收，不参与转向。终止任意试验可在另一终端执行 `./scripts/dev.sh stop`，随后按既定顺序关闭进程。

## 2. 实车移植与操作

旧图像偏差控制的上车步骤已替换。使用 [精简中文实车指南](E4_QUICKSTART_ZH.md)：ENet v3 + e4 Pure Pursuit，先只观察，再完成现场参数验证后启用低速闭环。官方驱动不改；SLY适配器补齐制动确认、换挡确认和停稳退出流程。硬件参数继续留空，当前不是已验证实车版本。

## 4. 当前限制

- Highbay 允许可靠单侧＋历史宽度；暂缺当前观测时仅允许上述有限缓存续行，近处矛盾或缓存失效停车。宽度固定假设和 400 s / 70 m 上限仅为当前仿真配置，未证明适用于真实道路。
- 记忆丢失会保持禁用；旧 bag 中持续驾驶并不能模拟这种停车后的车辆画面。
- 图像控制模式不输出米制路径；Pure Pursuit 模式仅在模型投影有效时发布局部米制路径，无效时为空。
- 本地测试证明接口行为，不证明实车制动力、延迟、轮胎附着或转向映射正确。
- 代码与操作指南同步个人公开仓库；原始数据、实验报告、视频和构建产物仅留本地。
