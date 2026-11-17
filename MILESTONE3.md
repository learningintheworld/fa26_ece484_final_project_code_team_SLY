# Milestone 3：图像车道跟随操作

目标是实车根据前置相机完成低速车道跟随，**尚未完成实车验收**。电脑端 Highbay 此前已通过三次独立 49 m 组合弯路线验证，本轮新增一次 137.53 m 独立整圈验证，详见 [单侧边界报告](results/single_boundary/report.md)。旧 [双边界 Pure Pursuit](results/pure_pursuit/report.md) 和 [降速试验](results/highbay_control/report.md) 均保留。

Highbay 默认使用 RGB、CameraInfo、可靠单侧标线＋历史宽度及 Pure Pursuit；前视 3.5 m、速度上限 0.2 m/s，单侧和转弯时进一步降速。两侧都不可见仍停车锁定。相机几何使用模型参数，不依赖旧 bag 的 TF。e4 配置与仿真分开，真实数据研究已启用受误差预算约束的单侧功能；实车适配器默认只观察。

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

原图像控制对照使用 `./scripts/dev.sh lane-sim`，不能与 Pure Pursuit 节点同时运行。

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

调试图标注 `single_left` / `single_right` 时，中心线来自当前单侧观测和历史宽度。此时原始双边界状态可以是 `NO_SUPPORTED_EGO_PAIR`，应同时查看控制有效性与 `observation_mode`；它不等同于停车。没有可靠边界时则发布空路径和零速请求并锁定。

### Highbay 整圈试跑

整圈约 137.53 m，需要回到起点；不同于上述约 49 m 组合弯路线。当前 0.2 m/s、前视 3.5 m 已完成一次独立整圈，原 55.69 m 歧义已修复。0.3 m/s 试验在 42.87 m 因可靠单侧片段不足停车，因此默认仍为 0.2 m/s。见 [最新报告与录像](results/nearfield/report.md)；早期失败记录保留在 [旧整圈报告](results/full_lap/report.md)。单次复现：

```bash
./scripts/dev.sh exec python scripts/highbay_repeat.py --route full-lap --repeats 1 --speed 0.2 --lookahead 3.5 --output results/nearfield/my_attempt
```

先关闭已有项目仿真/感知/适配器，使用新的输出目录。脚本不会在故障停车后重新启用；失败时保留全部记录并退出。

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

- Highbay 允许可靠单侧＋历史宽度；近处矛盾、观测不足或两侧消失停车。宽度固定假设和 400 s / 70 m 上限仅为当前仿真配置，未证明适用于真实道路。
- 记忆丢失会保持禁用；旧 bag 中持续驾驶并不能模拟这种停车后的车辆画面。
- 图像控制模式不输出米制路径；Pure Pursuit 模式仅在模型投影有效时发布局部米制路径，无效时为空。
- 本地测试证明接口行为，不证明实车制动力、延迟、轮胎附着或转向映射正确。
- 所有改动和结果保留在本地，未提交 GitHub。
