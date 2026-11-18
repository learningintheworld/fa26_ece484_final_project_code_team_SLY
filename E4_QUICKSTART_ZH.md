# GEM e4 实车操作

项目目录按今天实车路径：`~/fa26_ece484_final_project_code_team_SLY`。
官方驱动暂按 `~/gem_ws`；若实际位置不同，每个终端先执行 `export SLY_DRIVER_WS=实际工作空间路径`。入口自动加载 Humble、官方驱动和本项目，保持车辆原有 ROS 通信域。

## 使用前提

- 下面的 `scripts/e4.sh` 是实车短命令入口，使用前需同步最新代码到车上。
- 保留今天车上已完成的 PyTorch 1.13 权重加载兼容修复。当前仓库仍使用 `weights_only=True`，重新覆盖代码可能再次出现 `Unsupported operand 71`。
- `vision` 暂用 CPU 绕过缺少 `libcublasLt.so.12` 的问题，仅用于观察；实时速度尚未验证。
- PACMod 消息环境、制动及换挡参数尚未验证。当前只运行 `observe`；`drive` 留待现场验证后使用，不填写猜测参数。

## 准备：更新后构建一次

在已同步本次文件的项目目录运行；ZIP 解压目录不使用 `git pull`，也不要覆盖车上已有兼容修复。

```bash
cd ~/fa26_ece484_final_project_code_team_SLY
bash scripts/e4.sh build
```

## 每个新终端先进入项目

如使用 Python 虚拟环境，先激活车上已成功运行网络的同一环境。

```bash
cd ~/fa26_ece484_final_project_code_team_SLY
```

## 终端 1：前置 OAK 相机

启动相机驱动，项目消费 RGB 与 CameraInfo。已有相机进程时跳过。

```bash
bash scripts/e4.sh camera
```

## 终端 2：PACMod 驱动

已有驱动时跳过，不重复启动。

```bash
bash scripts/e4.sh dbw
```

## 终端 3：手柄输入

只启动输入节点。SLY 控制时不能同时运行官方 `dbw_joystick.launch.py` 或 `pacmod2_game_control`，避免争用执行器。

```bash
bash scripts/e4.sh joy
```

## 终端 4：ENet 感知与 Pure Pursuit

生成识别结果和控制请求，本命令不直接控制车辆。

```bash
bash scripts/e4.sh vision
```

## 终端 5：执行器适配器（当前只观察）

接收控制请求和车辆反馈，不发送执行器命令。

```bash
bash scripts/e4.sh observe
```

## 终端 6：识别画面

```bash
bash scripts/e4.sh view
```

## 终端 7：录制

保存到 `results/e4/run_日期_时间`。录制相机、车辆反馈、手柄及项目消息；不存在或无法订阅的话题不会自动补齐。结束时在本终端按 Ctrl+C 保存。

```bash
bash scripts/e4.sh record
```

## 现场验证通过后：启用实车闭环

仅在车辆反馈、控制时延、转向方向、制动、换挡与使能按钮验证完成，并正确填写 `e4_adapter.yaml` 后使用。当前配置仍未验证，速度上限为 0.2 m/s，不沿用仿真 1 m/s。

终端 5 按 Ctrl+C 退出 `observe`，改为：

```bash
bash scripts/e4.sh drive
```

终端 8：车辆停稳、空挡、PACMod 禁用，驾驶员按住已确认的使能按钮后执行：

```bash
bash scripts/e4.sh enable
```

正常停止，保持适配器运行至车辆停稳并完成空挡、禁用流程：

```bash
bash scripts/e4.sh stop
```

车辆停稳、退出控制后，才在其他终端按 Ctrl+C。停止进程或禁用 PACMod 不等于车辆制动。

需要重新初始化识别时，仅在停稳且确认车道位置后执行；不会自动恢复驾驶：

```bash
bash scripts/e4.sh reset
```
