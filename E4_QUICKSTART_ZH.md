# GEM e4 实车操作

使用个人公开仓库，实车目录为 `~/sly_public`。复用 Ubuntu 22.04、ROS Humble 和已有 PyTorch，不安装或升级依赖，不创建 venv。

## 首次部署

新目录克隆，保留旧项目及其现场修改：

```bash
git clone https://github.com/learningintheworld/fa26_ece484_final_project_code_team_SLY.git ~/sly_public
cd ~/sly_public
bash scripts/e4.sh setup
```

`setup` 加载官方驱动，检查 PACMod 消息、ENet 权重与一次真实 GPU 推理，成功后构建。不会启动车辆；失败时按提示修复现有环境，不会自动安装软件或降级到 CPU。权重已经包含在仓库，无需保留旧的手工加载补丁。

默认官方工作空间为 `~/gem_ws`，默认使用当前 `python3`。若实际路径不同，在每个操作终端设置对应变量（填写真实路径）：

```bash
export SLY_DRIVER_WS=/实际官方工作空间路径
export SLY_PYTHON=/已有Python环境/bin/python3
```

已有环境可先激活再运行入口；解释器必须与车上 Humble 的 Python 3.10 兼容。不要复制笔记本的虚拟环境、build 或 install。

## 后续更新

```bash
cd ~/sly_public
git pull --ff-only
bash scripts/e4.sh build
```

若 Git 提示本地修改冲突，先保留现场修改，不强制覆盖。`build` 不重复 GPU 检查；环境改变后重新运行 `setup`。

## 每个新终端

```bash
cd ~/sly_public
```

脚本自动依次加载 Humble、官方工作空间、本项目，并设置 `VEHICLE_NAME=e4`；沿用车辆通信域。以下已有相机或驱动进程不重复启动。

### 终端 1：前置 OAK

启动官方相机驱动，项目使用 RGB 与 CameraInfo。

```bash
bash scripts/e4.sh camera
```

### 终端 2：PACMod 驱动

```bash
bash scripts/e4.sh dbw
```

### 终端 3：手柄输入

只发布手柄输入。SLY 控制时不同时运行 `dbw_joystick.launch.py` 或 `pacmod2_game_control`。

```bash
bash scripts/e4.sh joy
```

### 终端 4：GPU 感知与 Pure Pursuit

启动前执行一次 GPU 推理检查；失败直接退出。本命令不直接发布执行器命令。

```bash
bash scripts/e4.sh vision
```

只有排障时才改用下面命令，不与 GPU 感知同时运行；CPU 实时驾驶性能未验证。如果首次 `setup` 仅因 GPU 失败而未构建，先运行 `bash scripts/e4.sh build`。

```bash
bash scripts/e4.sh vision-cpu
```

### 终端 5：只观察执行器适配器

读取请求和反馈，不发布执行器命令。

```bash
bash scripts/e4.sh observe
```

### 终端 6：显示识别结果

```bash
bash scripts/e4.sh view
```

### 终端 7：录制

保存到 `results/e4/run_日期_时间`；Ctrl+C 结束保存。缺失或无法订阅的话题不会自动补齐。

```bash
bash scripts/e4.sh record
```

## 现场验证后才启用闭环

`setup` 通过不代表车辆已可自主驾驶。必须先确认实时感知延迟、车辆反馈、转向方向、制动、换挡及手柄按钮，并填写 `e4_adapter.yaml` 的实测参数。当前未验证参数保持为空，实车速度上限仍为 0.2 m/s。

终端 5：Ctrl+C 退出 `observe` 后运行：

```bash
bash scripts/e4.sh drive
```

终端 8：车辆停稳、空挡、PACMod 禁用，驾驶员按住已确认的使能按钮后启用：

```bash
bash scripts/e4.sh enable
```

正常停止，保持适配器运行直到停稳、完成空挡和禁用流程：

```bash
bash scripts/e4.sh stop
```

车辆停稳并退出控制后，再在其他终端按 Ctrl+C。停进程或禁用 PACMod 不等于制动。

仅在停稳、确认车道位置后重置识别；不会自动恢复驾驶：

```bash
bash scripts/e4.sh reset
```
