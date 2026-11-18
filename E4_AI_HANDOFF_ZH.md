# GEM e4 实车排障交接：供网页版 ChatGPT 使用

更新时间：2026-11-17。技术基线：`59c0b5a`（Prepare reproducible e4 startup）。后续代码可能改变，请先以用户提供的提交号为准。

## 1. 先给接手 AI 的说明

你正在协助 Team SLY 排查 ECE484 GEM e4 实车部署问题。用户在车辆上使用网页版 ChatGPT；你不能假设能够访问车辆、执行命令或读取用户的本地文件。请根据完整终端输出判断，先定位故障层，再提供最少量、可以复制的命令。

- 区分已证实事实、推测和待验证事项；不要把没有输出当作检查通过。
- 每轮优先提供 1–3 条有区分度的检查命令，并解释结果意味着什么；避免反复整套重装。
- 先处理环境、消息与驱动，再处理感知、控制；不要通过改算法掩盖环境问题。
- 使用英文半角引号和 Markdown 代码块。用户此前多次因智能引号 `“ ” ‘ ’`、复制提示符或下划线格式改变而执行失败。
- 若需要改文件，先定位实际文件和版本，保留原文件；说明最小改动以及如何恢复。不要覆盖现场硬件参数。
- 不索取 SSH 私钥、GitHub Token 或完整环境变量转储。
- 本项目遵循：两侧不可见或真实选道歧义时停车；故障恢复后不自动重新启用。
- 不把 `hardware_verified` 直接改为 true；不猜测制动、油门、换挡参数；不放宽超时或强制发送运动命令来绕过失败。
- 禁用 PACMod、停止发布、关闭终端或 Ctrl+C 都不等于车辆制动。实际车辆操作必须遵循助教规程，驾驶员负责接管。

## 2. 仓库、设备和当前状态

公开仓库（无需账号、SSH 或 Token 即可克隆）：

https://github.com/learningintheworld/fa26_ece484_final_project_code_team_SLY

日常操作指南：[E4_QUICKSTART_ZH.md](E4_QUICKSTART_ZH.md)。本文件负责排障背景，不替代现场操作规程。

| 项目 | 已知信息 |
|---|---|
| 新部署目录 | `~/sly_public`，由公开仓库 git clone 得到 |
| 旧实车目录 | `/home/gem/fa26_ece484_final_project_code_team_SLY`，此前从 ZIP 解压，有现场临时修改；不是 Git 仓库 |
| 官方驱动工作空间 | 默认 `~/gem_ws`，实际路径尚需现场确认，可用 `SLY_DRIVER_WS` 指定 |
| 实车系统 | Ubuntu 22.04、ROS 2 Humble、Python 3.10；此前打印为 Python 3.10.12 |
| 曾观察到的 Torch | `1.13.1+cu117`；后续环境是否变化必须重新读取 |
| 官方 e4 计算平台 | Nuvo-8108GC、Xeon E-2278G、RTX A4000、64 GB RAM；这不是实车 nvidia-smi 实测结果 |
| 笔记本开发环境 | Jazzy Docker、Python 3.12；不能把它的 venv、build、install 复制到车辆 |
| 环境原则 | 复用车载已安装环境，不自动安装/升级 Torch、CUDA、驱动，不新建或上传 venv |

官方硬件资料：https://publish.illinois.edu/robotics-autonomy-resources/gem-e4/hardware/
官方驱动源码：https://github.com/UIUC-Robotics/gem_ws

### 2026-11-17 现场结果

- RGB、CameraInfo 可以接收，原图约 30 Hz；这不代表网络推理也是 30 Hz。
- ENet/Pure Pursuit 节点在 CPU 下启动，持续输出状态和控制请求。
- 夜间状态为 `NO_SUPPORTED_EGO_PAIR` / `NO_RELIABLE_SINGLE_BOUNDARY`，速度与转角请求均为 0。光照差是现场背景，不能只凭状态确定究竟是网络漏检还是候选选道失败。
- PACMod 五个反馈话题 echo 报消息类型 invalid；尚未证明硬件故障，也尚未验证车速、挡位和制动反馈。
- 录制日志仅确认订阅 RGB、CameraInfo、`/joy`、`/sly/lane/status`、`/sly/control/request`；不能宣称 PACMod 或 `/sly/control/status` 已录到。
- 未完成实车自主车道跟随、制动或换挡验收。
- Highbay 仿真已完成整圈，但使用模拟 e2 参数；不能替代实车 e4 验收。

## 3. 最新启动入口如何工作

首次克隆和准备：

```bash
git clone https://github.com/learningintheworld/fa26_ece484_final_project_code_team_SLY.git ~/sly_public
cd ~/sly_public
bash scripts/e4.sh setup
```

旧 ZIP 目录不要执行 git pull，也不要直接覆盖；新目录已有内容时不要重复克隆到里面。

| 命令后缀（均接 `bash scripts/e4.sh`） | 作用 |
|---|---|
| `setup` | PACMod 包/接口/Python 检查 → ENet 哈希/GPU 前向检查 → 系统 colcon 构建；失败停止，不安装软件、不启动驾驶 |
| `build` | 仅构建项目，不检查 GPU；首次 setup 仅因 GPU 失败时，可先 build 供 CPU 观察 |
| `camera` / `dbw` / `joy` | 官方 OAK 驱动 / 官方 PACMod 驱动 / 手柄输入节点 |
| `vision` | GPU 检查成功后启动感知；失败退出，不自动转 CPU |
| `vision-cpu` | 明确使用 CPU 观察；不证明实时性足够 |
| `observe` | 执行器适配器只观察，无执行器发布者 |
| `drive` | 实车适配器主动模式，仍受未验证配置和启用条件约束，不等于车辆立即运动 |
| `view` / `record` | 调试图像 / 录制到 `results/e4/run_日期_时间` |
| `enable` / `stop` / `reset` | 显式启用 / 请求正常停车 / 仅重置感知 |

脚本先记住 `SLY_PYTHON`（未设置则取当前 python3），清除继承的 ROS 包路径，再依次加载 Humble、官方驱动、本项目。`setup/build` 不加载本项目旧安装层，使用 `/usr/bin/colcon`；感知和适配器使用选定的 Python。构建和运行解释器必须与 Humble Python 3.10 兼容。

**脚本内部 source 不会改变调用它的父终端。** 即使 `bash scripts/e4.sh dbw` 曾成功，另一个终端手工执行 ros2 echo 前仍需加载环境。手工排障时在同一终端执行：

```bash
cd ~/sly_public
source /opt/ros/humble/setup.bash
source "${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"
source ./install/local_setup.bash
```

如项目尚未构建，暂不执行最后一行。工作空间加载顺序本身是正常的，不应把 source 官方工作空间直接判为故障根因。

## 4. 故障分类与最小诊断

### A. 权重加载失败

权重：`models/enet_v3.pth`，1,619,847 字节。SHA-256：

```text
5b6b3c12d4ba1609a6107f6241a3f42f6ce9a8b7112fcacb436be0960452a547
```

```bash
sha256sum models/enet_v3.pth
```

此前实车哈希完全相同。`Unsupported operand 71` 曾由旧 Torch 的 weights-only 加载器不能读取浮点元数据引起，不能据此判定文件损坏。

最新 `neural.py` 先安全加载；只有该精确哈希且错误为 `Unsupported operand 71` 才兼容重试。未知权重不进入宽松 pickle 加载。不要让用户对任意文件直接设置 `weights_only=False`。

最新代码仍出现相同错误时，先确认实际导入位置是否还是旧 ZIP 目录：

```bash
"${SLY_PYTHON:-python3}" -c 'import sys; import sly_lane_following.neural as n; print(sys.executable); print(n.__file__)'
```

### B. CUDA/GPU 失败

已出现过：`Could not load library libcublasLt.so.12`。它说明所需动态库无法加载，不证明没有 GPU。此前显示 cu117，而错误请求 CUDA 12 库，环境混用只是待验证假设。

```bash
nvidia-smi
"${SLY_PYTHON:-python3}" -m torch.utils.collect_env
```

结合 GPU、驱动、Python 路径、Torch 路径及 CUDA runtime 判断。不要把 nvidia-smi 的 CUDA Version 当作当前 Torch 的编译 runtime，不要创建 `.so.11 → .so.12` 假链接，不要先升级整套系统。

`GPU_FORWARD_OK` 只证明真实模型一次推理成功；仍须测相机采集到控制请求的延迟、持续处理频率及录制同时运行时的负载。1.6 MB 权重不等于计算必然足够快。当前图像超时默认 0.5 秒；不要为迁就性能直接放宽。

仅在停车观察、完成项目构建后，用 `vision-cpu` 排障。GPU 与 CPU 感知节点不可同时运行。

### C. PACMod 消息类型 invalid

现场错误类型包括 `pacmod2_msgs/msg/VehicleSpeedRpt`、`GlobalRpt`、`SystemRptFloat` 和 `SystemRptInt`。

先加载本节前述环境，再逐项执行：

```bash
ros2 pkg prefix pacmod2_msgs
ros2 interface show pacmod2_msgs/msg/VehicleSpeedRpt
"${SLY_PYTHON:-python3}" -c 'from pacmod2_msgs.msg import VehicleSpeedRpt; print("PACMOD_IMPORT_OK")'
```

- 包定位失败：检查官方工作空间路径及是否安装该包。只有源代码目录存在不等于构建安装完成。
- 包存在但接口失败：检查安装产物与版本；不是相机问题。
- 接口存在但 Python 导入失败：检查解释器、PYTHONPATH、消息类型支持与 ABI；不要把 Jazzy 编译产物用于 Humble。
- 三项通过后，再判断驱动是否真的发布反馈：

```bash
ros2 topic info /pacmod/vehicle_speed_rpt
ros2 topic echo /pacmod/vehicle_speed_rpt --once
```

发布者为 0 时检查 DBW 进程、话题名称和通信域；有发布者但 echo 等待时，再检查驱动日志、CAN 通信与 QoS。等待可按 Ctrl+C 结束，不代表成功。关闭 DBW 终端可能造成驱动退出，但不能单独解释本地消息类型无法导入。

项目使用的官方启动文件是 `basic_launch/dbw_only.launch.py`，其包含 `pacmod2/launch/pacmod2.launch.xml`。不要在已有驱动运行时重复启动；不要随意重编译全部官方工作空间。

### D. 感知无效、请求为零

```bash
ros2 topic echo /sly/lane/status --once
ros2 topic echo /sly/control/request --once
```

| 状态 | 含义与下一步 |
|---|---|
| `NO_SUPPORTED_EGO_PAIR` | 没有被接受的当前车道双边界组合；不等于图像没到达或掩膜完全为空 |
| `NO_RELIABLE_SINGLE_BOUNDARY` | 当前观测不能可靠支持单侧推算；不能直接延长时限 |
| `STALE_IMAGE_HEADER` | 图像时间戳过期；检查时间基准、处理耗时与积压 |
| `CAMERA_INFO_FRAME_MISMATCH` | 图像和 CameraInfo 坐标系不匹配；先核对来源 |
| speed=0 | 当前请求停车；不能据此证明物理车辆已制动 |

让用户提供原图与 `/sly/lane/debug_image` 截图、相机位置/光照、完整状态。先区分白线漏检、候选连接失败、身份关联/选道歧义、投影/目标支持不足，再考虑改对应模块。

RGB：`/oak/rgb/image_raw`；内参：`/oak/rgb/camera_info`。输入只使用 RGB 和 CameraInfo，不使用 RGB-D、LiDAR、GNSS 或 IMU 融合。采用官方 e4/DepthAI 名义几何，不要求补录 TF；这不代表实车外参经过实测。

### E. 适配器没有状态或车辆不动

```bash
ros2 topic info /sly/control/status
ros2 topic echo /sly/control/status --once
```

`observe` 本来就不发布执行器命令。`drive` 默认也不自动启用。`adapter.py` 没有模块运行保护入口，因此不要擅自改成 `python -m sly_lane_following.adapter`；使用提供的脚本。

配置基线：`hardware_verified: false`，速度上限 0.2 m/s，前轮转角限幅 0.25 rad；`stop_brake`、`shift_brake`、`max_throttle`、`speed_kp` 均为空。手柄按钮索引 4 是待现场确认的配置。不能通过填猜测值解除阻塞。

常见控制原因：`NO_REQUEST`、`STALE_REQUEST`、`STALE_FEEDBACK`、`MANUAL_OVERRIDE`、`COMPETING_PUBLISHER`、`STOP_REQUEST`、`COMMAND_LIMIT`。先读完整原因；不得将官方 joystick 控制器与 SLY 执行器适配器同时用于发布车辆命令。

### F. 录制与 Git

- 录制必须看实际订阅日志和结束后的 `ros2 bag info 具体bag目录`；命令中列了话题不代表消息已录到。
- 录制终端 Ctrl+C 正常保存；不要为排障删除已有 bag。
- 只有 git clone 的目录可 pull。ZIP 目录出现 `not a git repository` 属于预期。
- 更新前确认所在目录，使用 `git pull --ff-only`；存在本地修改时保留，不提供强制 reset/clean。

## 5. 接手 AI 可查阅的关键源码

以下均为仓库根目录相对路径：

- `scripts/e4.sh`：实车启动、环境加载、解释器选择。
- `scripts/e4_check.py`：只读预检查；不安装软件、不发布控制命令。
- `src/sly_lane_following/sly_lane_following/neural.py`：权重兼容与推理预处理。
- 同目录 `node.py`：图像/CameraInfo 配对、时效检查、状态与控制请求。
- 同目录 `core.py`、`real_corridor.py`：候选边界、当前车道选择与历史关联。
- 同目录 `geometry.py`：地面投影、单侧宽度推算和 Pure Pursuit。
- 同目录 `adapter.py`、`control.py`：执行器适配、启停流程和请求门控。
- `src/sly_lane_following/config/real_enet.yaml`、`e4_pure_pursuit.yaml`、`e4_adapter.yaml`：实车感知、几何控制、硬件参数。

本次部署验证：21 项针对性测试通过；权重张量一致，单线程 CPU 前向完全一致；笔记本 RTX 5060 / Torch 2.7.1+cu128 GPU 前向成功。GPU 原有 max-unpool 运算非确定，不能要求重复 GPU 推理逐像素完全一致。上述结果不证明车载 Torch 1.13、A4000 或 PACMod 已通过实车验收；旧加载器异常分支使用模拟测试覆盖。

## 6. 用户发给网页 AI 的模板

复制下面这段，加上本文件和终端文本：

> 请先阅读附件 E4_AI_HANDOFF_ZH.md。我们在 GEM e4 上运行 Team SLY 车道跟随代码。请先判断属于环境、传感器/驱动、感知还是控制问题，再给最少的诊断命令。不要先重装环境、修改算法或解除停车保护。我会提供每条命令的完整结果。
>
> 当前目录：
> 当前提交号（git rev-parse --short HEAD）：
> 我执行的完整命令：
> 从启动开始的完整输出/Traceback：
> 已启动且仍运行的终端（camera/dbw/joy/vision/observe 等）：
> 使用 GPU 还是 CPU：
> 车辆是否停稳、是否只观察、是否有助教指导：
> SLY_DRIVER_WS / SLY_PYTHON（若设置）：
> 与上次相比做过什么修改：
>
> 若你无法读取公开仓库，请明确告诉我需要上传哪几个源文件，不要猜测代码实现。
