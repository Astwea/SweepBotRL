# SweepBotRL

**SweepBotRL** 是一个基于差速驱动平台的全栈式自主清扫机器人系统，集成了高性能强化学习训练、多智能体仿真、嵌入式控制固件和 ROS 2 集成，支持从仿真到真实部署的完整流程。

[English Version](./README.md#english-version)

---

## 🚀 功能亮点

- 🧠 **强化学习导航**：基于 IsaacLab 和 RL-Games 训练机器人导航策略
- 🔁 **多智能体支持**：支持最多 8192 个仿真实例并行训练
- 🤖 **差速清扫机器人**：面向实际清扫任务设计
- 🛠️ **嵌入式控制固件**：基于 STM32 + FreeRTOS 的 PWM 与 PID 控制系统
- 🧭 **ROS 2 集成**：完整的 cmd_vel 接口及导航控制模块
- 🧪 **仿真到实机迁移**：确保策略在仿真和现实中的一致性

---

## 📁 项目结构

```bash
SweepBotRL/
├── source/              # 强化学习环境、控制逻辑与仿真配置
│   └── Mydog/
│       ├── tasks/direct/mydog_marl/         # 自定义 IsaacLab 环境
│       └── control/                         # 嵌入式 PID 控制逻辑
├── firmware/           # STM32Cube HAL/FreeRTOS 固件工程
├── ros2_ws/            # ROS 2 工作空间与节点
├── configs/            # RL 训练相关 YAML 配置
├── scripts/            # 训练、评估、模型转换脚本
├── docs/               # 系统结构、电路说明、通信协议文档
├── requirements.txt
├── README.md
├── LICENSE
└── .gitignore
```

---

## 🧠 训练机器人策略

1. 克隆 IsaacLab 并完成依赖配置  
2. 克隆本仓库并安装依赖：

```bash
pip install -r requirements.txt
```

3. 开始训练：

```bash
python scripts/train.py
```

---

## 🤖 部署到实体机器人

- 使用 STM32CubeIDE 打开 `firmware/STM32_Project`
- 编译并烧录至控制板
- 使用 ROS 2 启动 `ros2_ws/src/mydog_ros2/` 中的控制节点

---

## 📡 ROS 2 接口

- `cmd_vel` 控制线速度与角速度  
- 可选传感器：`laser_scan` 或 `odom`  
- 启动指令：`ros2 launch mydog_ros2 bringup.launch.py`

---

## 📄 许可证

本项目使用 MIT 开源许可证，详见 [LICENSE](./LICENSE)。

---

## ✨ 致谢

本项目构建于以下开源工具之上：
- [IsaacLab](https://github.com/NVIDIA-Omniverse/IsaacLab)
- [rl-games](https://github.com/Denys88/rl_games)
- STM32Cube HAL + FreeRTOS
- ROS 2 Humble

---

## 🔗 项目概述

> **SweepBotRL** 是一个由强化学习驱动的差速清扫机器人，支持大规模仿真与实机部署的完整解决方案。

---

## English Version

For English description, see the top of [README.md](./README.md#sweepbotrl).