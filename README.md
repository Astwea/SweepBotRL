# SweepBotRL

**SweepBotRL** is a full-stack autonomous cleaning robot system built on a differential-drive platform. It combines high-performance reinforcement learning training, multi-agent simulation, embedded control firmware, and ROS 2 integration for real-world deployment.
📖 [点击查看中文版说明](./README_CN.md)
---

## 🚀 Features

- 🧠 **Reinforcement Learning**: Trains navigation policy using IsaacLab + RL-Games
- 🔁 **Multi-agent Support**: Up to 8192 simulated environments for scalable training
- 🤖 **Differential-Drive Robot**: Designed for real-world cleaning tasks
- 🛠️ **Embedded Control**: STM32 + FreeRTOS-based firmware with PID and PWM control
- 🧭 **ROS 2 Integration**: Full support for cmd_vel, sensors, and navigation stack
- 🧪 **Simulation to Real Transfer**: Designed for consistency between sim and hardware

---

## 📁 Project Structure

```bash
SweepBotRL/
├── source/              # RL environment, simulation and control logic
│   └── Mydog/
│       ├── tasks/direct/mydog_marl/         # Custom IsaacLab env
│       └── control/                         # Embedded PID control logic
├── firmware/           # STM32Cube HAL/FreeRTOS firmware
├── ros2_ws/            # ROS 2 workspace and nodes
├── configs/            # RL configuration YAMLs
├── scripts/            # Training, evaluation, model conversion
├── docs/               # Architecture, hardware design, communication protocol
├── requirements.txt
├── README.md
├── LICENSE
└── .gitignore
```

---

## 🧠 Training the RL Agent

1. Clone IsaacLab and follow setup instructions  
2. Clone this repo and install dependencies:

```bash
pip install -r requirements.txt
```

3. Start training:

```bash
cd source/Sweap_Task
python scripts/rl_games/train.py --task=Template-Mydog-Marl-Direct-v0
```

---

## 🤖 Deploying to Hardware

- Use STM32CubeIDE to open `firmware/STM32_Project`
- Compile and flash to the robot controller board
- Use ROS 2 nodes under `ros2_ws/src/mydog_ros2/` for command bridging

---

## 📡 ROS 2 Interface

- `cmd_vel` for velocity control  
- `laser_scan` or `odom` as optional inputs  
- Launch file: `ros2 launch mydog_ros2 bringup.launch.py`

---

## 📄 License

This project is licensed under the MIT License.  
See [LICENSE](./LICENSE) for details.

---

## ✨ Acknowledgements

Built with:

- [IsaacLab](https://github.com/NVIDIA-Omniverse/IsaacLab)
- [rl-games](https://github.com/Denys88/rl_games)
- STM32Cube HAL + FreeRTOS
- ROS 2 Humble

---

## 🔗 Project Summary

> **SweepBotRL** is a reinforcement learning-powered differential-drive cleaning robot, designed for scalable simulation and real-world deployment.
