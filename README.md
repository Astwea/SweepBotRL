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
├── firmware/           # STM32Cube HAL/FreeRTOS firmware
├── ros2_ws/            # ROS 2 workspace and nodes
├── scripts/            # Training, evaluation, model conversion
├── Docs/               # Architecture, hardware design, communication protocol
├── README.md
├── LICENSE
└── .gitignore
```

---

## 🧠 Training the RL Agent

1. Clone IsaacLab and follow setup instructions  
2. Clone this repo and install any Python dependencies required by IsaacLab and rl-games.

3. Start training:

```bash
cd source/Sweap_Task
python scripts/rl_games/train.py --task=Template-Mydog-Marl-Direct-v0
```

---

## 🤖 Deploying to Hardware

- Use STM32CubeIDE to open `firmware/pwm_test.ioc`
- Compile and flash to the robot controller board
- Use ROS 2 packages under `ros2_ws/src/run_saodi/` and `ros2_ws/src/wit_ros2_imu/` for hardware integration
- See [Docs/firmware.md](Docs/firmware.md) for a full firmware build guide

---

## 🛠️ Building the ROS 2 Workspace

1. Install ROS 2 Humble and source the environment:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Install package dependencies with `rosdep`:

   ```bash
   cd ros2_ws
   rosdep install --from-paths src -r -y
   ```

3. Build the workspace using `colcon`:

   ```bash
   colcon build
   ```

4. Source the workspace before running the nodes:

   ```bash
   source install/setup.bash
   ```

---

## 📡 ROS 2 Interface

- `cmd_vel` for velocity control  
- `laser_scan` or `odom` as optional inputs  
- Launch file: `ros2 launch run_saodi start_robot.launch.py`
- IMU launch: `ros2 launch wit_ros2_imu imu.launch.py`

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
