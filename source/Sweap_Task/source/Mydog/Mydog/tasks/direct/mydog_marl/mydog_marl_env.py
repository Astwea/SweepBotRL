import gymnasium as gym
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import ContactSensor
from isaaclab.sim.spawners.from_files import spawn_ground_plane,spawn_from_usd, UsdFileCfg, GroundPlaneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.assets import AssetBaseCfg
from .mydog_marl_env_cfg import MydogMarlEnvCfg

try:
    from isaacsim.util.debug_draw import _debug_draw as debug_draw
    debug_draw_available = True
except ImportError:
    debug_draw_available = False
    print("Debug draw is not available. Please check your Isaac Sim installation.")

import math
from isaaclab.utils.math import quat_from_angle_axis
from torch.utils.tensorboard import SummaryWriter
import time

import torch
import math
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import numpy as np

def define_markers(path, idx) -> VisualizationMarkers:
    """Define markers with various different shapes."""
    if idx == 0:
        markers={
                "sphere": sim_utils.SphereCfg(
                    radius=0.5,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                )
            }
    elif idx == 1:
        markers={
                "arrow": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.5, 0.5, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
                )
            }
    elif idx == 2:
        markers={
                "arrow1": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.5, 0.5, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                )
            }
    marker_cfg = VisualizationMarkersCfg(
        prim_path=f"/Visuals/myMarkers/{path}",
        markers=markers
    
    )
    return VisualizationMarkers(marker_cfg)

class MydogMarlEnv(DirectRLEnv):
    # 1. 配置初始化
    cfg: MydogMarlEnvCfg

    def __init__(self, cfg: MydogMarlEnvCfg, render_mode: str | None = None, **kwargs):
        # 1.1 初始化父类
        super().__init__(cfg, render_mode, **kwargs)

        # 1.2 初始化动作存储
        # - 记录当前和前一次的动作，用于计算奖励和动态控制
        self._actions = torch.zeros(self.num_envs, 2, device=self.device)  # (线速度, 角速度)
        self._previous_actions = torch.zeros(self.num_envs, 2, device=self.device)


        self.arrow_visual = define_markers(path="arrows", idx=1)
        self.target_arrow_visual = define_markers(path="target_arrows", idx=2)

        self._commands = torch.zeros(self.num_envs, 2, device=self.device)  # (x, y, z)
        self._trajectories = torch.zeros(self.num_envs, self.cfg.num_waypoints*self.cfg.num_interp, 2, device=self.device)
        self._current_wp_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._prev_wp_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.seed = 10
        self.epoch = 0

        self.dist_to_target = None
        self._prev_dist_to_target = torch.zeros(self.num_envs, device=self.device)
        self.target_orientations = None
        self.positions = self._robot.data.root_state_w[:, :2]
        self.last_pos = self._robot.data.root_state_w[:, :2]
        self.cos_phi = torch.zeros(self.num_envs, 1, device=self.device)
        self.sin_phi = torch.zeros(self.num_envs, 1, device=self.device)
        # 1.3 初始化日志记录
        # - 记录每个回合中的关键性能指标
        self.writer = SummaryWriter(log_dir=f"{cfg.log_dir}/{time.strftime('%Y-%m-%d_%H-%M-%S')}/summary")
        self.begin_time = time.time()
        self.global_step = 0
        self._episode_sums = {
            key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device) 
            for key in ["tracking_reward","direction_reward","goal_bias","action_rate_penalty","action_mag_penalty"]
        }
        self.joint_idx, _ = self._robot.find_joints(['left_wheel_joint', 'right_wheel_joint'])
        self.positions = self._robot.data.root_state_w[:, :2]
        self.yaw  = self._robot.data.root_state_w[:, 3:7]
        if debug_draw_available:
            self.debug_draw = debug_draw.acquire_debug_draw_interface()
        else:
            self.debug_draw = None
        self.count = 0

    # 2. 场景设置
    def _setup_scene(self):
        # 2.1 初始化机器人模型
        self._robot = Articulation(self.cfg.robot)
        # spawn_ground_plane(prim_path="/World/ground", cfg=Envconfig())
        spawn_ground_plane(
            prim_path="/World/ground",
            cfg=GroundPlaneCfg(color=(0.5, 0.5, 0.5),
                               size=(300, 300))
        )
        self.scene.articulations["robot"] = self._robot
        
        # 2.2 初始化接触传感器
        # self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        # self.scene.sensors["contact_sensor"] = self._contact_sensor
        # self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        # self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        # self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)
        
        # 2.3 克隆环境
        # - 创建多个环境实例，提高并行效率
        self.scene.clone_environments(copy_from_source=False)
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)
    # 3. 物理步前处理

    def quaternion_to_forward_vector(self, quaternions):
        # 提取四元数 (qx, qy, qz, qw)
        qx, qy, qz, qw = quaternions[:, 0], quaternions[:, 1], quaternions[:, 2], quaternions[:, 3]

        # 计算正前方方向向量
        forward_x = 2 * (qx * qz + qw * qy)
        forward_y = 2 * (qy * qz - qw * qx)
        forward_z = 1 - 2 * (qx ** 2 + qy ** 2)

        return torch.stack([forward_x, forward_y, forward_z], dim=1)


    def quaternion_to_yaw(self,quat):
        """
        计算四元数 (w, x, y, z) 对应的 yaw 角度
        """
        w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return yaw

    def compute_orientation(self, pos, target):
        # 计算目标方向
        direction_to_target = target - pos
        yaw_target = torch.atan2(direction_to_target[:, 1], direction_to_target[:, 0])
        qx = torch.zeros_like(yaw_target)
        qy = torch.zeros_like(yaw_target)
        qz = torch.sin(yaw_target / 2)
        qw = torch.cos(yaw_target / 2)
        return torch.stack([qw, qx, qy, qz], dim=1)

    def _pre_physics_step(self, actions: torch.Tensor):
        # 3.1 缓存当前动作
        # - 记录输入的动作，以便后续使用
        self._actions = actions.clone().clamp(-1.0, 1.0)  # 限制动作范围

    def adjust_yaw_with_velocity_tensor(self, quat, vx):
        # 提取四元数分量
        w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
        
        # 计算当前的 yaw
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        
        # 修正 yaw
        reversing_mask = vx < 0
        yaw[reversing_mask] = (yaw[reversing_mask] + math.pi) % (2 * math.pi)

        # 计算新的四元数
        half_yaw = yaw / 2
        sin_half_yaw = torch.sin(half_yaw)
        cos_half_yaw = torch.cos(half_yaw)

        # 直接修改原始四元数中的 z 和 w
        adjusted_quat = quat.clone()
        adjusted_quat[:, 3] = sin_half_yaw
        adjusted_quat[:, 0] = cos_half_yaw

        return adjusted_quat

    # 4. 应用动作到物理引擎
    def _apply_action(self):
        # 4.1 将动作映射为左右轮速度
        linear_vel, angular_vel = self._actions[:, 0], self._actions[:, 1]
        
        # # 4.2 计算左右轮速度
        left_wheel_vel = linear_vel - angular_vel * self.cfg.wheel_base / 2
        right_wheel_vel = linear_vel + angular_vel * self.cfg.wheel_base / 2
        
        # # 4.3 设置机器人关节速度目标
        wheel_radius = 0.0357  # 假设轮子半径是 5cm
        left_wheel_vel /= wheel_radius
        right_wheel_vel /= wheel_radius
        # # 只选择左右轮的关节
        #    # 设置关节速度目标
        # zero_joint_vel = torch.zeros_like(left_wheel_vel)
        joint_vels = torch.stack([left_wheel_vel, right_wheel_vel],dim=1)
        # 只设置左右轮的速度
        self._robot.set_joint_velocity_target(joint_vels, joint_ids=self.joint_idx)
        self.positions = self._robot.data.root_state_w[:, :2]
        self.yaw  = self._robot.data.root_state_w[:, 3:7]
        positions = torch.tensor([[x, y, 0.5] for x, y in self.positions])
        self.target_orientations = self.compute_orientation(self.positions, self._trajectories[range(self.num_envs), self._current_wp_idx])
        angle_diff = self.quaternion_to_yaw(self.target_orientations) - self.quaternion_to_yaw(self.yaw)
        self.cos_phi = torch.cos(angle_diff).unsqueeze(1)
        self.sin_phi = torch.sin(angle_diff).unsqueeze(1)
        linear_vel = self._robot.data.root_lin_vel_b[:, 0]
        self.yaw = self.adjust_yaw_with_velocity_tensor(self.yaw, linear_vel)
        self.arrow_visual.visualize(translations=positions, orientations=self.yaw, marker_indices=torch.zeros(self.num_envs, dtype=torch.int64))
        self.target_arrow_visual.visualize(translations=positions, orientations=self.target_orientations, marker_indices=torch.zeros(self.num_envs, dtype=torch.int64))

        self.last_pos = self.positions

    # 5. 获取观测数据
    def _get_observations(self) -> dict:
        self._previous_actions = self._actions.clone()

        future_len = 4  # 当前点 + 未来3个轨迹点
        traj_points = []
        traj_deltas = []
        traj_len = self._trajectories.shape[1]
        next_idx = torch.clamp(self._current_wp_idx + 1, max=traj_len - 1)
        dist_curr = torch.norm(self.positions - self._trajectories[range(self.num_envs), self._current_wp_idx], dim=1)
        dist_next = torch.norm(self.positions - self._trajectories[range(self.num_envs), next_idx], dim=1)

        # 如果离下一个更近，就推进 index
        advance_condition = (dist_next + 0.05 < dist_curr) & (dist_curr < 0.4)

        self._current_wp_idx = torch.where(
            advance_condition,
            torch.clamp(self._current_wp_idx + 1, max=traj_len - 1),
            self._current_wp_idx
        )
        for i in range(future_len):
            idx = torch.clamp(self._current_wp_idx + i, max=self._trajectories.shape[1] - 1)
            point = self._trajectories[torch.arange(self.num_envs), idx]
            traj_points.append(point)
            if i > 0:
                prev_idx = torch.clamp(self._current_wp_idx + i - 1, max=self._trajectories.shape[1] - 1)
                prev_point = self._trajectories[torch.arange(self.num_envs), prev_idx]
                traj_deltas.append(point - prev_point)
        
        # 目标点信息
        traj_feats = torch.cat(traj_points, dim=-1)       # [num_envs, 2 * future_len]
        traj_delta_feats = torch.cat(traj_deltas, dim=-1) # [num_envs, 2 * (future_len - 1)]

        # 当前与最近目标点之间的误差向量（可以理解为目标误差）
        relative_error = traj_points[0] - self.positions  # [num_envs, 2]

        # yaw 方向 → cos/sin(yaw)
        yaw_tensor = self.quaternion_to_yaw(self.yaw)
        cos_yaw = torch.cos(yaw_tensor).unsqueeze(1)
        sin_yaw = torch.sin(yaw_tensor).unsqueeze(1)

        obs = torch.cat([
            self._robot.data.root_lin_vel_b[:, :2],   # 线速度 (2,)
            self._robot.data.root_ang_vel_b[:, 2:],   # 角速度 (1,)
            self._actions,                            # 当前动作 (2,)
            self._previous_actions,                   # 上一步动作 (2,)
            relative_error,                           # 当前误差 (2,)
            traj_feats,                               # 当前+未来轨迹点 (2×4)
            traj_delta_feats,                         # 轨迹趋势 (2×3)
            cos_yaw, sin_yaw                          # 姿态信息 (2,)
        ], dim=-1)

        return {"policy": obs}

    # 6. 计算奖励
    def _get_rewards(self) -> torch.Tensor:
        self.global_step += 1

        # === 轨迹误差 ===
        current_target = self._trajectories[torch.arange(self.num_envs), self._current_wp_idx]
        next_target = self._trajectories[torch.arange(self.num_envs), self._current_wp_idx + 1]
        
        tracking_error = torch.norm(self.positions - current_target, dim=1)
        tracking_reward = torch.exp(-tracking_error / 0.5)  # 更稳定，更平滑


        # === 到达终点奖励 ===
        done_mask = (self._current_wp_idx >= self._trajectories.shape[1] - 1) & (tracking_error < 0.2)
        bias = torch.zeros_like(tracking_reward)
        bias[tracking_error < 0.2] = 1.0  # 每个点都能触发，不止终点
        bias[done_mask] = 5.0 # 终点 bonus

        # === 朝向奖励 ===
        forward_vector = self.quaternion_to_forward_vector(self.yaw)[:, :2]
        direction = current_target - self.positions
        traj_tangent = next_target - current_target
        traj_tangent = traj_tangent / (torch.norm(traj_tangent, dim=1, keepdim=True) + 1e-6)
        traj_alignment = torch.sum(forward_vector * traj_tangent, dim=1)

        traj_dir_reward = ((traj_alignment + 1) / 2.0)
        cos_theta = torch.sum(forward_vector * direction, dim=1) / (
            torch.norm(forward_vector, dim=1) * torch.norm(direction, dim=1) + 1e-6
        )
        direction_reward = ((cos_theta + 1.0) / 2.0) # 映射到 [0,2]

        # === 动作惩罚项 ===
        action_rate = torch.sum(torch.square(self._actions - self._previous_actions), dim=1)
        action_magnitude = torch.norm(self._actions, dim=1)
        action_rate_penalty = -action_rate
        action_mag_penalty = -action_magnitude

        # === 合并奖励 ===
        rewards = {
            "tracking_reward": tracking_reward * self.cfg.traj_track_scale * self.step_dt,
            "direction_reward": direction_reward * self.cfg.direction_scale * self.step_dt + traj_dir_reward * self.cfg.direction_scale * self.step_dt,
            "goal_bias": bias * self.cfg.traj_done_bonus,
            "action_rate_penalty": action_rate_penalty * self.cfg.action_rate_reward_scale * self.step_dt,
            "action_mag_penalty": action_mag_penalty * self.cfg.action_magnitude_scale * self.step_dt,
        }

        reward = torch.sum(torch.stack(list(rewards.values())), dim=0)

        # === 日志记录 ===
        for key, value in rewards.items():
            self._episode_sums[key] = self._episode_sums.get(key, torch.zeros_like(value)) + value
            self.writer.add_scalar(f"Reward/{key}", value.mean().item(), self.global_step)

        self.writer.add_scalar("Reward/Total", reward.mean().item(), self.global_step)

        return reward
    # 7. 判断回合结束
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # 7.1 判断是否达到最大回合步数
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        reached_target = torch.zeros_like(time_out, dtype=torch.bool)
        if self.dist_to_target is not None:
            # 7.2 判断是否到达目标位置
            reached_target = self.dist_to_target < 0.2
            self._current_wp_idx[reached_target] += 1
        finished_traj = self._current_wp_idx >= self._trajectories.shape[1]
        self.count += sum(finished_traj)
        return finished_traj, time_out

    # 8. 环境重置
    def draw_spline(self, traj_points, color=(0.0, 1.0, 0.0), thickness=1.0):
        """
        使用 debug_draw 绘制 spline 曲线
        traj_points: torch.Tensor (N, 2) or (N, 3)
        """
        import carb
        if traj_points.shape[1] == 2:
            z = torch.full((traj_points.shape[0], 1), 0.05, device=traj_points.device)
            traj_points = torch.cat([traj_points, z], dim=1)

        # 转换为 List[carb.Float3]
        points = [carb.Float3(*p.cpu().tolist()) for p in traj_points]

        self.debug_draw.draw_lines_spline(points, carb.ColorRgba(*color, 1.0), thickness, False)

    def generate_random_walk_trajectory(self,start_pos, num_points=15, step_size=0.5, seed=42, num_interp=10):
        torch.manual_seed(seed)
        traj = [start_pos]
        for _ in range(num_points - 1):
            angle = torch.rand(1) * 2 * math.pi
            direction = torch.tensor([torch.cos(angle), torch.sin(angle)], device=self.device)
            new_point = traj[-1] + direction * step_size
            traj.append(new_point)
        points = torch.stack(traj).cpu().numpy()
        t = np.linspace(0, 1, len(points))
        cs_x = CubicSpline(t, points[:, 0])
        cs_y = CubicSpline(t, points[:, 1])
        
        t_new = np.linspace(0, 1, len(points) * num_interp)
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
        smoothed = np.stack([x_new, y_new], axis=1)

        return torch.tensor(smoothed, dtype=torch.float32, device=self.device)



    def _reset_idx(self, env_ids: torch.Tensor | None):
        # 8.1 获取需要重置的环境 ID
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES

        # 8.2 重置机器人状态
        self._robot.reset(env_ids)
        # 8.3 重置基础环境状态
        super()._reset_idx(env_ids)

        # 8.4 重置动作缓冲
        time.sleep(1)
        default_root_state = self._robot.data.default_root_state[env_ids]
        self._prev_dist_to_target[env_ids] = 0.0
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        self._robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        if self.debug_draw:
            self.debug_draw.clear_lines()
        num_points = self.cfg.num_waypoints
        for i, env_id in enumerate(env_ids):
            start = self._robot.data.root_state_w[env_id, :2]

            # 随机生成终点
            traj = self.generate_random_walk_trajectory(start, num_points=num_points, num_interp=self.cfg.num_interp,
                                                         step_size=self.cfg.step_size, seed=self.seed)

            self._trajectories[env_id] = traj
            self._current_wp_idx[env_id] = 0

            # 可选可视化
            if self.debug_draw:
                self.draw_spline(traj)



