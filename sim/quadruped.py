from dm_control import mjcf
import numpy as np
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics
from dm_control.composer import Entity, Task

from sim.observables import RobotObservables


class Robot(Entity):
    def __init__(self):
        self._model_path = "mjcf/quadruped/robot.xml"
        super().__init__()

    def _build(self):
        self._model = mjcf.from_path(self._model_path)
        self._imu = self._model.find("site", "imu")
        self.joint_ranges = np.array([j.range for j in self._model.find_all("joint")])

        self.mjcf_joints = list(self._model.find_all("joint"))
        for joint in self._model.find_all("joint"):
            if joint.type == "free":
                continue
            # joint.actuatorfrcrange = "-100 100"
            self._model.actuator.add(
                "position",
                name=joint.name,
                joint=joint,
                ctrlrange=joint.range,
                kp=15,
                kv=1,
            )

    def _build_observables(self):
        return RobotObservables(self)

    @property
    def observables(self) -> "RobotObservables":
        return self._observables

    @property
    def mjcf_model(self):
        return self._model

    @property
    def body_width(self) -> float:
        left_hip = self.mjcf_model.find("joint", "motor_front_left_shoulder")
        right_hip = self.mjcf_model.find("joint", "motor_front_right_shoulder")
        return np.linalg.norm(left_hip.pos - right_hip.pos)

    @property
    def body_length(self) -> float:
        front_hip = self.mjcf_model.find("joint", "motor_front_left_shoulder")
        rear_hip = self.mjcf_model.find("joint", "motor_rear_left_shoulder")
        return np.linalg.norm(front_hip.pos - rear_hip.pos)

    @property
    def max_height(self) -> float:
        return self.arm_length + self.wrist_length

    @property
    def arm_length(self) -> float:
        arm_joint = self._model.find("joint", "motor_front_left_arm")
        wrist_joint = self._model.find("joint", "motor_front_left_wrist")
        return np.linalg.norm(arm_joint.pos - wrist_joint.pos)

    @property
    def wrist_length(self) -> float:
        return 0.12

    @property
    def hip_offset(self) -> tuple[float, float]:
        shoulder_pos = self._model.find("joint", "motor_front_left_shoulder").pos
        arm_pos = self._model.find("joint", "motor_front_left_arm").pos
        shoulder_offsets = np.abs(shoulder_pos - arm_pos)

        return np.array([shoulder_offsets[2], shoulder_offsets[1]])


class Quadruped(Task):
    def __init__(self, control_timestep: float = 0.01):
        super().__init__()

        self._robot = Robot()
        self._arena = Floor(reflectance=0.0)
        self._arena.add_free_entity(self._robot)
        self.set_timesteps(control_timestep, physics_timestep=0.005)

        self._robot.observables.joint_positions.enabled = True
        self._robot.observables.joint_velocities.enabled = True
        self._robot.observables.joint_efforts.enabled = True
        self._robot.observables.orientation.enabled = True
        self._robot.observables.angular_velocity.enabled = True
        self._robot.observables.linear_acceleration.enabled = True

    def initialize_episode(self, physics: Physics, random_state):
        self._robot.set_pose(physics, np.array([0, 0, 0]), np.array([1, 0, 0, 0]))

    @property
    def root_entity(self):
        return self._arena

    def get_reward(self, physics: Physics):
        return 0
