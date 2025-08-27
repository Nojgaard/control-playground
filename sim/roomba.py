import os

import numpy as np
from dm_control import composer, mjcf
from dm_control.composer import Entity
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics

from sim.observables import RobotObservables


class Robot(Entity):
    def __init__(self):
        self._model_path = os.path.join("mjcf", "roomba", "robot.xml")
        super().__init__()

    def _build(self):
        self._model = mjcf.from_path(self._model_path)
        self.mjcf_joints = [a.joint for a in self.mjcf_model.find_all("actuator")]
        print("JOINT", self.mjcf_joints)

    @property
    def mjcf_model(self):
        return self._model

    def _build_observables(self):
        return RobotObservables(self)

    @property
    def observables(self) -> "RobotObservables":
        return super().observables


class Roomba(composer.Task):
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
    def wheel_radius(self):
        """Radius of the robot's wheels (in meters)."""
        return 0.035

    @property
    def wheel_separation(self):
        """Distance between the wheels (in meters)."""
        return 0.24

    @property
    def root_entity(self):
        return self._arena

    def get_reward(self, physics: Physics):
        return 0
