from dm_control import composer
from dm_control.composer import Observables
from dm_control.composer.observation import observable


class RobotObservables(Observables):
    @composer.observable
    def joint_positions(self):
        return observable.MJCFFeature("qpos", self._entity.mjcf_joints)

    @composer.observable
    def joint_velocities(self):
        return observable.MJCFFeature("qvel", self._entity.mjcf_joints)

    @composer.observable
    def joint_efforts(self):
        return observable.Generic(lambda physics: physics.data.actuator_force)

    @composer.observable
    def angular_velocity(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.gyro,
        )

    @composer.observable
    def linear_acceleration(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.accelerometer
        )

    @composer.observable
    def orientation(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.framequat
        )
