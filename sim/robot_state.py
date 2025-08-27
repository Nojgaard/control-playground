import numpy.typing as npt
from dm_env import TimeStep
from scipy.spatial.transform import Rotation


class RobotState:
    def __init__(self, time_step: TimeStep):
        """
        Initialize the State with a dm_env.TimeStep object.
        """
        self._time_step = time_step

    @property
    def joint_positions(self) -> npt.ArrayLike:
        """
        Joint positions in radians.
        Returns:
            Array-like: Joint positions [radians].
        """
        return self._time_step.observation["robot/joint_positions"]

    @property
    def joint_velocities(self) -> npt.ArrayLike:
        """
        Joint velocities in meters per second (m/s).
        Returns:
            Array-like: Joint velocities [m/s].
        """
        return self._time_step.observation["robot/joint_velocities"]

    @property
    def joint_efforts(self) -> npt.ArrayLike:
        """
        Joint efforts (forces) in Newtons.
        Returns:
            Array-like: Joint efforts [N].
        """
        return self._time_step.observation["robot/joint_efforts"]

    @property
    def orientation(self) -> npt.ArrayLike:
        """
        Returns the orientation of the robot as Euler angles.

        The orientation is retrieved as a quaternion [w, x, y, z] from the observation,
        then converted to Euler angles (XYZ order, in radians).

            np.ndarray: Orientation as Euler angles [x, y, z] in radians.
        """
        obs = self._time_step.observation["robot/orientation"]
        return Rotation.from_quat(obs, scalar_first=True).as_euler("XYZ", degrees=False)

    @property
    def angular_velocity(self) -> npt.ArrayLike:
        """
        Angular velocity in meters per second (m/s) for each axis.
        Returns:
            Array-like: Angular velocity [m/s].
        """
        return self._time_step.observation["robot/angular_velocity"]

    @property
    def linear_acceleration(self) -> npt.ArrayLike:
        """
        Linear acceleration in meters per second squared (m/s^2).
        Returns:
            Array-like: Linear acceleration [m/s^2].
        """
        return self._time_step.observation["robot/linear_acceleration"]
