import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Pose

class Conversion():
    
    @staticmethod
    def list2pose(sixd: list, degrees: bool = True) -> Pose:
        """
        Convert 6D representation to ROS Pose message format.

        Parameters
        ----------
        sixd : 1x6 : obj : `list`
            list of xyz and three euler angles
        degrees : bool
            whether euler angles are given in degree or radian

        Returns
        -------
        `Pose` : pose message composed with xyz and quaternion
        """
        position = Point(sixd[0], sixd[1], sixd[2])
        temp = R.from_euler('xyz', sixd[3:], degrees).as_quat()
        quat = Quaternion(temp[0], temp[1], temp[2], temp[3])

        return Pose(position=position, orientation=quat)

    @staticmethod
    def pose2T(pose: Pose) -> np.ndarray:
        """
        Convert ROS Pose message format to homegeneous transformation. 

        Parameters
        ----------
        pose : obj : `Pose`
            pose message composed with xyz and quaternion

        Returns
        -------
        res : 4x4 : obj : `np.ndarray`
            homegeneous transformation format
        """
        res = np.eye(4)
        res[:3,3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, 
                pose.orientation.w]
        res[:3,:3] = R.from_quat(quat).as_matrix()

        return res

def noisy_pose(mu: list = [0.0, 0.0], sigma: list = [0.5, 2.0], bouds: list = [1.5, 5.0]) -> list:
    """
    Add gaussian noise on pose values.

    Parameters
    ----------
    mu : 1x2 : obj : `list`
        means of gaussian noise models
    sigma : 1x2 : obj : `list`
        standard deviations of gaussian noise models

    Returns
    -------
    noise_position : 3x1 : obj : `np.ndarray`
        array of position noise in xyz
    noise_orientation : 4x1 : obj : `np.ndarray`
        array of orientation noise in quaternion
    """
    np.random.seed()
    noise_position = np.random.normal(mu[0],sigma[0],3)
    noise_orientation = np.random.normal(mu[1],sigma[1],3)
    noise_position = np.clip(noise_position, -bouds[0], bouds[0]) * 0.01
    noise_orientation = np.clip(noise_orientation, -bouds[1], bouds[1]) * np.pi / 180
    noise_orientation = R.from_euler('xyz', noise_orientation).as_quat()

    return noise_position, noise_orientation 