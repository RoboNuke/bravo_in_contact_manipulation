import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, GetModelState
from bravo_in_contact_manipulation.utils import Conversion, noisy_pose
class Model():
    def __init__(self, name: str = None, pose: Pose = None, sdf_path: str = None) -> None:  
        self.name, self.init_pose, self.sdf_name = name, pose, sdf_path
        self.base2obj = Conversion().pose2T(pose)
