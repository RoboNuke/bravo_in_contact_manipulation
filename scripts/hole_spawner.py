#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from bravo_in_contact_manipulation.env_manager import EnvManager

def main():
    #rospy.init()
    # read params for peg
    pegShape = rospy.get_param("peg_shape", "circle")
    pegSize = rospy.get_param("peg_size", "30")
    modelPath = rospy.get_param("model_path", "/home/hunter/catkin_ws/src/bravo_in_contact_manipulation/")

    envMan = EnvManager()
    # load hole base
    baseSDF = modelPath + "/sdfs/hole_holder.sdf"
    basePose = Pose()
    basePose.position = Point(0.5, 0.0, 0.00)
    basePose.orientation.w = 1.0
    envMan.spawn_object("hole_holder", basePose,baseSDF, False)

    
    # load hole plate
    plateSDF = modelPath + f"/sdfs/plates/{pegShape}_{pegSize}mm_plate.sdf"
    envMan.spawn_object(f"{pegShape}_{pegSize}mm_plate", basePose, plateSDF, False)
    

import sys
if __name__=="__main__":
    main()
    sys.exit()