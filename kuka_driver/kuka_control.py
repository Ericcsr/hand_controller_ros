import numpy as np
import time
import rospy
import roslib
from allegro_hand_kdl.srv import PoseGoalRequest, PoseGoal
from std_msgs.msg import String
import tf
from iiwaPy3 import iiwaPy3
# import pybullet as pb

R = np.array([[0.7071,-0.7071,0.0],
              [0.7071,0.7071,0.0],
              [0.0,0.0,1.0]]) # +45 degree
t = np.array([624.1869927062419, -84.9738197397598, 180]) * 0.001 # tune by hand.
euler_offset = np.array([-1.65164718e+00, -2.70486858e-05, -np.pi])

def get_ee_world_pose(pos):
    return R.T@(pos[:3] * 0.001 - t), pos[3:] - euler_offset

def get_ee_base_pose(pos,ori):
    return ((R@pos[:3] + t) * 1000).tolist() + (ori + euler_offset).tolist()


if __name__ == '__main__':
    rospy.init_node("kuka_position_control")
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher("kuka_status", String, queue_size=1)
    ip  = "172.31.1.147"
    iiwa = iiwaPy3(ip)

    def new_pose_handler(pose_msg):
        print("New message received.")
        pose = np.array(pose_msg.pose)
        ee_base = get_ee_base_pose(pose[:3], pose[3:])
        print(ee_base, iiwa.getEEFPos())
        iiwa.movePTPLineEEF(ee_base, [100])
        ee_pose_new = np.array(iiwa.getEEFPos())
        ee_pos_new, ee_ori_new = get_ee_world_pose(ee_pose_new)
        if np.linalg.norm(ee_pos_new - pose[:3]) > 0.005 or np.linalg.norm((ee_ori_new - pose[3:] + np.pi)%(np.pi*2) - np.pi) > 0.001:
            print("Failed to reach target.")
            return False
        return True

    def cmd_handler(cmd_msg):
        if cmd_msg.data == "close":
            iiwa.close()
            rospy.signal_shutdown("Close command received.")

    ee_base = get_ee_base_pose([-0.2,0.0,0.61], [0.0,0.0,0.0])
    iiwa.movePTPLineEEF(ee_base, [100])


    
    rospy.Subscriber("/kuka_cmd", String, cmd_handler)
    rospy.Service("kuka_pose_service", PoseGoal, new_pose_handler)
    rospy.spin()

    # try:
    #     ee_pos = np.array(iiwa.getEEFPos())
    #     print("Start moving:", ee_pos)
    #     ee_pos, ee_ori = get_ee_world_pose(ee_pos)
        
    #     # br.sendTransform((ee_pos[0],ee_pos[1],ee_pos[2]),
    #     #                   tf.transformations.quaternion_from_euler(ee_ori[0],ee_ori[1],ee_ori[2]),
    #     #                   rospy.Time.now(),
    #     #                   "iiwa_link_ee",
    #     #                   "world")
    #     # unit measured in mm, internal IK doesn't seems to be reliable...
    #     pos = [-0.0, -0.0, 0.21]
    #     ee_base = get_ee_base_pose(pos, [0.0,0.0,0.0])
    #     print("Moving to:", ee_base)
    #     iiwa.movePTPLineEEF(ee_base, [100])
    #     ee_pos_new = iiwa.getEEFPos()
    #     print("Done:", ee_pos_new)
    # except Exception as e:
    #     print("Error happened:",e)
    # iiwa.close()