import numpy as np
import time
import rospy
import roslib
from allegro_hand_kdl.srv import PoseGoalRequest, PoseGoal
from std_msgs.msg import String
import tf
from iiwaPy3 import iiwaPy3
from scipy.spatial.transform import Rotation
# import pybullet as pb

R = np.array([[0.7071,-0.7071,0.0],
              [0.7071,0.7071,0.0],
              [0.0,0.0,1.0]]) # +45 degree
t = np.array([5.39358750e+02, -1.69763481e+02, 180]) * 0.001 # tune by hand, where the table world frame is w.r.t. the robot.
euler_offset = np.array([0.0, 0.0, 0.0]) # [yaw, pitch, roll ] euler angle w.r.t.the base frame

def get_ee_world_pose(pos):
    pos_R = Rotation.from_euler('zyx', pos[3:]+np.array([0.0,0.0,-np.pi]))
    euler_ee = Rotation.from_matrix(R.T@pos_R.as_matrix()).as_euler('xyz')
    return R.T@(pos[:3] * 0.001 - t), euler_ee - euler_offset

def get_ee_base_pose(pos,ori):
    euler = Rotation.from_matrix(Rotation.from_euler("xyz",ori+euler_offset).as_matrix()@R).as_euler("zyx") + np.array([0.0,0.0,np.pi])
    euler
    return ((R@pos[:3] + t) * 1000).tolist() + euler.tolist()


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
        print("New base pose:", ee_pose_new)
        ee_pos_new, ee_ori_new = get_ee_world_pose(ee_pose_new)
        if np.linalg.norm(ee_pos_new - pose[:3]) > 0.005 or np.linalg.norm((ee_ori_new - pose[3:] + np.pi)%(np.pi*2) - np.pi) > 0.001:
            print("Failed to reach target:", ee_base)
            return False
        return True
    
    def new_joint_handler(joint_msg):
        print("New message received.")
        joint = np.array(joint_msg.pose)
        iiwa.movePTPJointSpace(joint, [0.15])
        joint_new = np.array(iiwa.getJointsPos())
        if np.linalg.norm(joint_new - joint) > 0.005:
            print("Failed to reach target:", joint)
            return False
        return True

    def cmd_handler(cmd_msg):
        if cmd_msg.data == "close":
            iiwa.close()
            rospy.signal_shutdown("Close command received.")

    # ee_base = get_ee_base_pose([0.0,0.0,0.3], [0.0,0.0,np.pi])
    # print("Initial base pose:", ee_base)
    iiwa.movePTPJointSpace([-0.7153848657390822, 0.23627692865494376, -0.06146527133579401, -1.2628601611175012, 0.01487889923612773, 1.6417360407890011, -2.344269879142319], [0.15])
    ee_pose_new = np.array(iiwa.getEEFPos())
    print("New base pose:", ee_pose_new)


    
    rospy.Subscriber("/kuka_cmd", String, cmd_handler)
    rospy.Service("kuka_pose_service", PoseGoal, new_pose_handler)
    rospy.Service("kuka_joint_service", PoseGoal, new_joint_handler)
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
