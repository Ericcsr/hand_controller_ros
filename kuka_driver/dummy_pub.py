import rospy
import actionlib
from allegro_hand_kdl.srv import PoseGoalRequest, PoseGoal

import time

if __name__ == "__main__":
    time.sleep(1.0)
    client = rospy.ServiceProxy("kuka_pose_service", PoseGoal)
    client.wait_for_service()

    req = PoseGoalRequest()
    req.pose = [0.07, 0.0, 0.21, 0.0, 0.0, 0.0]
    res = client(req)
    print("Finished:", res.success)

    req.pose[0] = -0.07
    res = client(req)
    print("Finished:", res.success)

