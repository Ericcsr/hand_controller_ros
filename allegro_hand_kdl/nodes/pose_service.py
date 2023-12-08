import rospy
from allegro_hand_kdl.srv import PoseGoalRequest, PoseGoal


if __name__ == "__main__":
    #rospy.init_node("test_pose_service")
    client = rospy.ServiceProxy("desired_cartesian_pose", PoseGoal)
    #client.wait_for_service("desired_cartesian_pose")

    input("Press to send request")
    req = PoseGoalRequest()
    req.pose = [0.08, 0.08, 0.0625,0.08, 0.0, 0.0625,0.08, -0.08, 0.0625,0.08, 0.0571, -0.03]
    res = client(req)
    print(res.success)

    input("Press to send request")
    req = PoseGoalRequest()
    req.pose = [0.06, 0.08, 0.1,0.06, 0.0, 0.1,0.06, -0.08, 0.1,0.08, 0.0571, -0.0]
    res = client(req)
    print(res.success)
