#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


def create_goal(x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quat = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*quat)
    return goal


def send_goal(client, goal, label=None, wait_time=0):
    if label:
        rospy.loginfo(label)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    if result:
        if wait_time > 0:
            rospy.loginfo("清掃中")
            rospy.sleep(wait_time)
        return True
    else:
        rospy.logwarn("目標失敗！")
        return False


def main():
    rospy.init_node('multi_goal_navigation')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base action server...")
    client.wait_for_server()

    rospy.loginfo("開始清掃...")

    # 第一階段：固定清掃直線（列印清掃中）
    path = [
        (1.1, 0.0),
        (1.1, 2.0),
        # (0.8, 2.0),
        # (0.8, 1.0),
        # (0.5, 1.0),
        (0.5, 2.0)
    ]

    for x, y in path:
        rospy.loginfo("清掃中")
        goal = create_goal(x, y)
        success = send_goal(client, goal)
        if not success:
            rospy.logwarn("清掃中斷於 (%.2f, %.2f)" % (x, y))
            return

    # 第二階段：三個任務點位
    targets = [
        (2.8, -0.1),
        (3.5, 3.5),
        (2.0, 2.2),
        (0.3, 0.0)
    ]

    for x, y in targets:
        rospy.loginfo("前往 (%.1f, %.1f)" % (x, y))
        goal = create_goal(x, y)
        success = send_goal(client, goal, wait_time=2)
        if not success:
            rospy.logwarn("無法抵達任務點 (%.2f, %.2f)" % (x, y))

    # 回家
    rospy.loginfo("回家充電中，清掃結束")
    home = create_goal(0.2, 0.0, yaw=0.0)  # 微調位置，朝X軸
    send_goal(client, home)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
