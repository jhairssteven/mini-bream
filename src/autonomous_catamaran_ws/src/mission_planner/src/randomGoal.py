#!/usr/bin/env python
""" Every 2s publish a PoseStamped with random x,y in the range -10,10. 
    This is primarily used for testing the real-time online waypoint goal setting performance
    of the actionServer of the PathPlanner. 
    """
import rospy
import random
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def generate_random_goal():
    goal = PoseStamped()
    goal.header = Header()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = random.uniform(-10, 10)
    goal.pose.position.y = random.uniform(-10, 10)
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    return goal

def publisher():
    rospy.init_node('random_goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(0.5)  # Publish every 2 seconds

    while not rospy.is_shutdown():
        goal = generate_random_goal()
        rospy.loginfo(f"Publishing new goal: x={goal.pose.position.x}, y={goal.pose.position.y}")
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
