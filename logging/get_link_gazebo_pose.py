#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

"""
string[] name
geometry_msgs/Pose[] pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist[] twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z

"""


class GazeboLink(object):
	def __init__(self, link_name):
		self._link_dict = {}
		# self._robots_pose_list = []
		# self._robots_index_dict = {}
		self.link_name = link_name
		self.link_pose = Pose()

		# self.get_link()

		# We now start the suscriber once we have the indexes of each model
		rospy.Subscriber(self.link_name, Odometry, self.callback)

	def get_link_pose(self):
		data = None
		found_link = False
		while not found_link and not rospy.is_shutdown():
			log_str = "Retrieveing the link:" + self.link_name
			rospy.loginfo(log_str)
			try:
				data = rospy.wait_for_message(self.link_name, Odometry, timeout=5)
				# Save it in the format {"robot1":4,"robot2":2}
				if data:
					link_pose = data.pose.pose
					found_link = True
				else:
					log_str = "Topic:" + self.link_name + "NOT Ready yet, trying again"
					rospy.loginfo(log_str)
				return link_pose

			except Exception as e:
				s = str(e)
				rospy.loginfo("Error in get_robot_link = " + self.link_name)


	def callback(self, data):
		self.link_pose = data.pose.pose
		print "here"

        # for robot_name in self._gazebo_models_list:
            # # Retrieve the corresponding index
            # robot_name_found = self.update_robot_index(data, robot_name)
            # if robot_name_found:
                # data_index = self._link_dict[robot_name]
                # # Get the pose data from theat index
                # try:
                    # data_pose = data.pose[data_index]
                # except IndexError:
                    # rospy.logwarn("The model with data index " + str(data_index) + ", something went wrong.")
                    # data_pose = None
            # else:
                # data_pose = None
            # # Save the pose inside the dict {"robot1":pose1,"robot2":pose2}
            # self._robots_models_dict[robot_name] = data_pose

	def get_model_pose(self, link_name):

		pose_now = None

		try:
			pose_now = self._robots_models_dict[robot_name]
		except Exception as e:
			s = str(e)
			rospy.loginfo("Error, The _robots_models_dict is not ready = " + s)

		return pose_now


def listener():
	'''
	arg1: link name (str)
	'''
	rospy.init_node('link pose listener', anonymous=True)
	gz_link = GazeboLink(sys.argv[1])
	rate = rospy.Rate(10)  # 10hz
	while not rospy.is_shutdown():
		# pose_now = gz_link.get_link_pose()
		# print ("POSE NOW ROBOT =" + link_name + "==>" + str(pose_now))
		print "Pose:", gz_link.link_pose
		rate.sleep()


if __name__ == '__main__':
    listener()
