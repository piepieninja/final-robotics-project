#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

pub = None
sub = None

def camera_callback(camera_data, pose_data):
	print('in callback')
	rospy.loginfo(rospy.get_caller_id() + "I got %s", str(camera_data))
	rospy.loginfo(rospy.get_caller_id() + "I got %s", str(pose_data))

	rospy.loginfo(rospy.get_caller_id() + "Inverse got %s", str(wheel_vel))
	pub.publish(wheel_vel)

def Listen_Camera():
    global sub, pub
    rospy.init_node('Listen_Camera', anonymous=True)
    pub = rospy.Publisher('/image_to_camera_pose', Pose, queue_size = 10)
    print('in the listen')

    camera_sub  = message_filters.Subscriber("/sense_dude_square_loc", Pose)
    position_sub = message_filters.Subscriber("/poseupdate", PoseWithCovarianceStamped)
    #changed timeSynch to approximateTimeSynch so as to allow bayes guy to capture messages in 0.3 sec interval and ones with no headers(command topic). 
    ts = message_filters.ApproximateTimeSynchronizer([camera_sub, position_sub], 10, 0.3, allow_headerless=True)
    ts.registerCallback(camera_callback)

    rospy.spin()

if __name__ == '__main__':
  try :
    Listen_Camera()
  except rospy.ROSInterruptException :
    pass
