import requests
import json
import urllib2
import urllib
import time
import cv2
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

import numpy as np




	
global linear_vel,angle_vel
ble_position=np.zeros((2,))

flag=0



def getVelocity_cb(data):
	global linear_vel,angle_vel
	linear_vel=np.array([[data.linear.x,data.linear.y,data.linear.z]],dtype=np.float32)
	angle_vel=np.array([[data.angular.x,data.angular.y,data.angular.z]],dtype=np.float32)
	

def getPose_cb(data):
	global ble_position
	global flag
	flag=1
	ble_position=np.array([data.position.x,data.position.y])


url = "http://192.168.31.100:2345/bleloc"
#loc = urllib2.urlopen(url)
point_color = (0, 0, 255) # BGR
linear_vel=np.zeros((1,3),dtype=np.float32)
angle_vel=np.zeros((1,3),dtype=np.float32)
bleloc_cmd=Twist()
bleloc_p=Pose()


map=cv2.imread("/home/songyang/BlelocMap.png")

rospy.init_node('bleloc', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, getVelocity_cb)
rospy.Subscriber("/pose", Pose, getPose_cb)

bleloc_p_pub = rospy.Publisher('/pose', Pose, queue_size=1)


while not rospy.is_shutdown():

	draw_map=map.copy()
	print(flag)
	if(flag==1):
		print("pos:{}".format(ble_position))		
		cv2.circle(draw_map, (int(ble_position[0]),469-int(ble_position[1])),1, point_color, 4)
		cv2.imshow("map",draw_map)
		cv2.waitKey(1)
		# print(loc)
	time.sleep(1)

cv2.destroyAllWindows()






