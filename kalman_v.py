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
import Queue



def getVelocity_cb(data):
	global linear_vel,angle_vel
	linear_vel=np.array([[data.linear.x,data.linear.y,data.linear.z]],dtype=np.float32)
	angle_vel=np.array([[data.angular.x,data.angular.y,data.angular.z]],dtype=np.float32)
	# print("linear_vel:{}".format(linear_vel))

def judgeQueueDiff(x_queue,thresh):
	x_list=x_queue.queue
	for i in range(1,len(x_list)):
		if(np.abs(x_list[i]-x_list[i-1])>thresh):
			return False
	return True


global linear_vel,angle_vel
url = "http://192.168.31.100:2345/bleloc"
xmin=174
xmax=376
ymin=95
ymax=123
topomap_x=np.array([161,168,194,217,244,268,295,320,345,363],np.int64)
topomap_x_small=np.array([175,238,308,340],np.int64)
dist_thresh=13.35*1.5

x_aver=0
y_aver=0
v=13.35



#loc = urllib2.urlopen(url)
point_color = (0, 0, 255) # BGR
linear_vel=np.zeros((1,3),dtype=np.float32)
angle_vel=np.zeros((1,3),dtype=np.float32)
bleloc_cmd=Twist()
bleloc_p=Pose()
kalman=cv2.KalmanFilter(4,4)
kalman.measurementMatrix=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32)
kalman.transitionMatrix=np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
kalman.processNoiseCov=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32)*3
kalman.measurementNoiseCov=np.array([[1,0,0,0],[0,1,0,0],[0,0,0.01,0],[0,0,0,0.01]],np.float32)

posX_queue=Queue.Queue(3)
posY_queue=Queue.Queue(3)
last_pos_x=0
curr_pos_x=0
posx_clip_q=[]
posy_clip_q=[]

map=cv2.imread("/home/songyang/BlelocMap.png")

rospy.init_node('bleloc', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, getVelocity_cb)
bleloc_p_pub = rospy.Publisher('/pose', Pose, queue_size=1)
iter=0
curr_pos_x_thresh_filter=0

curr_pos_x_thresh_filter_disre=0

while not rospy.is_shutdown():
	iter+=1
	print(iter)
	draw_map=map.copy()
	loc = requests.get(url).json()
	# print(type(loc))
	PosX,PosY=loc[u'posX'],loc[u'posY']


	kalman.correct(np.array([[PosX],[PosY],[-13.35],[0.0]],np.float32))
	prediction=kalman.predict()

	bleloc_p.position.x = PosX
	bleloc_p.position.y = PosY
	bleloc_p.position.z = 0.0
	# Make sure the quaternion is valid and normalized
	bleloc_p.orientation.x = 0.0
	bleloc_p.orientation.y = 0.0
	bleloc_p.orientation.z = 0.0
	bleloc_p.orientation.w = 0.0
	bleloc_p_pub.publish(bleloc_p)

	posx_clip=np.clip(PosX,xmin,xmax)
	# if(last_pos_x!=0):
	# 	if(np.abs(last_pos_x-posx_clip)<dist_thresh):
	# 		curr_pos_x_thresh_filter=posx_clip
	# 		curr_pos_x_thresh_filter_disre=topomap_x_small[np.argmin(np.abs(posx_clip-topomap_x_small))]


	last_pos_x=posx_clip
	posy_clip=np.clip(PosY,ymin,ymax)

	posx_clip_q.append(posx_clip)
	posy_clip_q.append(posy_clip)
	if(posX_queue.full()):
		posX_queue.get()
		posY_queue.get()
		x_aver=np.mean(posX_queue.queue)
		y_aver=np.mean(posY_queue.queue)

	posX_queue.put(posx_clip)
	posY_queue.put(posy_clip)

	if (posX_queue.full()):
		last_pos_x=posX_queue.queue[1]
		curr_pos_x=posX_queue.queue[2]
		if(judgeQueueDiff(posX_queue,dist_thresh)):
			curr_pos_x_thresh_filter_disre = topomap_x_small[np.argmin(np.abs(curr_pos_x - topomap_x_small))]

	if(v>0):
		if(curr_pos_x>last_pos_x):
			curr_pos_x=last_pos_x-v




	posx_rec=topomap_x_small[np.argmin(np.abs(posx_clip-topomap_x_small))]
	posy_rec=110
	cv2.circle(draw_map, (int(PosX),469-int(PosY)),1, (255, 0, 0), 4)
	# cv2.circle(draw_map, (int(posx_rec),469-int(posy_rec)),1, point_color, 4)
	# cv2.circle(draw_map, (int(posx_rec), 469 - 110), 1, (0, 0, 255), 4)
	# cv2.circle(draw_map, (int(posx_clip),469-int(posy_clip)),1, (255, 0, 0), 4)
	if(curr_pos_x_thresh_filter_disre!=0):
		cv2.circle(draw_map, (int(curr_pos_x_thresh_filter_disre), 469 - 110), 1, (0, 0, 255), 4)
	cv2.imshow("map",draw_map)
	cv2.waitKey(1)
	# print(loc)
	time.sleep(1)

cv2.destroyAllWindows()






