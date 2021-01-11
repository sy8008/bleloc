import requests
import json
import urllib2
import urllib
import time
import cv2
url = "http://192.168.31.100:2345/bleloc"
#loc = urllib2.urlopen(url)
point_color = (0, 0, 255) # BGR

map=cv2.imread("/home/zx/BlelocMap.png")
while True:
	draw_map=map.copy()
	loc = requests.get(url).json()
	print(type(loc))
	PosX,PosY=loc[u'posX'],loc[u'posY']
	# cv2.circle(draw_map, (469-int(PosY), int(PosX)), 1, point_color, 4)
	cv2.circle(draw_map, (int(PosX),469-int(PosY)),1, point_color, 4)
	cv2.imshow("map",draw_map)
	cv2.waitKey(1)
	print(loc)
	time.sleep(1.5)

cv2.destroyAllWindows()