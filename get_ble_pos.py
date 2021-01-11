import requests
import json
import urllib2
import urllib
import rospy


url = "http://192.168.10.2:2345/bleloc"
#loc = urllib2.urlopen(url)

loc = requests.get(url).json()
print(loc)
print("\n")
print(type(loc))