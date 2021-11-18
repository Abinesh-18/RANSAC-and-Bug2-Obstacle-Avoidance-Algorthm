#!/usr/bin/env python
import roslib
roslib.load_manifest('lab2')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import random
import math


def distlp(m,c,X,Y): #Distance between line and points
	D=abs((m*X)+(-1*Y)+c)
	D=D/math.sqrt(m*m+1)
	return D 

def linepoints(x,y): # Find the start and end points of the finalised line
	n=len(x)
	max=0
	max1=0
	max2=0
	for i in range(0,n-1):
		for j in range(0,n-1):	
			dist = math.hypot(x[i]-x[j], y[i]-y[j])
			if dist>max:
				max=dist
				max1=i
				max2=j
	return max1,max2
					
def poly2cart(r, theta):
    x = r * math.cos(theta)	
    y = r * math.sin(theta)
    return(x, y)			

def callback(data):
	
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	
	pub2 = rospy.Publisher('/ransac_vis',Marker, queue_size = 100)
	#uint32_t shape = visualization_msg.Marker.CUBE

	marker = Marker()
	marker.header.frame_id = "/base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns="lines"
	
	marker.type=marker.LINE_LIST
	marker.action = marker.ADD
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.1
	#marker.scale.y = 0.1
	#marker.scale.z = 1.0
	marker.color.b = 1.0
        marker.color.a = 1.0	
	
		

	rangelist=[]
	for i in range(120,240):
		rangelist.append(data.ranges[i])
				
	#if min(rangelist)<=1:
	#	pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,random.uniform(0,3.14))))
	#else:
	#	pub.publish(Twist(Vector3(2,0,0),Vector3(0,0,0)))
	
	r=[]
	theta=[]
	x=[]
	y=[]	
	x3=[]
	y3=[]
	angle=-1.57079
	for i in range(0,359):
		r.append(data.ranges[i])
				
		theta.append(angle)
		angle=angle+0.0087266
		
		x2,y2=poly2cart(r[i],theta[i])
		x.append(x2)
		y.append(y2)
		
		if r[i]<2:
			x3.append(x2)   #coordinate x of all filtered points
			y3.append(y2)	#coordinate x of all filtered points
			
			
		
	n=len(x3)
	if n!=0:
		max=0
		for j in range(0,15):  #for number of iterations		
			rand1=random.randint(0,n-1)
			rand2=random.randint(0,n-1)
			if rand1!=rand2:
				points=[(x3[rand1],y3[rand1]),(x3[rand2],y3[rand2])]
				x_co, y_co = zip(*points)
				m = (y_co[0] - y_co[1])/(x_co[0] - x_co[1])
                                c = y_co[0] - m*x_co[0] # Y = mX + c
				inliers=0
                                outliers=0
                                x6=[]
                                y6=[]
				x4=[]
				y4=[]
				for i in range(0,n-1):
					dist=distlp(m,c,x3[i],y3[i])				
					if dist<0.5:
						inliers=inliers+1
						x4.append(x3[i])
						y4.append(y3[i])
                                        else: 
                                                outliers = outliers + 1;
                                                x6.append(x3[i])
			                        y6.append(y3[i])
				if inliers>max:
					max=inliers
					x5=x4[:]
					y5=y4[:]
                  
                      
					
		if rand1!=rand2:
			start,end = linepoints(x5,y5)
			marker.id=1

			sixPoints=[]	
			p1=Point()
			p1.x=x5[start]
			p1.y=y5[start]
			p1.z=0
			sixPoints.append(p1)	
			p2=Point()
			p2.x=x5[end]
			p2.y=y5[end]
			p2.z=0
			sixPoints.append(p2)
			marker.points=sixPoints
			pub2.publish(marker)
				
					




	
def Evader():
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rospy.init_node('evader',anonymous=True)
	rate = rospy.Rate(120) 
	sub = rospy.Subscriber('/base_scan',LaserScan,callback)
		
	


	
		
if __name__ =="__main__":
	try:
		Evader()
	except rospy.ROSInterruptException:
        	pass
	
	
        rospy.spin()
	
