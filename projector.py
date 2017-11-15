#!/usr/bin/env python
import xml.etree.ElementTree as ET
import rospy
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf.transformations as transformations
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class MapCreator():

	def __init__(self):
		rospy.init_node('get_walls')
		self.publisher = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=1)
		# self.world_file = '/home/abhishek/catkin_ws/src/pypcl/square.xml'
		self.world_file = '/home/abhishek/catkin_ws/src/pypcl/corridor2.xml'
		self.walls=[]
		self.wallcount=0;
		self.poses=[]
		self.posecount=0;
		self.getXMLWalls()
		# rospy.loginfo(self.poses)
		self.POINTS=[]
		self.HEADER = Header(frame_id='/temp')
		self.l=1;
		self.b=1;
		self.h=1;
		self.FIELDS = [

			PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
			PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
			PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
			PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
			# PointField(name='my_field1', offset=16, datatype=PointField.FLOAT32, count=1),
			# PointField(name='my_field2', offset=20, datatype=PointField.FLOAT32, count=1),]
			]
		self.create_points()




	def getXMLWalls(self):
		# rospy.loginfo("hello")
		wall_objects={}
		tree = ET.parse(self.world_file)
		# rospy.loginfo(tree)
		root = tree.getroot()
		for model in root.findall('./world/model'):
			model_name = model.attrib['name']
			# rospy.loginfo(model_name)
			if 'grey' in model_name:
				# rospy.loginfo("wall found")
				box_size_str = model.find('.link/collision/geometry/box/size').text
				box_size = [float(x) for x in box_size_str.split()]
				box_size_vec = Vector3(*box_size)
				wall_object = {}
				wall_object["size"] = box_size_vec
				wall_objects[model_name] = wall_object
				# wall_objects[model_name] = wall_object
				self.walls.append([self.wallcount,model_name,box_size_vec])
				self.wallcount=self.wallcount+1;


		for model in root.findall('./world/state/model'):
			model_name = model.attrib['name']
			if model_name in wall_objects:
				wall_object = wall_objects[model_name]
				box_pose_str = model.find('./link/pose').text
				box_pose = [float(x) for x in box_pose_str.split()]
				pose = Pose()
				pose.position = Vector3(*box_pose[0:3])
				orientation = transformations.quaternion_from_euler(*box_pose[3:6])
				pose.orientation = Quaternion(*orientation)
				# wall_object["pose"] = pose
				self.poses.append([self.posecount,model_name,pose])
				self.posecount=self.posecount+1
				# rospy.loginfo("pose found")

		# return wall_objects.values()

	def create_points(self):
		for n in range(0,(self.wallcount)):
			
			l=self.walls[n][2].x
			b=self.walls[n][2].y
			h=self.walls[n][2].z
			# rospy.loginfo(self.poses[2])
			px=self.poses[n][2].position.x
			py=self.poses[n][2].position.y
			pz=self.poses[n][2].position.z
			quat=(self.poses[n][2].orientation.x,self.poses[n][2].orientation.y,self.poses[n][2].orientation.z,self.poses[n][2].orientation.w)

			a=transformations.euler_from_quaternion(quat)

			lx=abs(l*np.cos(a[2])-b*np.sin(a[2]));
			ly=abs(l*np.sin(a[2])+b*np.cos(a[2]));

			index=20.0
			
						
			for p in xrange(-int(lx*index/2.0), int(lx*index/2.0)):
				for q in xrange(-int(ly*index/2.0), int(ly*index/2.0)):
						for r in xrange(-int(h*index/2.0), int(h*index/2.0)):
							# rospy.loginfo(p)
							x=px+p/index;
							y=py+q/index;
							z=pz+r/index;
							self.POINTS.append([x,y,z,0xff0000])
		
		self.publish_points()

		
		# rospy.loginfo(self.POINTS)

	def publish_points(self):
		point_cloud = pc2.create_cloud(self.HEADER, self.FIELDS, self.POINTS)
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			# rospy.loginfo(self.POINTS)
			self.publisher.publish(point_cloud)
			r.sleep()



def main():
	try:
		mapmaker=MapCreator()        
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()



