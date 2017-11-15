#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2






# POINTS = []



class CustomPointCloud(object):
    def __init__(self):
        rospy.init_node('publish_custom_point_cloud')
        self.publisher = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=1)
        self.POINTS=[]
        self.HEADER = Header(frame_id='/odom')
        self.FIELDS = [

            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            # PointField(name='my_field1', offset=16, datatype=PointField.FLOAT32, count=1),
            # PointField(name='my_field2', offset=20, datatype=PointField.FLOAT32, count=1),
        ]


    def create_points(self,l,b,h):

         # ([0.3, 0.0, 0.0, 0xff0000])
        number_of_pixels=5
        for p in xrange(0, l*20):
            for q in xrange(0, int(b*20)):
                    for r in xrange(0, h*20):
                        # rospy.loginfo(p)
                        x=p/5.0;
                        y=q/5.0;
                        z=r/5.0;

                        self.POINTS.append([x,y,z,0xff0000])
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
        custom_point_cloud = CustomPointCloud()
        custom_point_cloud.create_points(5,0.5,5)
        custom_point_cloud.publish_points()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()