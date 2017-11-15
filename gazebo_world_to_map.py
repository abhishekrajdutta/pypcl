#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from nav_msgs.msg import OccupancyGrid, MapMetaData
#from map_server import crop_map
import numpy as np
import cv2
import tf.transformations as transformations
import math
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
import tf2_ros

import xml.etree.ElementTree as ET


import threading

class MapCreator():
    def __init__(self):
        self.worldState = None
        self.resolution = .1
        #self.wall_size = [7.5,.25]
        self.wall_size = Vector3(x=7.5,y=.2,z=2.8)
        self.OCC_OBSTACLE = 100
        self.OCC_UNKNOWN = -1

        self.gazebo_frame_id = 'map'
        self.odom_frame = "odom"
        self.model_topic = '/gazebo/model_states'

        self.map_topic = "map"

        #rospy.init_node("map_creator")

        self.transform = None

        #world_file = '/home/justin/Documents/catkin_ws/src/navigation_test/configs/world/rectangular_room.world'
        world_file = '/home/justin/Documents/catkin_ws/src/navigation_test/configs/world/fourth_floor.world'

        #self.getXMLWalls(world_file)

        self.map_pub = None

        #self.tf_broadcaster_r = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_m = None


        self.tfBuffer = tf2_ros.Buffer()

        self.tf_listener = None

        #self.stateSub = rospy.Subscriber(self.model_topic, ModelStates, self.statesCallback, queue_size=1)

        #self.publishMapOdomTransform()

        rospy.on_shutdown(self.shutdown)
        
        #rospy.spin()

        #self.shutdown()



    def runMapOdomPublisher(self):
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_broadcaster_m = tf2_ros.TransformBroadcaster()

        self.transform_publisher_thread = threading.Thread(target=self.updateMapTransform)
        self.transform_publisher_thread.start()

        self.transform_updater = rospy.Timer(period = rospy.Duration(.5), callback = self.updateTfs)

    def runMapPublisher(self, grey_walls = True, world_file=None):
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size = 1, latch=True)
        freq = 1

        r = rospy.Rate(freq)

        try:
            keep_going = True
            while not rospy.is_shutdown() and keep_going:
                wall_objects = []
                if grey_walls:
                  wall_objects.extend(self.getGreyWalls())

                if world_file is not None:
                  wall_objects.extend(self.getXMLWalls(world_file))
                  if grey_walls == False:
                    keep_going = False

                self.updateMap(wall_objects=wall_objects)
                r.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Node shutdown")




    def shutdown(self):
        pass

    def transformToMat(self, transform_stamped):
        # numpy arrays to 4x4 transform matrix

        transform = transform_stamped.transform
        t_array = [transform.translation.x, transform.translation.y, transform.translation.z]
        quat_array = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        trans_mat = transformations.translation_matrix(t_array)
        rot_mat = transformations.quaternion_matrix(quat_array)
        # create a 4x4 matrix
        transf_mat = np.dot(trans_mat, rot_mat)
        return transf_mat

    #Based on http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche53.html
    def invertMatTransform(self, mat):
        inv = np.zeros_like(mat)
        R_t = np.transpose(mat[0:3,0:3])
        inv[0:3,0:3] = R_t
        inv[3,3] = 1
        Rt = -R_t.dot(mat[0:3,3])
        inv[0:3,3] = Rt
        return inv

    def matToTransform(self, mat):
        trans = TransformStamped()

        # go back to quaternion and 3x1 arrays
        rot_array = transformations.quaternion_from_matrix(mat)
        trans_array = transformations.translation_from_matrix(mat)

        trans.transform.translation.x = trans_array[0]
        trans.transform.translation.y = trans_array[1]
        trans.transform.translation.z = trans_array[2]

        trans.transform.rotation.x = rot_array[0]
        trans.transform.rotation.y = rot_array[1]
        trans.transform.rotation.z = rot_array[2]
        trans.transform.rotation.w = rot_array[3]

        return trans

    def updateTfs(self, event=None):

            model_states = self.getWorldState()
            Tr_m= self.getRobotTransform(model_states=model_states)
            if Tr_m is None:
                return None

            try:
                To_r = self.tfBuffer.lookup_transform("base_footprint", "odom", Tr_m.header.stamp, rospy.Duration(.1))   #replace Time() with Tm_r.header.stamp, plus add duration

                #Approach inspired by https://answers.ros.org/question/215656/how-to-transform-a-pose/?answer=215666#post-id-215666
                To_r_mat = self.transformToMat(To_r)
                Tr_m_mat = self.transformToMat(Tr_m)

                Tm_o_mat = Tr_m_mat.dot(To_r_mat)

                Tm_o = self.matToTransform(Tm_o_mat)
                Tm_o.header.frame_id = "map"
                Tm_o.header.stamp = To_r.header.stamp
                Tm_o.child_frame_id = "odom"

                self.tf_broadcaster_m.sendTransform(Tm_o)
                self.transform = Tm_o


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print "can't find odom to base"


    def updateMap(self, wall_objects):
        map = self.getMap(wall_objects=wall_objects)
        if map is not None:
            self.map_pub.publish(map)

    def updateMapTransform(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if(self.transform is not None):
                trans = self.transform
                trans.header.stamp = rospy.Time.now() + rospy.Duration(.1)
                self.tf_broadcaster_m.sendTransform(trans)
                r.sleep()


    def getXMLWalls(self,world_file):
        '''
        <collision name='Wall_2_Collision'>
          <geometry>
            <box>
              <size>6.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose frame=''>0 0 1.25 0 -0 0</pose>
        '''
        wall_objects={}

        tree = ET.parse(world_file)
        root = tree.getroot()
        for link in root.findall('./world/model/link'):
            link_name = link.attrib['name']
            if 'Wall_' in link_name:
                #print link.tag , link.attrib
                box_size_str = link.find('.collision/geometry/box/size').text
                box_size = [float(x) for x in box_size_str.split()]

                box_size_vec = Vector3(*box_size)
                #box_size_vec.x = box_size[0]
                #box_size_vec.y = box_size[1]
                #box_size_vec.z = box_size[2]


                wall_object = {}
                wall_object["size"] = box_size_vec

                wall_objects[link_name] = wall_object


        for link in root.findall('./world/state/model/link'):
            link_name = link.attrib['name']
            if link_name in wall_objects:
                wall_object = wall_objects[link_name]

                box_pose_str = link.find('./pose').text
                box_pose = [float(x) for x in box_pose_str.split()]

                pose = Pose()
                # pose.position.x = box_pose[0]
                # pose.position.y = box_pose[1]
                # pose.position.z = box_pose[2]
                pose.position = Vector3(*box_pose[0:3])

                orientation = transformations.quaternion_from_euler(*box_pose[3:6])

                # pose.orientation.x = orientation[0]
                # pose.orientation.y = orientation[1]
                # pose.orientation.z = orientation[2]
                # pose.orientation.w = orientation[3]
                pose.orientation = Quaternion(*orientation)
                wall_object["pose"] = pose
                #print str(link_name) + ":\nsize:\n" + str(wall_object["size"]) + "\n" + str(wall_object["pose"]) + "\n"

        return wall_objects.values()


    def getRobotTransform(self, model_states):
        robot_pose = self.getRobotPose(model_states)
        if(robot_pose is None):
            return None

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.gazebo_frame_id
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = robot_pose.position.x
        t.transform.translation.y = robot_pose.position.y
        t.transform.translation.z = robot_pose.position.z
        t.transform.rotation = robot_pose.orientation
        return t

    def publishRobotTransform(self,t):
        self.tf_broadcaster_r.sendTransform(t)


    def statesCallback(self,model_states):
        t = self.getRobotTransform(model_states=model_states)
        self.publishRobotTransform(t)


    def getWorldState(self):
        model_states = rospy.wait_for_message(self.model_topic, ModelStates)
        return model_states

    def getWallPoses(self, model_states):
        wall_poses = [model_states.pose[n] for n in xrange(len(model_states.name)) if "grey_wall" in model_states.name[n]]
        return wall_poses

    def getRobotPose(self, model_states):
        robot_poses = [model_states.pose[n] for n in xrange(len(model_states.name)) if "mobile_base" in model_states.name[n]]
        if len(robot_poses)==1:
            return robot_poses[0]
        else:
            return None

    def getGreyWalls(self, model_states=None):
        if model_states is None:
            model_states = self.getWorldState()
        wall_poses = self.getWallPoses(model_states=model_states)
        wall_objects = [{'pose': pose, 'size': self.wall_size} for pose in wall_poses]
        return wall_objects

    def getMap(self,wall_objects):
        if len(wall_objects) == 0:
            return None

        now = rospy.Time.now()

        half_wall_length = self.wall_size.x / 2.0

        #TODO: add/subtract half the length from each object as go along
        min_x = min([wall['pose'].position.x for wall in wall_objects])
        min_y = min([wall['pose'].position.y for wall in wall_objects])
        max_x = max([wall['pose'].position.x for wall in wall_objects])
        max_y = max([wall['pose'].position.y for wall in wall_objects])

        min_x -= half_wall_length
        min_y -= half_wall_length
        max_x += half_wall_length
        max_y += half_wall_length

        map_origin = Pose()
        map_origin.position.x = min_x
        map_origin.position.y = min_y#
        map_origin.orientation.w = 1# .505
        map_origin.orientation.z = 0#.505

        map_width = int((max_x - min_x) / self.resolution)
        map_height = int((max_y - min_y) / self.resolution)

        empty_map = self.OCC_UNKNOWN*np.ones(shape=(map_width,map_height), dtype=np.int8)

        map_image = empty_map

        #pts = np.array(shape=(len(wall_poses),2),dtype=np.int32)
        #pose_ind = 0
        for wall in wall_objects:
            pose = wall['pose']
            x_im = pose.position.y - min_y
            y_im = pose.position.x - min_x
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

            angle = transformations.euler_from_quaternion(quat)[2]

            wall_size = wall['size']
            half_wall_length = wall_size.x/2.0
            wall_thickness = int(wall_size.y / self.resolution)

            p1x = x_im + math.sin(angle) * half_wall_length
            p2x = x_im - math.sin(angle) * half_wall_length
            p1y = y_im + math.cos(angle) * half_wall_length
            p2y = y_im - math.cos(angle) * half_wall_length

            p1x = int(p1x/self.resolution)
            p2x = int(p2x/self.resolution)
            p1y = int(p1y/self.resolution)
            p2y = int(p2y/self.resolution)


            cv2.line(img=map_image, pt1=(p1x,p1y),pt2=(p2x,p2y),color=self.OCC_OBSTACLE,thickness=wall_thickness)

            #pts.append(((p1x,p1y),(p2x,p2y)))
            #cv2.imshow(winname="map",mat=map_image)
            #cv2.waitKey(10)

        #cv2.polylines(img=map_image,pts=pts,isClosed=False,color=self.OCC_OBSTACLE,thickness=wall_thickness)

        #cv2.imshow(winname="map",mat=map_image)
        #cv2.waitKey(1)

        map_msg = OccupancyGrid()
        map_msg.header.frame_id = self.gazebo_frame_id
        map_msg.header.stamp = now
        map_msg.data = np.reshape(a=map_image,newshape=-1,order='F').tolist()    #First flatten the array, then convert it to a list

        metadata = MapMetaData()
        metadata.resolution = self.resolution
        metadata.map_load_time = now
        metadata.width = map_width
        metadata.height = map_height

        metadata.origin = map_origin

        map_msg.info = metadata

        return map_msg
