#!/usr/bin/env python
import os, sys, time
import subprocess
import csv
from optparse import OptionParser
import pandas as pd
import argparse
import rosbag
import rospy
import Queue
import numpy as np
from std_msgs.msg import String
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import logging
import math
import time
from dbw_mkz_msgs.msg import *
from planning import planning_trajectory_pb2
from perception import obstacle_detection_pb2
from perception import lane_detection_pb2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2

HeaderRadar = ['time','obj_id','track_id','obj_pos_x','obj_pos_y','obj_vel_x','obj_vel_y','obj_acc_x','obj_acc_y']
TopicsRadar = ['/conti_bumper_radar/radar_tracks']
HeaderCamera = ['time','obj_id','track_id','obj_pos_x','obj_pos_y']
TopicsCamera = ['/perception/obstacles']
HeaderLead = ['time','lead_id','lead_distance','lead_speed','lead_ttc']
TopicsLead = ['/planning/lead_info']
HeaderImage = ['time','frame_id']
TopicsImage = ['/front_left_camera/image_color/compressed', '/front_right_camera/image_color/compressed']
HeaderOdom = ['time','ego_pos_x','ego_pos_y','ego_z','ego_w']
TopicsOdom = ['/navsat/odom']
HeaderLane = ['time','Lane.x','Lane.y']
TopicsLane = ['/perception/lane_path']
FL_path = './FrontLeftImg/'
FR_path = './FrontRightImg/'

def hasBag(path):
    for root, _, files in os.walk(path):
        for file in files:
            if file.endswith('.bag.active') or file.endswith('.bag'):
                return True
    return False

def process(path):
    bag_name = ''
    radar_csv_name = ''
    camera_csv_name = ''
    image_l_csv_name = ''
    image_r_csv_name = ''
    odom_csv_name = ''
    lane_l_csv_name = ''
    lane_r_csv_name = ''
    if hasBag(path):
        for root, _, files in os.walk(path):
            os.mkdir('FrontLeftImg')
            os.mkdir('FrontRightImg')
            for file in files:
                if file.endswith('.bag.active') or file.endswith('.bag'):
                    bag_name = os.path.join(root, file)
                    if file.endswith('.bag.active'):
                        cmd = 'rosbag reindex ' + '\'' + bag_name + '\''
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    # ****** try to get radar data, ros data
                    radar_csv_name = bag_name.replace('.bag','Radar.csv')
                    print ' ', bag_name
                    print ' ', radar_csv_name

                    if os.path.exists(radar_csv_name):
                        cmd = 'rm -f ' + radar_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    csv_handle = open(radar_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderRadar)
                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, _ in bag_content.read_messages(topics=TopicsRadar):
                            if topic == '/conti_bumper_radar/radar_tracks':
                                # numtracks = len(msg.tracks)
                                if len(msg.tracks) == 0:
                                    continue
                                else:
                                    for i in range(len(msg.tracks)):                                        
                                        obj_id = msg.tracks[i].track_id
                                        track_id = i
                                        obj_pos_x = msg.tracks[i].track_shape.points[0].x
                                        obj_pos_y = msg.tracks[i].track_shape.points[0].y
                                        obj_vel_x = msg.tracks[i].linear_velocity.x
                                        obj_vel_y = msg.tracks[i].linear_velocity.y
                                        obj_accel_x = msg.tracks[i].linear_acceleration.x
                                        obj_accel_y = msg.tracks[i].linear_acceleration.y
                                        row_str = [str(msg.header.stamp),
                                            str(obj_id),
                                            str(track_id),
                                            str(obj_pos_x),
                                            str(obj_pos_y),
                                            str(obj_vel_x),
                                            str(obj_vel_y),
                                            str(obj_accel_x),
                                            str(obj_accel_y)]
                                        csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()
                    # ****** get radar data finish                                 

                    # ****** try to get odom data, ros data
                    odom_csv_name = bag_name.replace('.bag','Odom.csv')
                    print ' ', bag_name
                    print ' ', odom_csv_name

                    if os.path.exists(odom_csv_name):
                        cmd = 'rm -f ' + odom_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    csv_handle = open(odom_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderOdom)                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, _ in bag_content.read_messages(topics=TopicsOdom):
                            if topic == '/navsat/odom': 
                                ego_pos_x = msg.pose.pose.position.x
                                ego_pos_y = msg.pose.pose.position.y
                                ego_z = msg.pose.pose.orientation.z
                                ego_w = msg.pose.pose.orientation.w
                                row_str = [str(msg.header.stamp),
                                    str(ego_pos_x),
                                    str(ego_pos_y),
                                    str(ego_z),
                                    str(ego_w)]
                                csv_stream.writerow(row_str)

                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()
                    # ****** get odom data finish

                    # ****** try to get perception data, pb data
                    camera_csv_name = bag_name.replace('.bag','Camera.csv')
                    print ' ', bag_name
                    print ' ', camera_csv_name

                    if os.path.exists(camera_csv_name):
                        cmd = 'rm -f ' + camera_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    csv_handle = open(camera_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderCamera)                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, t in bag_content.read_messages(topics=TopicsCamera):
                            if topic == '/perception/obstacles': 
                                pb = obstacle_detection_pb2.ObstacleDetection()
                                pb.ParseFromString(msg.data)
                                if len(pb.obstacle) == 0:
                                    continue
                                else:
                                    for i in range(len(pb.obstacle)):                                        
                                        obj_id = pb.obstacle[i].id      
                                        track_id = i
                                        obj_pos_x = pb.obstacle[i].motion.x
                                        obj_pos_y = pb.obstacle[i].motion.y
                                        row_str = [str(t),
                                            str(obj_id),
                                            str(track_id),
                                            str(obj_pos_x),
                                            str(obj_pos_y)]
                                        csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()
                    # ****** get perception data finish

                    # ****** try to get perception lane markers data
                    lane_l_csv_name = bag_name.replace('.bag','LaneL.csv')
                    lane_r_csv_name = bag_name.replace('.bag','LaneR.csv')
                    print ' ', bag_name
                    print ' ', lane_l_csv_name

                    if os.path.exists(lane_l_csv_name):
                        cmd = 'rm -f ' + lane_l_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    if os.path.exists(lane_r_csv_name):
                        cmd = 'rm -f ' + lane_r_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    csv_handle = open(lane_l_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderLane)                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, t in bag_content.read_messages(topics=TopicsLane):
                            if topic == '/perception/lane_path': 
                                pb = lane_detection_pb2.LaneDetection()
                                pb.ParseFromString(msg.data)
                                row_str = [str(t)]
                                for i in range(len(pb.lane[0].left_boundary.curve.segment[0].line_segment.point)):
                                    lane_l_pos_x = pb.lane[0].left_boundary.curve.segment[0].line_segment.point[i].x
                                    lane_l_pos_y = pb.lane[0].left_boundary.curve.segment[0].line_segment.point[i].y
                                    row_str.append(str(lane_l_pos_x))
                                    row_str.append(str(lane_l_pos_y))
                                csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()


                    csv_handle = open(lane_r_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderLane)                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, t in bag_content.read_messages(topics=TopicsLane):
                            if topic == '/perception/lane_path': 
                                pb = lane_detection_pb2.LaneDetection()
                                pb.ParseFromString(msg.data)
                                row_str = [str(t)]
                                for i in range(len(pb.lane[0].right_boundary.curve.segment[0].line_segment.point)):
                                    lane_r_pos_x = pb.lane[0].right_boundary.curve.segment[0].line_segment.point[i].x
                                    lane_r_pos_y = pb.lane[0].right_boundary.curve.segment[0].line_segment.point[i].y
                                    row_str.append(str(lane_r_pos_x))
                                    row_str.append(str(lane_r_pos_y))
                                csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()
                    # ****** get perception lane markers data finish

                    # ****** try to get lead data, pb data
                    lead_csv_name = bag_name.replace('.bag','Lead.csv')
                    print ' ', bag_name
                    print ' ', lead_csv_name

                    if os.path.exists(lead_csv_name):
                        cmd = 'rm -f ' + lead_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    csv_handle = open(lead_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderLead)                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, t in bag_content.read_messages(topics=TopicsLead):
                            if topic == '/planning/lead_info': 
                                pb = planning_trajectory_pb2.LeadInfo()
                                pb.ParseFromString(msg.data)                                      
                                lead_id = pb.id      
                                lead_distance = pb.distance
                                lead_speed = pb.speed
                                lead_ttc = pb.ttc
                                row_str = [str(t),
                                    str(lead_id),
                                    str(lead_distance),
                                    str(lead_speed),
                                    str(lead_ttc)]
                                csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()
                    # ****** get lead data finish

                   
                    # ****** try to get image data, rosdata
                    image_l_csv_name = bag_name.replace('.bag','LImage.csv')
                    image_r_csv_name = bag_name.replace('.bag','RImage.csv')
                    print ' ', bag_name
                    print ' ', image_l_csv_name
                    print ' ', image_r_csv_name

                    if os.path.exists(image_l_csv_name):
                        cmd = 'rm -f ' + image_l_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()
                    if os.path.exists(image_r_csv_name):
                        cmd = 'rm -f ' + image_r_csv_name
                        sp = subprocess.Popen(cmd, shell=True)
                        sp.wait()

                    csv_handle = open(image_l_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderImage)                    
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, _ in bag_content.read_messages(topics=TopicsImage):
                            if topic == '/front_left_camera/image_color/compressed': 
                                try:
                                    if msg.format.find("jpeg")!=-1 :
                                        np_arr = np.fromstring(msg.data, np.uint8)
                                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                except CvBridgeError as e:
                                    print e
                                timestr = "%.6f" %  msg.header.stamp.to_sec()
                                image_name = timestr+ "_L.png"
                                cv2.imwrite(FL_path + image_name, cv_image)
                                row_str = [str(msg.header.stamp)]
                                csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()

                    csv_handle = open(image_r_csv_name, 'a')
                    csv_stream = csv.writer(csv_handle)
                    csv_stream.writerow(HeaderImage)
                    try:
                        bag_content = rosbag.Bag(bag_name)
                        for topic, msg, _ in bag_content.read_messages(topics=TopicsImage):
                            if topic == '/front_right_camera/image_color/compressed': 
                                try:
                                    if msg.format.find("jpeg")!=-1 :
                                        np_arr = np.fromstring(msg.data, np.uint8)
                                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                except CvBridgeError as e:
                                    print e
                                timestr = "%.6f" %  msg.header.stamp.to_sec()
                                image_name = timestr+ "_R.png"
                                cv2.imwrite(FR_path + image_name, cv_image)
                                row_str = [str(msg.header.stamp)]
                                csv_stream.writerow(row_str)
                    except Exception as e:
                        print e
                        return
                    csv_handle.flush()
                    csv_handle.close()
                    # ****** get image perception data finish



if __name__ == '__main__':
    paser = OptionParser()
    paser.add_option("-p", "--path", dest="dir", help="bag path")
    (options,args) = paser.parse_args()
    if options.dir == None:
        print 'warning: Please input rosbag path!'
        print 'usage: python cut_in_data_2_csv.py -p [path]'
        exit(1)
    print '> Start!!!'
    for dir in os.listdir(options.dir):
        if not dir.startswith('.'):
            dir = os.path.join(options.dir,dir)
            if os.path.isdir(dir):
                print '> processing folder:',dir
                try:
                    process(dir)

                except Exception as e:
                    print('error type ***',e.__class__.__name__)
                    print('error detail ***',e)
    print '> Finish!!!'