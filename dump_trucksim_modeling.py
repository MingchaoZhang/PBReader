#!/usr/bin/env python

import rosbag
import argparse
import tf
import euler
from dbw_mkz_msgs.msg import *
from control import dbw_reports_pb2

# Make signal naming align with TruckSim variables
# HOW to run
# python dump_trucksim_modeling.py xxx.bag

def get_topic_map(bag_path):
    # for FAW J7
    return {
            '/vehicle/steering_report': (file(bag_path + '.steering_report.tsv', 'w'), dump_steering_report),
            '/imu/data': (file(bag_path + '.imu.tsv', 'w'), dump_imu),
            '/navsat/odom': (file(bag_path + '.odom.tsv', 'w'), dump_odom),
            '/vehicle/throttle_info_report': (file(bag_path + '.throttle_info_report.tsv', 'w'), dump_throttle_info_report),
            '/vehicle/throttle_report': (file(bag_path + '.throttle_report.tsv', 'w'), dump_throttle_report),
            '/vehicle/gear_report': (file(bag_path + '.gear_report.tsv', 'w'), dump_gear_report),
            '/vehicle/vehicle_dynamic': (file(bag_path + '.vehicle_dynamic.tsv', 'w'), dump_vehicle_dynamic),
            '/vehicle/dbw_reports': (file(bag_path + '.reports.tsv', 'w'), dump_dbw_reports),
            }

def msg_time(msg):
    return msg.header.stamp.to_sec()


def dump_odom(odom, bag_time, f, write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'utm_x', 'utm_y', 'altitude', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz']) + "\n")
    orientation_matrix = tf.transformations.quaternion_matrix([odom.pose.pose.orientation.x,
                                                               odom.pose.pose.orientation.y,
                                                               odom.pose.pose.orientation.z,
                                                               odom.pose.pose.orientation.w])
    roll, pitch, yaw = euler.recover_euler(orientation_matrix)[0]
    f.write("%.3f\t%.3f\t%.6f\t%.6f\t%.6f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n" %
            (bag_time, msg_time(odom),
             odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
             roll, pitch, yaw,
             odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z))

def dump_imu(msg, bag_time, f, write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'roll', 'pitch', 'yaw', 'lateral_accel', 'longitu_accel', 'z_accel', \
            'pitch_rate', 'roll_rate', 'yaw_rate']) + "\n")
    orientation_matrix = tf.transformations.quaternion_matrix([msg.orientation.x,
                                                               msg.orientation.y,
                                                               msg.orientation.z,
                                                               msg.orientation.w])
    roll, pitch, yaw = euler.recover_euler(orientation_matrix)[0]
    f.write('%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n' %
            (bag_time, msg_time(msg),
             roll, pitch, yaw,
             msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
             msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

def dump_steering_report(msg, bag_time, f, write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'vehicle_speed']) + "\n")
    f.write("%.3f\t%.3f\t%.3f\n" % (bag_time, msg_time(msg),
                                                    msg.speed))


def dump_throttle_info_report(msg, bag_time, f, write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'engine_rpm']) + "\n")
    f.write("%.3f\t%.3f\t%.3f\n" % (bag_time, msg_time(msg),
                                    msg.engine_rpm))

def dump_throttle_report(msg, bag_time, f, write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'pedal_input', 'pedal_cmd']) + "\n")
    f.write("%.3f\t%.3f\t%.3f\t%.3f\n" % (bag_time, msg_time(msg),
                                              msg.pedal_output,
                                              msg.pedal_cmd))
    

def dump_gear_report(msg,bag_time,f,write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'state_gear']) + "\n")
    f.write("%.3f\t%.3f\t%d\n" % (bag_time, msg_time(msg),
                                  msg.state.gear))

def dump_dbw_reports(msg,bag_time,f,write_header=False):
    pb = dbw_reports_pb2.DbwReports()
    pb.ParseFromString(msg.data)
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'brake']) + "\n")
    f.write("%.3f\t%.3f\t%.3f\n" % (bag_time, pb.header.envelope_timestamp_msec,
                                  pb.brake_report.pedal_cmd))

def dump_vehicle_dynamic(msg,bag_time,f,write_header=False):
    if write_header:
        f.write("\t".join(['bag_time', 'msg_time', 'lateral_accel', 'longitu_accel', 'yaw_rate']) + "\n")
    f.write("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n" % (bag_time, msg_time(msg), 
                                                msg.linear_acceleration.x, 
                                                msg.linear_acceleration.y,
                                                msg.angular_velocity.z))



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=str, help="path to bag (can be more than one, separated by commas/spaces)")

    args = parser.parse_args()
    # args.bag is bag path
    topic_map = get_topic_map(args.bag) 
    bag = rosbag.Bag(args.bag)
    seen_topics = {}
    for topic, msg, bag_time in bag.read_messages(topics=topic_map.keys()):
        # get the tsv file and corresponding dumper function
        tsv_file, dumper = topic_map[topic]
        dumper(msg, bag_time.to_sec(), tsv_file, topic not in seen_topics)
        seen_topics[topic] = True
