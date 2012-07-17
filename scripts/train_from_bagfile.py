#!/usr/bin/python

PKG = 'object_pose_estimation' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import os
import sys
import argparse
import subprocess
import signal
import time
import roslib
import shutil

def cut_bag(inbags, start_time):
  cut_name = '/tmp/cut_bag.bag'
  cut_cmd = ['rosrun', 'bag_tools', 'cut.py', '--inbag']
  cut_cmd += inbags
  cut_cmd += ['--outbag', cut_name, '--start', str(float(start_time) - 5), '--duration', '10']
  print '=== cutting input bagfile(s):', ' '.join(cut_cmd)
  subprocess.check_call(cut_cmd)
  return cut_name
 
def prepare_bag(inbags, start_time, object_id):
  cut_bag_name = cut_bag(inbags, start_time)
  namespace = 'object_pose_estimation'
  stereo = '/stereo_down'
  left_image_topic  = stereo + '/left/image_raw'
  left_info_topic   = stereo + '/left/camera_info'
  right_image_topic = stereo + '/right/image_raw'
  right_info_topic  = stereo + '/right/camera_info'
  topics = [left_image_topic, left_info_topic, right_image_topic, right_info_topic]
  # slow playback to have more point clouds from stereo_image_proc
  bag_play_cmd = ['rosbag', 'play', '--clock', '-r', '0.4', '-d', '2.0', cut_bag_name]
  bag_play_cmd.append('--topics')
  bag_play_cmd += topics
  for t in topics:
    remapping = t + ':=' + namespace + t
    bag_play_cmd.append(remapping)
  print '=== running PLAY process:', ' '.join(bag_play_cmd)
  play_process = subprocess.Popen(bag_play_cmd)
  image_proc_process = subprocess.Popen(['rosrun', 'stereo_image_proc', 'stereo_image_proc', '__ns:='+namespace+stereo, '_disparity_range:=128'])
  object_bagfile = '/tmp/' + object_id + '_training.bag'
  bag_record_cmd = ['rosbag', 'record', '-b', '1024', '-O', object_bagfile]
  prefix = namespace + stereo
  rec_topics = [prefix + '/left/image_rect_color', prefix + '/left/camera_info', prefix + '/points2', prefix + '/right/image_rect_color', prefix + '/right/camera_info']
  bag_record_cmd += rec_topics
  print '=== running RECORD process:', ' '.join(bag_record_cmd)
  record_process = subprocess.Popen(bag_record_cmd)
  play_process.wait()
  # stop processes when playback finished
  image_proc_process.send_signal(signal.SIGINT)
  image_proc_process.wait()
  record_process.send_signal(signal.SIGINT)
  record_process.wait()
  print '=== bagfile for object training created:', object_bagfile
  return object_bagfile

def reconstruct(bagfile):
  start_time = time.time()
  frame_id = '/stereo_down'
  rgbdslam_cmd = ['roslaunch', 'object_pose_estimation', 'stereo_rgbdslam.launch', 'stereo:=/object_pose_estimation/stereo_down', 'image_frame_id:=' + frame_id]
  print '=== running rgbdslam launch process:', ' '.join(rgbdslam_cmd)
  rgbdslam_process = subprocess.Popen(rgbdslam_cmd)
  bag_play_cmd = ['rosbag', 'play', '--clock', '-r', '0.5', '-d', '10.0', bagfile]
  print '=== running PLAY process:', ' '.join(bag_play_cmd)
  play_process = subprocess.Popen(bag_play_cmd)
  play_process.wait()
  subprocess.check_call(['rosservice', 'call', '/rgbdslam/ros_ui', 'save_cloud'])
  subprocess.check_call(['rosservice', 'call', '/rgbdslam/ros_ui', 'save_features'])
  rgbdslam_working_directory = roslib.packages.get_pkg_dir('rgbdslam') + '/bin'
  pcd_filename = rgbdslam_working_directory + '/quicksave.pcd'
  features_filename = rgbdslam_working_directory + '/feature_database.yml'
  last_t = time.time()
  print 'Waiting for rgbdslam to finish saving point cloud'
  while True:
    print '.',
    t = os.path.getmtime(pcd_filename)
    if t == last_t and t > start_time:
      break
    last_t = t
    time.sleep(1)
  last_t = time.time()
  print 'done.'
  print 'Waiting for rgbdslam to finish saving features',
  while True:
    print '.',
    t = os.path.getmtime(features_filename)
    if t == last_t and t > start_time:
      break
    last_t = t
    time.sleep(1)
  print 'done.'
  rgbdslam_process.send_signal(signal.SIGINT)
  return pcd_filename, features_filename

def create_object_model(inbags, start_time, object_id):
  bag = prepare_bag(inbags, start_time, object_id)
  pcd_filename, features_filename = reconstruct(bag)
  shutil.copyfile(pcd_filename, '/tmp/' + object_id + '_raw.pcd')
  shutil.copyfile(features_filename, '/tmp/' + object_id + '_features_raw.yaml')

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Train an object from a bagfile.')
  parser.add_argument('inbag', help='input bagfile', nargs='+')
  parser.add_argument('-t', '--start_time', required=True, help='timestamp of the object that has to be modeled in seconds from the beginning of the bagfile')
  parser.add_argument('-o', '--object_id', required=True, help='name for the object that is trained')
  args = parser.parse_args()
 
  try:
    create_object_model(args.inbag, args.start_time, args.object_id)
  except Exception, e:
    import traceback
    traceback.print_exc()
