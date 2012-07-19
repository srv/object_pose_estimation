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
import rosbag

def cut_bag(inbags, start_time, duration):
  cut_name = '/tmp/cut_bag.bag'
  cut_cmd = ['rosrun', 'bag_tools', 'cut.py', '--inbag']
  cut_cmd += inbags
  cut_cmd += ['--outbag', cut_name, '--start', str(start_time - duration / 2.0), '--duration', str(duration)]
  print '=== cutting input bagfile(s):', ' '.join(cut_cmd)
  subprocess.check_call(cut_cmd)
  return cut_name
 
def prepare_training_bag(inbags, start_time, duration, object_id, camera):
  cut_bag_name = cut_bag(inbags, start_time, duration)
  left_image_topic  = camera + '/left/image_raw'
  left_info_topic   = camera + '/left/camera_info'
  right_image_topic = camera + '/right/image_raw'
  right_info_topic  = camera + '/right/camera_info'
  topics = [left_image_topic, left_info_topic, right_image_topic, right_info_topic]
  # slow playback to have more point clouds from stereo_image_proc
  bag_play_cmd = ['rosbag', 'play', '--clock', '-r', '0.4', '-d', '2.0', cut_bag_name]
  bag_play_cmd.append('--topics')
  bag_play_cmd += topics
  namespace = '/object_pose_estimation'
  for t in topics:
    remapping = t + ':=' + namespace + t.replace(camera, '/stereo')
    bag_play_cmd.append(remapping)
  print '=== running PLAY process:', ' '.join(bag_play_cmd)
  play_process = subprocess.Popen(bag_play_cmd)
  image_proc_process = subprocess.Popen(['rosrun', 'stereo_image_proc', 'stereo_image_proc', '__ns:='+namespace+'/stereo', '_disparity_range:=128'])
  object_bagfile = '/tmp/' + object_id + '_training.bag'
  bag_record_cmd = ['rosbag', 'record', '-b', '1024', '-O', object_bagfile]
  prefix = namespace + '/stereo'
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
  print 'Looking for frame id...',
  bag = rosbag.Bag(bagfile)
  frame_id = ''
  for topic, msg, t in bag.read_messages(topics=['/object_pose_estimation/stereo/left/camera_info']):
    print msg
    frame_id = msg.header.frame_id
    break
  if not frame_id:
    raise
  print 'found:', frame_id
  start_time = time.time()
  rgbdslam_cmd = ['roslaunch', 'object_pose_estimation', 'stereo_rgbdslam.launch', 'stereo:=/object_pose_estimation/stereo', 'image_frame_id:=' + frame_id]
  print '=== running rgbdslam launch process:', ' '.join(rgbdslam_cmd)
  rgbdslam_process = subprocess.Popen(rgbdslam_cmd)
  bag_play_cmd = ['rosbag', 'play', '--clock', '-r', '0.2', '-d', '5.0', bagfile]
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

def create_object_model(inbags, start_time, duration, object_id, camera):
  bag = prepare_training_bag(inbags, start_time, duration, object_id, camera)
  pcd_filename, features_filename = reconstruct(bag)
  model_dir = roslib.packages.get_pkg_dir(PKG) + '/models'
  if not os.path.exists(model_dir):
    os.makedirs(model_dir)
  shutil.copyfile(pcd_filename, model_dir + '/' + object_id + '_raw.pcd')
  shutil.copyfile(features_filename, model_dir + '/' + object_id + '_features_raw.yaml')

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Train an object from a bagfile.')
  parser.add_argument('inbag', help='input bagfile', nargs='+')
  parser.add_argument('-t', '--start_time', required=True, type=float, help='timestamp of the object that has to be modeled in seconds from the beginning of the bagfile')
  parser.add_argument('-d', '--duration', type=float, default=10.0, help='time window of the object')
  parser.add_argument('-o', '--object_id', required=True, help='name for the object that is trained')
  parser.add_argument('-c', '--camera', required=True, help='base topic of the camera to use')
  args = parser.parse_args()
 
  try:
    create_object_model(args.inbag, args.start_time, args.duration, args.object_id, args.camera)
  except Exception, e:
    import traceback
    traceback.print_exc()
