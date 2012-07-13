#!/usr/bin/python

PKG = 'object_pose_estimation' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import os
import sys
import argparse
import subprocess
import time

def create_object_model(inbags, start_time, object_id):
  namespace = 'object_pose_estimation'
  stereo = '/stereo_down'
  left_image_topic  = stereo + '/left/image_raw'
  left_info_topic   = stereo + '/left/camera_info'
  right_image_topic = stereo + '/right/image_raw'
  right_info_topic  = stereo + '/right/camera_info'
  topics = [left_image_topic, left_info_topic, right_image_topic, right_info_topic]
  bag_play_cmd = ['rosbag', 'play', '--clock', '--start', start_time, '-d', '2.0']
  bag_play_cmd += inbags
  bag_play_cmd.append('--topics')
  bag_play_cmd += topics
  for t in topics:
    remapping = t + ':=' + namespace + t
    bag_play_cmd.append(remapping)
  print '=== running PLAY process:', ' '.join(bag_play_cmd)
  play_process = subprocess.Popen(bag_play_cmd, stdout=subprocess.PIPE)
  image_proc_process = subprocess.Popen(['rosrun', 'stereo_image_proc', 'stereo_image_proc', '__ns:='+namespace+stereo])
  bag_record_cmd = ['rosbag', 'record', '-b', '1024', '-O', '/tmp/'+object_id+'_training.bag']
  prefix = namespace + stereo
  rec_topics = [prefix + '/left/image_rect_color', prefix + '/left/camera_info', prefix + '/right/image_rect_color', prefix + '/right/camera_info', prefix + '/points2']
  bag_record_cmd += rec_topics
  print '=== running RECORD process:', ' '.join(bag_record_cmd)
  record_process = subprocess.Popen(bag_record_cmd)
  i = 0
  while i < 10:
    print 'player output is:', play_process.stdout.read()
    time.sleep(1)
    i = i + 1

  play_process.terminate()
  image_proc_process.terminate()
  record_process.terminate()




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
