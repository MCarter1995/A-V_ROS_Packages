#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
import vlc
import glob
import time

file_path = rospy.get_param('/audio_file_path')
waypoint_topic = rospy.get_param('/wp_topic')
as_topic = rospy.get_param('/as_topic')

audio_files = sorted(glob.glob(file_path + "/*"))
num_files = len(audio_files)

def callback(data):
    waypoint = data.data
    if any(waypoint in s for s in audio_files):
    	pub.publish(1)
    	audio_instance = vlc.Instance()
    	audio_player = audio_instance.media_player_new()
    	audio = audio_instance.media_new(file_path + "/" + waypoint + ".mp3")
    	audio_player.set_media(audio)
    	audio_player.play()
    	#print audio_player.get_length()
    	#debug = "File Length: %d" % audio_player.get_state()
    	pub.publish(2)
    	time.sleep(0.5)
    	current_state = audio_player.get_state()

    	pub.publish(3)	# Playing

    	while(1):
    		current_state = audio_player.get_state()
    		if current_state != 3:
    			pub.publish(6)	# Ended
    			break
    else:
    	pub.publish(7)	# Error

pub = rospy.Publisher(as_topic, Int16, queue_size=10)
rospy.init_node('audio_player', anonymous=True)
rospy.Subscriber(waypoint_topic, String, callback)

rospy.spin()

# 0: 'NothingSpecial',
# 1: 'Opening',
# 2: 'Buffering',
# 3: 'Playing',
# 4: 'Paused',
# 5: 'Stopped',
# 6: 'Ended',
# 7: 'Error'
