import vlc
import glob

audio_files = sorted(glob.glob("/home/mason/catkin_wss/audioPlayer_ws/src/audio_player/src/audiofiles/*"))
print audio_files

audio_player = vlc.MediaPlayer("/home/mason/catkin_wss/audioPlayer_ws/src/audio_player/src/audiofiles/Kathleen_Martin_-_02_-_Transistor_Sister_edit.mp3")
audio_player.play()
