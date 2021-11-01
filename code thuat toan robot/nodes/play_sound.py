#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
from playsound import *
import pyttsx3
import os
jarvis = pyttsx3.init() # Khởi tạo jarvis chuyển từ text sang giọng nói

def speak(audio): # Định nghĩa hàm để Jarvis nói
    print("JARVIS:", audio)
    tts = gTTS(text=audio, lang='vi', tld='com.vn')
    file = "speech.mp3"
    tts.save(file)
    playsound(file)
    os.remove(file)
    jarvis.runAndWait()
def listener():
    rospy.init_node('play_sound')
    rospy.Subscriber("/speech", String, callback)
    rospy.spin()
def callback(msg):
    if msg.data != '':
     speak(msg.data)
if __name__ == '__main__':
    try:

        listener()
    except rospy.ROSInterruptException:
        pass
