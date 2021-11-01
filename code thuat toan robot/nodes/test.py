#!/usr/bin/env python3
import os
import pyttsx3
from datetime import *
import speech_recognition as sr
import smach_ros
import smach
from gtts import gTTS
from playsound import *
import requests
import json

from time import *
jarvis = pyttsx3.init() # Khởi tạo jarvis chuyển từ text sang giọng nói
import rospy

from std_msgs.msg import String
class Recognizer(smach.State):
    def __init__(self,time = 10):
        smach.State.__init__(self, outcomes=['speak', 'mute'])
        self.time = time
        self.msg = String()
        self.file = open("/home/cuong/catkin_ws/src/rbx2/rbx2_tasks/nodes/hochoi.txt", 'r', encoding='utf-8')
        self.library = []
        for line in self.file:
            data = line.strip()
            self.library.append(data)
        self.file.close()

        self.pub = rospy.Publisher('/speech', String, queue_size=1)

        self.rate = rospy.Rate(5)
        self.check = 0
        self.audio = None

    def execute(self, userdata):
		
        t1 = datetime.now()
        while (datetime.now()-t1).seconds <= 20:
            query = self.command().lower()
            
            
            #query = acceptCommands().lower()

            if query in self.library:
                index = self.library.index(query)
                
                self.speak(self.library[index+1])     
            
                self.check=1
            elif 'ngày' in query:
                self.today()
                self.check=1
           
            elif 'có thể làm gì'  in  query:
                self.hotro()
                self.check=1
            else :
                self.check=0
          
                                  
            if (self.check == 1) :
               return "speak"
          
        return "mute"    
         
        
 
  

    def speak(self): # Định nghĩa hàm để Jarvis nói
        print("JARVIS:", self.audio)
        tts = gTTS(text=self.audio, lang='vi', tld='com.vn')
        file = "speech.mp3"
        tts.save(file)
        playsound(file)
        os.remove(file)
        jarvis.runAndWait()

    def today(self): # Hàm lấy ngày hiện tại
        date= datetime.now().strftime('%w')
        if date == '0':
            date = 'Chủ nhật'
        elif date == '1':
            date = 'Thứ Hai'
        elif date == '2':
            date = 'Thứ Ba'
        elif date == '3':
            date = 'Thứ Tư'
        elif date == '4':
            date = 'Thứ Năm'
        elif date == '5':
            date = 'Thứ Sáu'
        elif date == '6':
            date = 'Thứ Bảy'
        self.audio = date + ', ' + datetime.now().strftime('%d/%m/%Y')
        self.speak()

    def greeting(self): # Hàm để chào
        time = datetime.now().hour
        if 0 <= time <= 10:
            self.audio='Chào buổi sáng ! Tôi giúp gì được cho bạn ?'
        if 10 < time < 13:
            self.audio = 'Chào buổi trưa ! Tôi giúp gì được cho bạn ?'
        if 13 <= time < 19:
            self.audio='Chào buổi chiều ! Tôi giúp gì được cho bạn ?'
        if 19 <= time <= 24:
            self.audio='Chào buổi tối  ! Tôi giúp gì được cho bạn ? '

        self.speak()


    def command(self): # Nhận diện giọng nói rồi chuyển sang dạng text
        c = sr.Recognizer() # Khởi tạo biến nhận dạng giọng nói
        with sr.Microphone() as source: # Lấy nguồn nói từ Microphone
            print('Listening')
            #c.pause_threshold = 1 # Dừng 2s trước khi nhận lệnh mới
            audio = c.record(source, duration=5) # Biến audio là giá trị dạng chuỗi sau khi máy nghe và nhận dạng từ nguồn vào
            try:
                print("Recognizing")
                query = c.recognize_google(audio, language='vi') # Nhận diện bằng tiếng Việt và cho ra lệnh
                print("Bạn: ", query)
            except Exception as e:
                
                return "None"
                
            return query # Trả về lệnh


 
    def hotro(self):
       self.audio = " tôi có thể giúp gì cho bạn "
       self.speak()




