import threading
import requests # Getting the micro controller data  
import socket
import json 
import pickle
import getpass 
import pandas as pd
import gpiozero 
from gpiozero import Robot, MCP3008 # Getting the analog value input from the library 
from gpiozero import PhaseEnableMotor # Getting the motor to working 
import face_recognition # Getting the face recognition to working 
import pyfirmata # Getting the pyfirmata for the librery of the serial communication between the hardware
from pyzbar import pyzbar 
from printrun.printcore import printcore
from printrun import gcoder
import serial 
import csv 
import re 
import os ,sys ,time 
import datetime # Getting date time data 
from itertools import count
import cv2,imutils,subprocess
import math 
import numpy as np 
from pyzbar import pyzbar
import base64
import smbus 
#import cvlib as cv
#from cvlib.object_detection import draw_bbox
import speech_recognition as sr 
from google_speech import* 
from googletrans import Translator 
import wordninja 
import difflib 
from geopy.geocoders import Nominatim # Getting the geo positioning data 

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                                               #These function only able to enable from the singleboard computer 

#Support python version 3.7+ 3.8+ 
#Rpi fully support on the debian OS
#For the Jetson nano running on python version 3.6 not support 

# I2C servo motor board with the PCA9685
from board import SCL, SDA
import busio

# Import the PCA9685 module.
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import adafruit_mpu6050 
import adafruit_icm20x
# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
try: 
  print("i2c devices ",i2c_bus.scan())      
  pca = PCA9685(i2c_bus)
  pca.frequency = 50
except:
   print("No servo devices connect with the computer")
mem_sub_variable = [] # mem subscriber return variable 
mem_used_pins = {} # Collected the used pins on the list to avoid clash on the system hardware control 
Board_pins_sbc = {"Raspberry_pi_4":[{'label': '3.3V',      'type': 'Power'},
        {'label': '5V',        'type': 'Power'},
        {'label': 'BCM 2',     'type': 'IO'},
        {'label': '5V',        'type': 'Power'},
        {'label': 'BCM 3',     'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 4',     'type': 'IO'},
        {'label': 'BCM 14',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 15',    'type': 'IO'},
        {'label': 'BCM 17',    'type': 'IO'},
        {'label': 'BCM 18',    'type': 'IO'},
        {'label': 'BCM 27',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 22',    'type': 'IO'},
        {'label': 'BCM 23',    'type': 'IO'},
        {'label': '3.3V',      'type': 'Power'},
        {'label': 'BCM 24',    'type': 'IO'},
        {'label': 'BCM 10',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 9',     'type': 'IO'},
        {'label': 'BCM 25',    'type': 'IO'},
        {'label': 'BCM 11',    'type': 'IO'},
        {'label': 'BCM 8',     'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 7',     'type': 'IO'},
        {'label': 'BCM 0',     'type': 'IO'},
        {'label': 'BCM 1',     'type': 'IO'},
        {'label': 'BCM 5',     'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 6',     'type': 'IO'},
        {'label': 'BCM 12',    'type': 'IO'},
        {'label': 'BCM 13',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 19',    'type': 'IO'},
        {'label': 'BCM 16',    'type': 'IO'},
        {'label': 'BCM 26',    'type': 'IO'},
        {'label': 'BCM 20',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 21',    'type': 'IO'}]}

# Speech recognition function and Text to speech         
translator = Translator(service_urls=['translate.google.com','translate.google.com',])


#List language of translation function 
Languages = {
    'af': 'Afrikaans',
    'sq': 'Albanian',
    'am': 'Amharic',
    'ar': 'Arabic',
    'hy': 'Armenian',
    'az': 'Azerbaijani',
    'eu': 'Aasque',
    'be': 'Belarusian',
    'bn': 'Bengali',
    'bs': 'Bosnian',
    'bg': 'Bulgarian',
    'ca': 'Batalan',
    'ceb': 'Bebuano',
    'ny': 'Chichewa',
    'zh-cn': 'Chinese',
    'zh-tw': 'Chinese (traditional)',
    'co': 'Corsican',
    'hr': 'Croatian',
    'cs': 'Czech',
    'da': 'Danish',
    'nl': 'Dutch',
    'en': 'English',
    'eo': 'Esperanto',
    'et': 'Estonian',
    'tl': 'Filipino',
    'fi': 'Finnish',
    'fr': 'French',
    'fy': 'Frisian',
    'gl': 'Galician',
    'ka': 'Georgian',
    'de': 'German',
    'el': 'Greek',
    'gu': 'Gujarati',
    'ht': 'Haitian creole',
    'ha': 'Hausa',
    'haw': 'Hawaiian',
    'iw': 'Hebrew',
    'he': 'Hebrew',
    'hi': 'Hindi',
    'hmn': 'Hmong',
    'hu': 'Hungarian',
    'is': 'Icelandic',
    'ig': 'Igbo',
    'id': 'Indonesian',
    'ga': 'Irish',
    'it': 'Italian',
    'ja': 'Japanese',
    'jw': 'Javanese',
    'kn': 'Kannada',

    'kk': 'Kazakh',
    'km': 'Khmer',
    'ko': 'Korean',
    'ku': 'Kurdish (kurmanji)',
    'ky': 'Kyrgyz',
    'lo': 'Lao',
    'la': 'Latin',
    'lv': 'Latvian',
    'lt': 'Lithuanian',
    'lb': 'Luxembourgish',
    'mk': 'Macedonian',
    'mg': 'Malagasy',
    'ms': 'Malay',
    'ml': 'Malayalam',
    'mt': 'Maltese',
    'mi': 'Maori',
    'mr': 'Marathi',
    'mn': 'Mongolian',
    'my': 'Myanmar (burmese)',
    'ne': 'Nepali',
    'no': 'Norwegian',
    'or': 'Odia',
    'ps': 'Pashto',
    'fa': 'Persian',
    'pl': 'Polish',
    'pt': 'Portuguese',
    'pa': 'Punjabi',
    'ro': 'Romanian',
    'ru': 'Russian',
    'sm': 'Samoan',
    'gd': 'Scots gaelic',
    'sr': 'Serbian',
    'st': 'Sesotho',
    'sn': 'Shona',
    'sd': 'Sindhi',
    'si': 'Sinhala',
    'sk': 'Slovak',
    'sl': 'Slovenian',
    'so': 'Somali',
    'es': 'Spanish',
    'su': 'Sundanese',
    'sw': 'Swahili',
    'sv': 'Swedish',
    'tg': 'Tajik',
    'ta': 'Tamil',
    'te': 'Telugu',
    'th': 'Thai',
    'tr': 'Turkish',
    'uk': 'Ukrainian',
    'ur': 'Urdu',
    'ug': 'Uyghur',
    'uz': 'Uzbek',
    'vi': 'Vietnamese',
    'cy': 'Welsh',
    'xh': 'Xhosa',
    'yi': 'Yiddish',
    'yo': 'Yoruba',
    'zu': 'Zulu'}
class Microcontroller_pins(object): # Choose one microcontroller to control the pheripheral devices 
               #Request the microcontroller data from the api link 
               def request_mcu(self,mcu_code):

                     r = requests.get("https://raw.githubusercontent.com/KornbotDevUltimatorKraton/mcusdata.github.io/main/"+mcu_code+".json")
                     status = r.status_code 
                     data = r.json()
                     Pins_label = {}
                     Pins_list = []
                     Sub_label = {}
                     Sub_label2 = {}
                     ref_sub = {}
                     #sprint(status,data)
                     get_pins = data.get('Mcu') # getting the data import from the mcu requests
                     #print(get_pins)
                     print("Package_infos")
                     #for r in range(0,len(get_pins)):       
                     #        print(list(get_pins)[r])
                     #print(get_pins.get("Pin"))
                     for pins in range(0,len(get_pins.get("Pin"))):
       
                               #print(list(get_pins.get("Pin"))[pins])
                               Sub_label['label'] = list(get_pins.get("Pin"))[pins].get("@Name")             
                               Sub_label2['type'] = list(get_pins.get("Pin"))[pins].get("@Type")
                               #print(Sub_label,Sub_label2)
                               if len(Sub_label) > 1: 
                                    del Sub_label[next(iter(Sub_label))]
                                    print(Sub_label) 
                               if len(Sub_label2) > 1:
                                    del Sub_label2[next(iter(Sub_label2))]  
                                    print(Sub_label2)
                               ref_sub[0] = eval(str(Sub_label)),eval(str(Sub_label2))
                               Pins_list.append(ref_sub.get(0))
                               Pins_label[mcu_code] = Pins_list
                               #print(Pins_list)
                               #Pins_label[mcu_name[0]] = Pins_list
                               #print(Pins_label)
                               Pins_label.get(mcu_code)
                               #for r in Pins_label.get(mcu_code): 
                               #                print(r)
                     return json.dumps(Pins_label)
class Internal_Publish_subscriber(object): 
        
        def Publisher_dict(self,ip,input_message,port):
            try: 
              exec("sock_"+str(port)+" =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
              jsondata = json.dumps(input_message)
              message = pickle.dumps(jsondata)
              exec("sock_"+str(port)+".sendto(message,(ip,port))") # Sending the json data into the udp  
            except ValueError: 
                  print("Connection error via ip: ",str(ip))
                  return 
        def Publisher_string(self,ip,input_message,port):
            try: 
              sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
              
              message = input_message.encode() # Encode string to byte
              sock.sendto(message,(ip,port)) # Sending the json data into the udp  
            except: 
                  print("Connection error via ip: ",str(ip))  
        def Subscriber_dict(self,ip,buffer_size,port): 
            try: 
               exec("sock_"+str(port)+" =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
               address = (ip,port) 
               exec("sock_"+str(port)+".bind(address)") 
               exec("global data; data,addr"+ "= sock_"+str(port)+".recvfrom("+str(buffer_size)+")") # Btting the bit operating 
               received = pickle.loads(data)
               message = json.loads(received)
               exec("print(message,type(message),addr)")
               return message
            except:
                print("Subscriber connection value error at ip: ",ip,port) # Getting the report on the ip and port value 
        def Subscriber_string(self,ip,buffer_size,port): 
            try: 
               exec("sock_"+str(ip)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)") 
               address = (str(ip),port) 
               exec("sock_"+str(port)+".bind(address)")  
               exec("global data; data,addr"+ "= sock_"+str(port)+".recvfrom("+str(buffer_size)+")") # Btting the bit operating 
               message = data.decode()
               exec("print(message,type(message),addr)") 
               return message   
            except: 
                print("connection error via ip: ",str(ip))

#Sensors and actuator algorithm type of category 
class Action_control(object): 
           #GPIO output serial/local_gpio 
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
           # Local GPIO on Machine
           def DC_motor_driver_set(self,number_index,direction,gipoL,gpioR,pwm,boolean):  # DC motor driver with on-board gpio 
                  #This function working on 2 pins input from the gpioL gpioR at the same time from the input
                  exec("Motor_"+str(number_index)+" = PhaseEnableMotor("+str(gipoL)+","+str(gpioR),+"pwm="+boolean+","+"pin_factory=None)")  
                  exec("Motor_"+str(number_index)+"."+direction+"(speed = "+str(pwm))          
           def Stepper_motor_driver(self,GPIOA,GPIOB,GPIOC,GPIOD,g_code):  # Unipolar stepper motor control with stepper_motor             
           
                  pass

           def BLDC_motor_Driver(self,GPIO,pwm):# BLDC motor driver
                   
                  pass
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
           def Serial_mcu_connect(self,component_name,number,Serialdev):
                 print("Serial connected!",component_name) 
                 exec(str(component_name)+"_"+str(number)+" = pyfirmata.ArduinoMega('"+str(Serialdev)+"')") #Choosing the data selection from the sequence conditioning  

           #Serial GPIO 
           def Serial_DC_motordriver(self,node_type,component_name,Serialdev,mcu_number,number,pin_number,motor_name,speed,signal_type,gpiol,gpior): # Getting the motor name and the speed of the motor 
                  # Convertting this into the execution 
                  # using the pins mapping from the stm32F103C8T6 
              if mcu_number == "STM32F103C8TX": # Getting the microcontroller series
                  print("Selected mcu: ",mcu_number) 
                  #hardware = pyfirmata.ArduinoMega(serialdev) # Getting the serial dev input here for example /dev/ttyACM0 , /dev/ttyUSB0 
                  if node_type == "main_node":
                        print("devices_data",node_type,component_name,Serialdev,mcu_number,number,pin_number,motor_name,speed,signal_type,gpiol,gpior)
                        exec(str(component_name)+"_"+str(number)+" = pyfirmata.ArduinoMega('"+str(Serialdev)+"')") #Choosing the data selection from the sequence conditioning  
                  #exec("motordat"+str(number)+" = str(component_name)"+"_"+str(number)+".get_pin('d:"+str(2)+":"+str('p'))
                  exec("motorl_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(pin_number[0])+":"+str(signal_type)+"')")
                  exec("motorr_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(pin_number[1])+":"+str(signal_type)+"')")
                  exec("if gpiol"+" == 1 and gpior"+"== 0:"+"\n\t"+"motorl_"+str(number)+".write("+str(speed)+")"+"\n\t"+"motorr_"+str(number)+".write(0)") 
                  exec("if gpiol"+" == 0 and gpior"+"== 1:"+"\n\t"+"motorl_"+str(number)+".write(0)"+"\n\t"+"motorr_"+str(number)+".write("+str(speed)+")") 
                  exec("if gpiol"+" == 0 and gpior"+"== 0:"+"\n\t"+"motorl_"+str(number)+".write(0)"+"\n\t"+"motorr_"+str(number)+".write(0)") 

              if mcu_number == "STM32F303K8TX":
                  print("Selected mcu: ",mcu_number) 
                  #hardware = pyfirmata.ArduinoMega(serialdev) # Getting the serial dev input here for example /dev/ttyACM0 , /dev/ttyUSB0 
                  if node_type == "main_node":
                        print("devices_data",node_type,component_name,Serialdev,mcu_number,number,pin_number,motor_name,speed,signal_type,gpiol,gpior)
                        exec(str(component_name)+"_"+str(number)+" = pyfirmata.ArduinoMega('"+str(Serialdev)+"')") #Choosing the data selection from the sequence conditioning  
                  #exec("motordat"+str(number)+" = str(component_name)"+"_"+str(number)+".get_pin('d:"+str(2)+":"+str('p'))
                  exec("motorl_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(pin_number[0])+":"+str(signal_type)+"')")
                  exec("motorr_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(pin_number[1])+":"+str(signal_type)+"')")
                  exec("if gpiol"+" == 1 and gpior"+"== 0:"+"\n\t"+"motorl_"+str(number)+".write("+str(speed)+")"+"\n\t"+"motorr_"+str(number)+".write(0)") 
                  exec("if gpiol"+" == 0 and gpior"+"== 1:"+"\n\t"+"motorl_"+str(number)+".write(0))"+"\n\t"+"motorr_"+str(number)+".write("+str(speed)+")") 
                  exec("if gpiol"+" == 0 and gpior"+"== 0:"+"\n\t"+"motorl_"+str(number)+".write(0)"+"\n\t"+"motorr_"+str(number)+".write(0)") 
                  
           def Serial_stepper_driver(self,serialdev,g_code): # Getting the stepper motor board name to classify the board 
                                     
                  pass 

           def Serial_BLDC_motor_Driver(self,GPIO,pwm): 
                   
                  pass

           def Serial_Servo_motor(self,number,motor_type,angle): #Build the servo motor from scratch using the control theory algorithm to calibrate the angle of the servo motor 

                   pass 
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

           #I2C GPIO 
           def I2C_DC_motordriver(self,i2c_address,gpiol,gpior):
 
                  pass 

           def I2C_servo_motor(self,name_servo,angle,pins):
                try:  
                   
                    exec(str(name_servo)+"_servo = "+"servo.Servo(pca.channels["+str(pins)+"])") # Getting the pins number of the servo 
                    exec(str(name_servo)+"_servo.angle = "+str(angle)) # Getting the angle of the servo to activate the function of the motion system
                except:
                     print("PCA9685 I2C connection error please check your device connection ") 
           def I2C_stepper_driver(self,i2c_address,g_code): 

                  pass 

           def I2C_BLDC_motor_Driver(self,GPIO,pwm): 

                  pass  

           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
class Visual_Cam_optic(object):  # Calling the camera and optic devices input for image processing publish and subscriber on one platform to be easy to movidy with one library 
           #Camera data in the raw input      
           def Camera_raw(self,cam_num,Buffers,portdata,ip_number): # Getting the raw image of the camera
                # Running the full function of the for loop capability to publish the data from this function individually   
                exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                exec("server_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                exec("server_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")                   
                exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)
                exec("print(host_ip_"+str(cam_num)+")")
                exec("port = "+str(portdata)) # Getting the portdata from the list to getting the camera data
                exec("socket_address_"+str(cam_num)+" = (host_ip_"+str(cam_num)+","+str(portdata)+")")
                exec("server_socket_"+str(cam_num)+".bind(socket_address_"+str(cam_num)+")")
                exec("print('Listening at_camnum"+str(cam_num)+":',socket_address_"+str(cam_num)+")") # Getting the raw camera data as a publisher to publish the camera data 
                exec("vid_"+str(cam_num)+" = cv2.VideoCapture("+str(cam_num)+")") #  replace 'rocket.mp4' with 0 for webcam
                exec("fps_"+str(cam_num)+","+"st_"+str(cam_num)+","+"frames_to_count_"+str(cam_num)+","+"cnt_"+str(cam_num)+" = (0,0,20,0)")
                exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tmsg_"+str(cam_num)+","+"client_addr_" +str(cam_num)+" = server_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num) +")"+"\n\t\tprint('GOT connection from',client_addr_"+str(cam_num)+")"+"\n\t\tWIDTH=400"+"\n\t\twhile(vid_"+str(cam_num)+".isOpened()):"+"\n\t\t\t_"+str(cam_num)+",frame_"+str(cam_num)+" = vid_"+str(cam_num)+".read()\n\t\t\tframe_"+str(cam_num)+" = imutils.resize(frame_"+str(cam_num)+",width=WIDTH)\n\t\t\tencoded_"+str(cam_num)+","+"buffer_"+str(cam_num)+" = cv2.imencode('.jpg',frame_"+str(cam_num)+",[cv2.IMWRITE_JPEG_QUALITY,80])\n\t\t\tmessage_"+str(cam_num)+" = base64.b64encode(buffer_"+str(cam_num)+")"+"\n\t\t\tserver_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",client_addr_"+str(cam_num)+")"+"\n\t\t\tframe_"+str(cam_num)+" = cv2.putText(frame_"+str(cam_num)+",'FPS: '+str(fps_"+str(cam_num)+"),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)") # Getting the frame from the video input 
           
           def Camera_subscriber(self,cam_num,Buffers,portdata,ip_host): # Getting the host ip data 
                  #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_host)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 
           #Using to manage the muti perpost camera from the single frame input from the camera to avoid the speed problem 
           def Multifunctional_camera(self): 
                     pass 

           def Camera_QR(self,cam_num,Buffers,portdata,port_message,ip_number):  # Getting the raw camera image 
                  #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  exec("global x")
                  exec("global y") 
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")"+"\n\t\tif _"+str(cam_num)+":"+"\n\t\t\tImage_"+str(cam_num)+" = cv2.cvtColor(frame_"+str(cam_num)+", cv2.COLOR_BGR2RGB)"+"\n\t\t\ttry:"+"\n\t\t\t\tbarcodes_"+str(cam_num)+" = pyzbar.decode(Image_"+str(cam_num)+")"+"\n\t\t\t\tprint(barcodes_"+str(cam_num)+")"+"\n\t\t\t\tfor barcode_"+str(cam_num)+" in barcodes_"+str(cam_num)+":"+"\n\t\t\t\t\t(x, y, w, h) = barcode_"+str(cam_num)+".rect"+"\n\t\t\t\t\tcv2.rectangle(Image_"+str(cam_num)+", (x, y), (x + w, y + h), (0, 0, 255), 2)"+"\n\t\t\t\t\tbarcodeData_"+str(cam_num)+" = barcode_"+str(cam_num)+".data.decode('utf-8')"+"\n\t\t\t\t\tbarcodeType_"+str(cam_num)+" = barcode_"+str(cam_num)+".type"+"\n\t\t\t\t\ttext_"+str(cam_num)+"= '{} {}'.format(barcodeData_"+str(cam_num)+", barcodeType_"+str(cam_num)+")"+"\n\t\t\t\tprint('Text reading from qr code',text_"+str(cam_num)+")"+"\n\t\t\t\tprint('Coordinate ',text_"+str(cam_num)+",x,y)"+"\n\t\t\t\tglobal message;message_data = {'Message':text_"+str(cam_num)+",'X':x,'Y':y}"+"\n\t\t\t\tprint('from message ',message_data)"+"\n\t\t\t\tQR_"+str(cam_num)+" = Internal_Publish_subscriber()"+"\n\t\t\t\tQR_"+str(cam_num)+".Publisher_dict('"+str(ip_number)+"',message_data,"+str(port_message)+")"+"\n\t\t\texcept:"+"\n\t\t\t\tprint('No QRcode detected!')") # Getting the return of the frame from the loop and 
                  
                  

           def Camera_OCR(self,cam_num,Buffers,portdata,port_message,ip_number):
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 
                  
                  

           def Camera_yolo(self,cam_num,Buffers,portdata,port_message,ip_number): # Getting the raw yolo image 

                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 

           def Camera_Face_recognition(self,cam_num,Buffers,portdata,port_message,ip_number):
 
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 

           def Camera_Visual_to_text(self,cam_num,Buffers,portdata,port_message,ip_number): 
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 

class Audio_function(object):
          
           def Speech_recognition(self,initial_lang,destination_lang,addresses,port): # Getting the prameter for language processing on the speech recognition 
                  lang =  initial_lang
                  lang2 = destination_lang
                  #Activate_word = ["translation","Translation","mode","translate","Translate"] #Activate translate mode concern word need more vocabulary 
                  #Direction_translate = ["to","in to"]
                  def callback(recognizer, audio):
                           # received audio data, now we'll recognize it using Google Speech Recognition
                           sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                           address = addresses 
                           try:
                   
                               print("Speech Recognition thinks you said " + recognizer.recognize_google(audio,language = lang)) # Setting the default language from the json file 
                               send_speech_data = {recognizer.recognize_google(audio,language =lang):lang}  
                               jsondata = json.dumps(send_speech_data) 
                               message = pickle.dumps(jsondata) 
                               sock.sendto(message,(address,port))      
                           except sr.UnknownValueError:
                                    print("Speech Recognition could not understand audio")
                  r = sr.Recognizer()
                  m = sr.Microphone()
                  with m as source:
                       r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening
                  stop_listening = r.listen_in_background(m, callback)
                  # `stop_listening` is now a function that, when called, stops background listening

                  # do some unrelated computations for 5 seconds
                  for i in count(0):time.sleep(0.2)
           def Text_to_speech(self,text,destination_lang,speed_speak,loudness):
                  sox_effects = ('speed',speed_speak)
                  lang = destination_lang
                  os.system("amixer -D pulse sset Master "+str(loudness)+"%")
                  speech  = Speech(text,lang)
                  speech.play(sox_effects) 
                    
class Odometry_function(object):
           
           def Odometry_algorithm(self,module,i2c): # Getting the x,y,theta angle  and i2c addres that may come from i2c expander  
                  
                mpu = adafruit_mpu6050.MPU6050(i2c)  # Getting the mpu6050 function  6-dof gyoscope 
                icm = adafruit_icm20x.ICM20948(i2c)  # Getting the ICM20948 function 9-dof gyroscope 
                for w in count(0):
                      if module == "ICM20948":
                           icm = adafruit_icm20x.ICM20948(i2c)
                           print(math.degrees(icm.gyro[0]),math.degrees(icm.gyro[1]),math.degrees(icm.gyro[2]))
                           return math.degrees(icm.gyro[0]),math.degrees(icm.gyro[1]),math.degrees(icm.gyro[2])
                      if module == "MPU6050":
                           print(math.degrees(mpu.gyro[2]),math.degrees(mpu.gyro[1]),math.degrees(mpu.gyro[0]))
                           return math.degrees(mpu.gyro[2]),math.degrees(mpu.gyro[1]),math.degrees(mpu.gyro[0])  # Getting output angle from the IMU sensor data in x,y,z    
                               
class Gyroscope_function(object):

          def Gyro_sensor_module(self,module,i2c,ip,port):
                if module == "MPU6050":
                   mpu = adafruit_mpu6050.MPU6050(i2c)  # Getting the mpu6050 function  6-dof gyoscope 
                if module == "ICM20948":
                   icm = adafruit_icm20x.ICM20948(i2c)  # Getting the ICM20948 function 9-dof gyroscope 

                for w in count(0):
                      if module == "ICM20948":
                           icm = adafruit_icm20x.ICM20948(i2c)
                           print(math.degrees(icm.gyro[0]),math.degrees(icm.gyro[1]),math.degrees(icm.gyro[2]))
                           #return math.degrees(icm.gyro[0]),math.degrees(icm.gyro[1]),math.degrees(icm.gyro[2])
                           #Using udp to sending the data over the message of the sensor for faster transfer 
                           message_data = {'X':math.degrees(icm.gyro[2]),"Y":math.degrees(icm.gyro[1]),"Z":math.degrees(icm.gyro[0])}
                           icx_sensor = Internal_Publish_subscriber() 
                           icx_sensor.Publisher_dict(ip,message_data,port) 
                      if module == "MPU6050":
                           print(math.degrees(mpu.gyro[2]),math.degrees(mpu.gyro[1]),math.degrees(mpu.gyro[0]))
                           #return math.degrees(mpu.gyro[2]),math.degrees(mpu.gyro[1]),math.degrees(mpu.gyro[0])  # Getting output angle from the IMU sensor data in x,y,z     
                           #Using udp to sending the data over the message of the sensor for faster transfer 
                           message_data = {'X':math.degrees(mpu.gyro[2]),"Y":math.degrees(mpu.gyro[1]),"Z":math.degrees(mpu.gyro[0])}
                           mpu_sensor = Internal_Publish_subscriber() 
                           mpu_sensor.Publisher_dict(ip,message_data,port)
class Encoder_function(object):

        def Magnetic_encoder(self,module,i2c):

               pass 

class Cellular_networking_com(object): # Getting the location outdoor from the cellular module in lattitude and longitude in realtime 
           def Call_out(self,serialdev,command_input):
                
                pass               
           def Location_cellular_network(self):

                pass 
              
class Navigation_sensors(object): 
           def Lidar_nav(self):
                pass  

           def GPS_module_nav(self,gpsname,serialdev,baudrate):
                  ser = serial.Serial(serialdev,baudrate) # Set the comport and baudrate  
                  x = str(ser.read(1200))
                  pos1 = x.find("$GPRMC")
                  pos2 = x.find("\n",pos1)
                  compass = x.find("$GPGLL")
                  loc = x[pos1:pos2]
                  data = loc.split(',')
                  geolocator = Nominatim(user_agent=gpsname)
                  if data[2] == 'V':
                        print('No location found')
                  for ty in count(0):
                       print(ser.readline()) 
                       try:
                           print("Latitude =" + str(float(data[117])/100))
                           print("Longtitude = " + str(float(data[119])/100))
                           location = geolocator.reverse(str(float(data[117])/100)+","+ str(float(data[119])/100)) 
                           print(location.address)
                           global output_address;output_address = location.address
                       except: 
                            print("GPS is not connected")
                           

                  gps_dat = {"Latitude":str(float(data[3])/100),"Longitude": str(float(data[5])/100),"Adress":output_address}
                  return gps_dat
           def camera_slam_nav(self):
                pass  
          
           def rssi_distance_converter(self):
                     
                    bssid_list = subprocess.check_output("iwlist scanning",shell=True)
                    data_raw_decode = bssid_list.decode() # decode the raw data 
                    #Finding the Signal strange from the list index
                    for i in data_raw_decode.split(":"): 
                          try:

                              if len(i.split("=")) == 3:

                                     rssi = int(i.split("=")[2].split("\n")[0].split(" ")[0]) 
                                     Pl = int(i.split("=")[1].split(" ")[0].split("/")[0]) 
                                     Pt = int(i.split("=")[1].split(" ")[0].split("/")[1])
                                     A = Pt-Pl 
                                     c_dat = (A-rssi)*0.014336239
                                     global distance;distance = math.pow(10,c_dat) 
                                     print("Output distance ",Pt,Pl,rssi," dbm",distance-2.55," m") 
                              return distance-2.55   
                          except: 
                               pass  


# This can be using with the multiple array of chemical sensor,Force sensor,Pressure sensor,
class Tactile_sensor(object):
          # Input sensor list with tuple
          # This array sensor can be streaming over multicache server and the frame data streaming will be process by the other image processing algorithm 
          def Numpy_array_image_frame(self,sensor_list_input,title_name,vis_output,size_video): # Getting the image frame from the numpy array sensor
              for tr in count(0):   
                  sensor_list = list(sensor_list_input) # Getting the sensor list input
                  array = np.array(sensor_list) 
                  array = array.astype(np.uint8) 
                  shape_data = array.shape
                  ex_shapecal = int(shape_data[0]) # using this shape of sensor input 
                  sqrt_state = math.sqrt(ex_shapecal)
                  check_int = isinstance(sqrt_state ,int)
                  if check_int == True: 
                         array = array.astype(np.uint8)       
                         array = np.reshape(array,(int(sqrt_state),int(sqrt_state))) 
                         color_image  = cv2.cvtColor(array, cv2.COLOR_GRAY2RGB)*255
                         if vis_output == 1:
                                im2 = cv2.resize(color_image,(size_video[0],size_video[1]))
                                cv2.imshow('"'+str(title_name)+'"',im2)
                                if cv2.waitKey(1) & 0xFF == ord(' '):
                                      break 
                  if check_int == False: 
                         array = array.astype(np.uint8)
                         # Finding the factors before add into the reshape by using the factor to calculate the right number
                         #Checking the vertical and herizontal state 
                         upper = math.ceil(math.sqrt(ex_shapecal))+1 
                         lower = math.floor(math.sqrt(ex_shapecal)) 
                         array = np.reshape(array,(upper,lower)) 
                         color_image  = cv2.cvtColor(array, cv2.COLOR_GRAY2RGB)*255 
                         if vis_output == 1:
                                im2 = cv2.resize(color_image,(size_video[0],size_video[1]))
                                cv2.imshow('"'+str(title_name)+'"',im2)
                                if cv2.waitKey(1) & 0xFF == ord(' '):
                                      break                     
                  return color_image  # Output image frame                               

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#a = Internal_Publish_subscriber() 
#a.Publisher_dict("127.0.0.1",{"input":9048},5080) # Sending the data type dicationary now thinking about getting the value input from json file and convert data into the specific data type 

#Getting the input from the hiroku json data component selector to classify the type of the data in the list and getting the new code to generate 
data_ex = {"actuator_1":3,"microcontroller_1":[5,7],"gpio_1":8} # list json to classify and get the message from each devices connected with it
#Check the sensor and the devices hardware connection
#Getting the data input to the pub with the key and value of dict representing the data in the topic and message data type 

def Create_node_pub(topic,message,addresses,initial_port): # Create the node public from the json file input from component selection and define function of the code in the node to control 
       #Create the module of the node component input into the function                
       #Before running the loop of publisher check that the module node is created
       port = initial_port
       address = addresses
       exec("pub_"+str(port)+ "= Internal_Publish_subscriber()")  
       exec("pub_"+str(port)+".Publisher_dict('"+str(address)+"',{"+"'"+str(topic)+"'"+":"+str(message)+"},"+str(port)+")") # Sending the data type dicationary now thinking about getting the value input from json file and convert data into the specific data type 
       
#Create the sensor receiver as the subscriber for each sensor parameter input from the json component input but this function going to define by json type code generator 
def Create_node_sub(t,addresses,buffer,initial_port):
      
           port = initial_port 
           address = addresses        
           exec("sub_"+str(t)+" = Internal_Publish_subscriber()")  
           exec("global data_return;data_return"+" = sub_"+str(t)+".Subscriber_dict('"+str(address)+"',"+str(buffer)+","+str(port)+")")
           return  data_return 

def Speech_recognition(initial_lang,destination_lang,address,port):
       speech_recog = Audio_function() 
       speech_recog.Speech_recognition(initial_lang,destination_lang,address,port)

def Speaking_languages(text,destination_lang,speed,loudness): 
           audio_speech = Audio_function() 
           audio_speech.Text_to_speech(text,destination_lang,speed,loudness)

def Sensor_array_input(sensor_name,number,sensor_list_input,vis_output,size_video):

        exec(str(sensor_name)+"_"+str(number)+" = Tactile_sensor()")
        Sensor_name_data = str(sensor_name)+"_"+str(number)
        title_name = "'"+str(Sensor_name_data)+"'"
        exec("global data_return;data_return = "+str(sensor_name)+"_"+str(number)+".Numpy_array_image_frame("+str(sensor_list_input)+","+str(title_name)+","+str(vis_output)+","+str(size_video)+")")
        return data_return           
def Gyroscope_sensor(module,ip,port):  # Getting i2c address 
             gyro_pos_1 = Gyroscope_function()
             gyro_pos_1.Gyro_sensor_module(module,i2c_bus,ip,port)# Getting i2c address and name of module
             
# Checking if serial is main_node or node 
def Create_serial_motor(node_type,component_name,Serialdev,mcu_number,number,pin_number,motor_name,speed,signal_type,gpiol,gpior):
        #node_type,component_name,Serialdev,mcu_number,number,pin_number,motor_name,speed,signal_type,gpiol,gpior
        sensor1 = Action_control()    # main_node motor /dev/ttyUSB0 STM32F103C8TX 1 [2, 3] motor_1 1 p 1 0 
        sensor1.Serial_DC_motordriver(node_type,component_name,Serialdev,mcu_number,number,pin_number,motor_name,speed,signal_type,gpiol,gpior)
        
def Create_i2c_Servo(servo_num,servo_name,angle,pin):
     try: 
        # define the name angle and pin of the servo to connect into the board 
        exec("servo_"+str(servo_num)+" = Action_control()") 
        exec("servo_"+str(servo_num)+".I2C_servo_motor('"+str(servo_name)+"',"+str(angle)+","+str(pin)+")")
     except: 
        print("Checking your variables and pins data input")      
def microcontroller_info_dat(mcu_code_name): 
            exec("mcu_"+str(mcu_code_name)+" = Microcontroller_pins()") 
            exec("global data_mcu; data_mcu" +" = mcu_"+str(mcu_code_name)+".request_mcu('"+str(mcu_code_name)+"')") # Getting the request mcu data 
            return data_mcu 

# Non execution pub node 
def Camera_pub_node(cam_num,buffers,port,ip_number): # running all theses in exec 
         
       exec("cam_"+str(cam_num)+" = Visual_Cam_optic()")
       exec("cam_"+str(cam_num)+".Camera_raw("+str(cam_num)+","+str(buffers)+","+str(port)+",'"+str(ip_number)+"')")

# Non execution sub node 
def Camera_QR_sub_node(cam_num,buffers,port,port_message,ip_number): # Input the function into the the command all list of computer vision are { Camera_raw , Camera_QR, Camera_OCR, Camera_yolo, Camera_face_recognition, Camera_Visual_to_text}
  
       exec("cam_"+str(cam_num)+" = Visual_Cam_optic()")
       exec("cam_"+str(cam_num)+".Camera_QR("+str(cam_num)+","+str(buffers)+","+str(port)+","+str(port_message)+",'"+str(ip_number)+"')")

def Camera_OCR_sub_node(cam_num,buffers,port,port_message,ip_number):
       exec("cam_"+str(cam_num)+" = Visual_Cam_optic()")
       exec("cam_"+str(cam_num)+".Camera_OCR("+str(cam_num)+","+str(buffers)+","+str(port)+","+str(port_message)+",'"+str(ip_number)+"')")
       

def Camera_face_rec_sub_node(cam_num,buffers,port,port_message,ip_number):
       exec("cam_"+str(cam_num)+" = Visual_Cam_optic()")
       exec("cam_"+str(cam_num)+".Camera_Face_recongnition("+str(cam_num)+","+str(buffers)+","+str(port)+","+str(port_message)+",'"+str(ip_number)+"')")   


def Camera_yolo_sub_node(cam_num,buffers,port,port_message,ip_number): 

        exec("cam_"+str(cam_num)+" = Visual_Cam_optic()")
        exec("cam_"+str(cam_num)+".Camera_yolo("+str(cam_num)+","+str(buffers)+","+str(port)+","+str(port_message)+",'"+str(ip_number)+"')")   

def rssi_indoor():
       rssi_distance = Navigation_sensors() 
       distance = rssi_distance.rssi_distance_converter()
       return distance

def mcu1():
   mcu_data = "STM32F103CBTx"
   mcu_list =  microcontroller_info_dat(mcu_data)
   print(mcu_list)

def mcu2(): 
   mcu_data = "STM32F303K8Tx"
   mcu_list =  microcontroller_info_dat(mcu_data)
   print(mcu_list)

# Configure port at the node generator 
# Before requesting this publisher and the subscriber 
#message={"speed":5,"Angle":90} # Sending message via exec code create more variable to input instance load of message into the server publisher
#def pub1():
#    Create_node_pub("actuators_2",[5,7],"127.0.0.1",5080) # Getting the node created from the json component selection 
#def pub2():
#    Create_node_pub("Servo_1",message,"127.0.0.1",5081)
#def camera_pub():
#    Camera_pub_node(0,65536,9800,'192.168.50.216')
#def camera_sub():
#    Camera_sub_node(0,65536,9801,5020,'192.168.50.192')
#Create_node_pub("Servo_2",message,"192.168.50.192",5020) 
#data_out = Create_node_sub(1,"127.0.0.1",4096,5090) # Getting the local message 
#print(data_out)

#if __name__=="__main__":
#          p1 = threading.Thread(target=pub1)
#          p2 = threading.Thread(target=pub2) 
#          p3 = threading.Thread(target=mcu1) 
#          p4 = threading.Thread(target=camera_sub)
#          p5 = threading.Thread(target=camera_pub)
#          p1.start() 
#          p2.start()
#          #p3.start()
#          p4.start() 
#          p5.start()
#          p1.join()
#          p2.join()
#          #p3.join() 
#          p4.join()
#          p5.join()
          
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#Create the code from json input firmware writer 
#data_ref = {"if_1":{"parameter_1":{"in":["input_parameter","list(data_ex)"]},"for_1":['i',"in range(0,10)"],"print_1":['i']}}  

#for r in range(0,len(data_ref)): 
#       print(list(data_ref)[r],list(data_ref)[r][0])
#       for inner_list in range(0,len(list(data_ref.get(list(data_ref)[r])))):
#                 print(list(data_ref.get(list(data_ref)[r]))[inner_list])
      


#b = Internal_Publish_subscriber() 
#data_return = b.Subscriber_dict("127.0.0.1",4096,5090) 
#print(data_return)

#c = Internal_Publish_subscriber() 
#c.Publisher_dict("127.0.0.1",{"input_1":5048},5040)