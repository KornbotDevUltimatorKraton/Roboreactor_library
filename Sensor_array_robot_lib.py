from roboreactmaster import Camera_pub_node,Camera_QR_sub_node,Create_node_sub,Create_i2c_Servo,Sensor_array_input 
import pyfirmata 
from itertools import count
try:
  board = pyfirmata.ArduinoMega("/dev/ttyUSB0")
except:
    try:
      board = pyfirmata.ArduinoMega("/dev/ttyUSB1")
    except:
        print("Dedevice connected")
it = pyfirmata.util.Iterator(board)
it.start()

board.analog[0].enable_reporting()
board.analog[1].enable_reporting() 
board.analog[2].enable_reporting() 
board.analog[3].enable_reporting() 
board.analog[4].enable_reporting() 
board.analog[5].enable_reporting() 
board.analog[6].enable_reporting()
board.analog[7].enable_reporting()
conv_units = 10000.0
for r in count(0):
    a0 = int(float(board.analog[0].read() or 0)*622.558)
    a1 = int(float(board.analog[1].read() or 0)*622.558) 
    a2 = int(float(board.analog[2].read() or 0)*622.558) 
    a3 = int(float(board.analog[3].read() or 0)*622.558) 
    a4 = int(float(board.analog[4].read() or 0)*622.558)
    a5 = int(float(board.analog[5].read() or 0)*622.558)
    a6 = int(float(board.analog[6].read() or 0)*622.558)
    a7 = int(float(board.analog[7].read() or 0)*622.558)
    list_array = (a0,a1,a2,a3,a4,a5,a6,a7)
    static_array = (255,0,34,67,78,240,255,255)
    data_out = Sensor_array_input("force_sensor",1,list_array,0,[320,280]) 
    data_out2 = Sensor_array_input("Chemo_sensor",1,static_array,0,[320,280]) 
    print(data_out,data_out2)
