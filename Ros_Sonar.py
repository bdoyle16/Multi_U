#!/usr/bin/env python

import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import String

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class sonar():
    def __init__(self):
        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar_loc',String, queue_size=1)
        self.r = rospy.Rate(15)
    def dist_sendor(self,dist):
        data = String()
        data.data=dist
        self.distance_publisher.publish(data)
        
        
gpio.setmode(gpio.BCM)
trig_f = 4 
echo_f = 17 
trig_l = ?
echo_l = ?
trig_r = ?
echo_r = ?
trig_d = ?

gpio.setup(trig_f, gpio.OUT)
gpio.setup(echo_f, gpio.IN)
gpio.setup(trig_l, gpio.OUT)
gpio.setup(echo_l, gpio.IN)
gpio.setup(trig_r, gpio.OUT)
gpio.setup(echo_r, gpio.IN)
gpio.setup(trig_d, gpio.OUT)
gpio.setup(echo_d, gpio.IN)

sensor=sonar()
time.sleep(0.5)
print ('-----------------------------------------------------------------sonar start')
try :
    while True :
        #print('running')
        gpio.output(trig_f, False)
        gpio.output(trig_l, False)
        gpio.output(trig_r, False)
        gpio.output(trig_d, False)
        time.sleep(0.1)
        gpio.output(trig_f, True)
        gpio.output(trig_l, True)
        gpio.output(trig_r, True)
        gpio.output(trig_d, True)
        time.sleep(0.00001)
        gpio.output(trig_f, False)
        gpio.output(trig_l, False)
        gpio.output(trig_r, False)
        gpio.output(trig_d, False)
        while gpio.input(echo_f) == 0 :
            pulse_start_f = time.time()
            #print('no')
        while gpio.input(echo_f) == 1 :
            pulse_end_f = time.time()
            #print('yes')
        pulse_duration_f = pulse_end_f - pulse_start_f
        distance_f = pulse_duration_f * 17000
            continue
        distance_f = round(distance_f, 3)
        while gpio.input(echo_l) == 0 :
            pulse_start_l = time.time()
            #print('no')
        while gpio.input(echo_l) == 1 :
            pulse_end_l = time.time()
            #print('yes')
        pulse_duration_l = pulse_end_l - pulse_start_l
        distance_l = pulse_duration_l * 17000
            continue
        distance_l = round(distance_l, 3)
        while gpio.input(echo_r) == 0 :
            pulse_start_r = time.time()
            #print('no')
        while gpio.input(echo_r) == 1 :
            pulse_end_r = time.time()
            #print('yes')
        pulse_duration_r = pulse_end_r - pulse_start_r
        distance_r = pulse_duration_r * 17000
            continue
        distance_r = round(distance_r, 3)
        while gpio.input(echo_d) == 0 :
            pulse_start_d = time.time()
            #print('no')
        while gpio.input(echo_d) == 1 :
            pulse_end_d = time.time()
            #print('yes')
        pulse_duration_d = pulse_end_d - pulse_start_d
        distance_d = pulse_duration_d * 17000
            continue
        distance_d = round(distance_d, 3)
        if distance_f >= 100 not distance_l >= 100 not distance_r >= 100 not distance_d >= 100 :
                loc = "f"
                continue
        elif distance_l >= 100 not distance_f >= 100 not distance_r >= 100 not distance_d >= 100:
                loc = "l"
                continue
        elif distance_r >= 100 not distance_f >= 100 not distance_l >= 100 not distance_d >= 100:
                loc = "r"
                continue
        elif distance_d >= 100 not distance_f >= 100 not distance_l >= 100 not distance_r >= 100:
                loc = "d"
                continue
        elif distance_f >= 100 and disntance_l >= 100 not distance_r >= 100 not distance_d >= 100 :
                loc = "fl"
                continue
        elif distance_f >= 100 and distance_r >= 100 not distance_l >= 100 not distance_d >= 100 :
                loc = "fr"
                continue
        elif distance_f >= 100 and distance_d >= 100 not distance_l >= 100 not distance_r >= 100 :
                loc = "fd"
                continue
        elif distance_l >= 100 and distance_r >= 100 not distance_f >= 100 not distance_d >= 100 :
                loc = "lr"
                continue
        elif distance_l >= 100 and distance_d >= 100 not distance_f >= 100 not distance_r >= 100 :
                loc = "ld"
                continue
        elif distance_r >= 100 and distance_d >= 100 not distance_f >= 100 not distance_l >= 100 :
                loc = "rd"
                continue
        elif distance_f >= 100 and distance_l >= 100 and distance_r >=100 not distance_d >= 100:
                loc = "flr"
                continue
        elif distance_f >= 100 and distance_l >= 100 and distance_d >= 100 not distance_r >= 100:
                loc = "fld"
                continue
        elif distance_f >= 100 and distance_r >= 100 and distance_d >= 100 not distance_l >= 100:
                loc = "frd"
                continue
        elif distance_l >= 100 and distance_r >= 100 and distance_d >= 100 not distance_f >= 100:
                loc = "lrd"
                continue
        elif distance_f >= 100 and distance_r >=100 and distance_d >= 100 and distance_l >= 100 :
                loc = "flrd"
                continue
        #print ('Location : %f '%loc)
                
        sensor.dist_sendor(loc)
        
        sensor.r.sleep()
        
except (KeyboardInterrupt, SystemExit):
    gpio.cleanup()
    sys.exit(0)
except:
    gpio.cleanup()
