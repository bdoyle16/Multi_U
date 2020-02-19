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
        if distance_f >= 100 :
                loc = f
                end
                
        sensor.dist_sendor(distance)
        
        sensor.r.sleep()
        
except (KeyboardInterrupt, SystemExit):
    gpio.cleanup()
    sys.exit(0)
except:
    gpio.cleanup()
