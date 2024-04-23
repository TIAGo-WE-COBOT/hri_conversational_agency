#!/usr/bin/env python3

import time
import rospy

class MyTimer:
    def __init__(self):
        self.start_time = time.perf_counter()
        self.beg_time = time.perf_counter()
        #print(self.beg_time)
        self.sec_flag = 0
        self.time_flag = 0
        self.sec = 0
        self.init_timer_flag = True
    
    def timer_cb(self, event):
        self.init_timer_flag = False
        print("Initialization completed")

    def fixed_timer(self, duration):
        rospy.Timer(rospy.Duration(duration), self.timer_cb, oneshot=True) #wait for n seconds

    def elapsed_fixed_time(self, th):
        while not self.time_flag:
            while not self.sec_flag:
                self.end_time = time.perf_counter()
                if self.end_time - self.start_time > 1:
                    self.sec = self.sec + 1
                    print(self.sec)
                    self.sec_flag = 1
                    self.start_time = time.perf_counter()
                    if self.sec == th: #elapsed time we want to measure in seconds
                        self.time_flag = 1
            self.sec_flag = 0
        self.time_flag = 0

    def elapsed_time(self):
        self.final_time = time.perf_counter()
        elapsed_time = self.final_time - self.beg_time
        return elapsed_time

if __name__ == "__main__":
    rospy.init_node('timer')
    t = MyTimer()

    rate = rospy.Rate(10)
    try:
        t.elapsed_fixed_time()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
