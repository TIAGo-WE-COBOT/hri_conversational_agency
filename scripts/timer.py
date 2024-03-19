#!/usr/bin/env python

import time

class TIMER:
    def __init__(self):
        self.start_time = time.perf_counter()
        self.beg_time = time.perf_counter()
        print(self.beg_time)
        self.flag = 0
        self.g_flag = 0
        self.sec = 0

    # def elapsed_time_1(self):
    #     while not self.g_flag:
    #         while not self.flag:
    #             self.end_time = time.perf_counter()
    #             if self.end_time - self.start_time > 1:
    #                 self.sec = self.sec + 1
    #                 print(self.sec)
    #                 self.flag = 1
    #                 self.start_time = time.perf_counter()
    #         self.flag = 0

    def elapsed_time(self):
        self.final_time = time.perf_counter()
        elapsed_time = self.final_time - self.beg_time
        return elapsed_time
    