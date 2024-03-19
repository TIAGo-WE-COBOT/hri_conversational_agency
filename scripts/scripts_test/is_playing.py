#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
import rospy

class CHECK_PLAY():
    def __init__(self):
        self.is_playing_pub = rospy.Publisher("/isplaying", String, queue_size = 1)
        self.is_playing_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.is_playing_cb)
        self.is_playing = False

    def is_playing_cb(self, data):
        if(data.status[0].values[0].key == "Active sounds" and data.status[0].values[0].value == "1"):
            self.is_playing = True
            self.is_playing_pub.publish(self.is_playing)
        else:
            self.is_playing = False
            self.is_playing_pub.publish(self.is_playing)
    

    # def is_playing_cb(self, data):
    #     for status in data.status:
    #         if status.name.endswith(": Node State"):
    #             for item in status.values:
    #                 if item.key == "Active sounds" and int(item.value) == 1:
    #                     print("TRUE")
                    # else:
                    #     print("FALSE")

    #def listen(self):
    #    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('check_play')
    check_play = CHECK_PLAY()
    rate = rospy.Rate(1)
    rospy.spin()
