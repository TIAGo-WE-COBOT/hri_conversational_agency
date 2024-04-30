#!/usr/bin/env python3

from playsound import playsound
import cv2
import rospy
from std_msgs.msg import String

class MediaPlayer:
    def __init__(self):
        self.play_media_sub = rospy.Subscriber('play/media',
                                               String,
                                               self.play_media_cb)
        
        self.end_media_pub = rospy.Publisher('end/media',
                                             String,
                                             queue_size=1)

        self.type_media = ''
        self.media = ''
        self.path_to_video = "/home/marconasato/VEX.mp4"
        self.path_to_audio = "/home/marconasato/SOUEX.wav"

    def play_video(self, ptv):
        screen_name = "Video_Player"
        cap = cv2.VideoCapture(ptv)
        cv2.namedWindow(screen_name, cv2.WND_PROP_FULLSCREEN)
        cv2.resizeWindow(screen_name, 1920, 1080)

        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret == True: 
            # Display the resulting frame 
                cv2.imshow(screen_name, frame) 
            # Press Q on keyboard to exit 
                if cv2.waitKey(25) & 0xFF == ord('q'): 
                    break
        # Break the loop 
            else: 
                break
        cap.release()   
        # Closes all the frames 
        cv2.destroyAllWindows()
        print("finito") 
        rospy.sleep(3) 

    def play_audio(self, pta):
        try:
            # Riproduci il file audio
            playsound(pta)
            print("finito") 
            rospy.sleep(3) 
        except Exception as e:
            print("Errore durante la riproduzione dell'audio:", str(e))

    # Chiamata alla funzione per riprodurre l'audio
    

    def play_media_cb(self, data):
        self.type_media = data.data.split('%')[0]
        self.media = data.data.split('%')[1]
        if(self.type_media == 'M' or self.type_media == 'AL'):
            #print("Ripriduco audio:", self.media)
            self.play_audio(self.path_to_audio)
            self.end_media_flag = String()
            self.end_media_flag.data = "True"
            self.end_media_pub(self.end_media_flag)

        elif(self.type_media == 'V'):
            #print("Ripriduco video:", self.media)
            self.play_video(self.path_to_video)
            self.end_media_flag = String()
            self.end_media_flag.data = "True"
            self.end_media_pub(self.end_media_flag)

if __name__ == "__main__":
    rospy.init_node('media_player')
    mp = MediaPlayer()
    rate = rospy.Rate(10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
            