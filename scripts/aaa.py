#!/usr/bin/env python3

import cv2
import numpy as np
import time
###DA IMPORTARE SU MEDIAPLAYER

path_to_video = "/home/marconasato/VEX.mp4"

# def play_video(ptv):
    
#     screen_name = "Video_Player"

#     cap = cv2.VideoCapture(ptv)

#     cv2.namedWindow(screen_name, cv2.WND_PROP_FULLSCREEN)
#     cv2.resizeWindow(screen_name, 1920, 1080)

#     while(cap.isOpened()):
#         ret, frame = cap.read()
#         if ret == True: 
#         # Display the resulting frame 
#             cv2.imshow(screen_name, frame) 
#         # Press Q on keyboard to exit 
#             if cv2.waitKey(25) & 0xFF == ord('q'): 
#                 break
    
#     # Break the loop 
#         else: 
#             break
#     cap.release()   
#     # Closes all the frames 
#     cv2.destroyAllWindows()
#     time.sleep(5)
#     print("finito") 

# play_video(path_to_video)

from playsound import playsound

path_to_audio = "/home/marconasato/SOUEX.wav"

def riproduci_audio(file_audio):
    try:
        # Riproduci il file audio
        playsound(file_audio)
        print("finito")
    except Exception as e:
        print("Errore durante la riproduzione dell'audio:", str(e))

# Chiamata alla funzione per riprodurre l'audio
riproduci_audio(path_to_audio)
