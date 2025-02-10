
import time
import os
import cv2
import sys
#add current output in pathonpath
sys.path.append(os.path.join(os.getcwd(),"outputs"))
import vlkpy

url = "rtsp://192.168.2.119:554"

iRet = vlkpy.VLK_init();
print('iRet = ', iRet)
time.sleep(1)


print('turn to home ...  0')
vlkpy.turnToHome()
time.sleep(2)


print('turn to left ...  3')
vlkpy.VLK_TurnTo(-100, 0)
time.sleep(1)


# /** @brief Control Gimbal yaw and pitch
#  *  @param sHorizontalSpeed the speed of changing yaw (0.01 degrees/s), 
#  *                          for example, 2000 means 20 degrees per second
#  *  @param sVeritcalSpeed the speed of changing pitch (0.01 degrees/s)
#  *  @details considering a small angle adjustment will result in a huge visual field 
#  *              changing in sky view, we limit speed in a proper range: \n
#  *           -VLK_MAX_YAW_SPEED <= sHorizontalSpeed <= VLK_MAX_YAW_SPEED \n
#  *           -VLK_MAX_PITCH_SPEED <= sVeritcalSpeed <= VLK_MAX_PITCH_SPEED \n
#  *           some example: \n
#  *           move up: VLK_Move(0, 1000); 
#  *           move left: VLK_Move(-1000, 0); 
#  *           move right: VLK_Move(1000, 0); 
#  *           move down: VLK_Move(0, -1000);
#  */
# print('turn to VLKmove ...  4')
# vlkpy.VLK_move(-1000, 0)
# time.sleep(3)
# vlkpy.VLK_Stop()



# print('turn to VLKmove ...  5')
# vlkpy.VLK_move(1000, 0)
# time.sleep(3)
# vlkpy.VLK_Stop()



cap = cv2.VideoCapture(url)

while(cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q') :
        break

cap.release()
# cv2.destoryAllWindows()