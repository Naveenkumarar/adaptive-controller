import cv2

from qcar.q_essential import Camera2D

            
cap = Camera2D(camera_id="3", frame_width=320, frame_height=240, frame_rate=120)

while True:
    
    cap.read()
    frame = cap.image_data.copy()

    cv2.imshow("RTSP View", frame)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
