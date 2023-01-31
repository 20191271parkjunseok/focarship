import cv2
import numpy as np
import time

class Crosswalk:
    def __init__(self):
        self.cnt = 0
        self.previous_time = time.time() + 5
        self.lower_white = (0, 0, 200)
        # (0, 0, 0)    (0, 0, 212)      (0, 0, 168)
        self.upper_white = (0, 0, 255)
        # (0, 0, 255)  (131, 255, 255)  (172, 111, 255)

    def check_crosswalk(self, img):
        if time.time() < self.previous_time + 10:
            return False

        img = img[240:] #240
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, self.lower_white, self.upper_white)

        # # IMSHOW FOR DEBUG
        # # print(np.count_nonzero(img_mask))
        # cv2.putText(img_mask, 'NONZERO %d'%np.count_nonzero(img_mask), (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        # cv2.imshow('img_mask', img_mask)

        if np.count_nonzero(img_mask) > 2500: # 숫자 변경,, 뭘로? - 현재 확인 불가
            self.on_detected()
            return True

        return False

    def on_detected(self):
        ## 3초 정지 (후 다시 출발 - 가능?)
        car_run_speed = 0
        # time.sleep(3)
        print('CROSSWALK DETECTED')
        self.previous_time = time.time()
        self.cnt += 1

if __name__ == '__main__':
    crosswalk = Crosswalk()

    cap = cv2.VideoCapture('video/original.avi')
    while cap.isOpened():
        ret, frame = cap.read()
        crosswalk.check_crosswalk(frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break