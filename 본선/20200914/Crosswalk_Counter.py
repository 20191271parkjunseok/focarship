import cv2
import numpy as np
import time

class Crosswalk_Counter:
    def __init__(self):
        self.cnt = 0
        self.line_num=0

    def draw_lines(self, img, lines, color=[0, 0, 255], thickness=4):
        # sig_xy=0;
        # sig_x=0;
        # sig_y=0;
        # sig_xx=0;
        # count=0;

        for line in lines:
                for x1,y1,x2,y2 in line:
                    resultx1=x1
                    resultx2=x2
                self.line_num+=1

        return cv2.line(img, (resultx1,0), (resultx2,480), color, thickness)


    def hough_transform(self, img, rect, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        #print(lines.size())

        if (lines is None or len(lines) == 0):
          return rect

        self.draw_lines(rect, lines)
        return rect

    def check_crosswalk(self, img):
        img=self.hough_transform(img, img, 1, 1 * np.pi/180, 30, 10, 20)
        if self.line_num>=9:
            self.cw_on_detected()
        print('line_num:',self.line_num)

    def cw_on_detected(self):
        print('CROSSWALK LINE DETECTED!!!')
        # self.previous_time = time.time()
        self.cnt += 1

if __name__ == '__main__':
    crosswalk_counter = Crosswalk_Counter()

    cap = cv2.VideoCapture('video/original.avi')
    while cap.isOpened():
        ret, frame = cap.read()
        crosswalk_counter.check_crosswalk(frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
