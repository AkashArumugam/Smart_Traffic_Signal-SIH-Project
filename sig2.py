import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import sys

class activateSignal():

    def __init__(self, sigIndex):
        self.si = sigIndex
        self.titles = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4']
        self.images = []
        self.img = np.zeros([537, 256, 3], np.uint8)
        self.img = cv2.circle(self.img, (128, 128), 128, (255, 255, 255), 3)
        self.img = cv2.circle(self.img, (128, 409), 128, (255, 255, 255), 3)
        self.signals = []
        
        self.cc = 0
        # print(frame1.shape)
        self.motion = True
        
        for i in range(4):
            self.signals.append(self.img.copy())
            if i == self.si:
                self.signals[i] = self.go(i)
            else:
                self.signals[i] = self.stop(i)
            self.images.append(self.signals[i])

    def display(self, t):
        for i in range(4):
            plt.subplot(1, 4, i + 1), plt.imshow(self.images[i])
            plt.title(self.titles[i])
            plt.xticks([]), plt.yticks([])
        plt.show(block=False)
        plt.pause(1)
        self.ismoving(self.si, t)
        plt.close()

    def stop(self, i):
        self.signals[i] = cv2.circle(self.signals[i], (128, 128), 125, (255, 0, 0), -1)
        self.signals[i] = cv2.circle(self.signals[i], (128, 409), 125, (0, 0, 0), -1)
        return self.signals[i]

    def go(self, i):
        self.signals[i] = cv2.circle(self.signals[i], (128, 128), 125, (0, 0, 0), -1)
        self.signals[i] = cv2.circle(self.signals[i], (128, 409), 125, (0, 255, 0), -1)
        return self.signals[i]
    
    def ismoving(self, i, t):
        i=self.si #<-<-<-<-<-<-<-<-<-<-<-<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        self.cap = cv2.VideoCapture(str(i)+'.mp4')
        self.ret, self.frame1 = self.cap.read()
        self.ret, self.frame2 = self.cap.read()
        t_end = time.time() + t
        while self.cap.isOpened() and time.time() < t_end:
            self.cc = 0
            self.diff = cv2.absdiff(self.frame1, self.frame2)
            self.gray = cv2.cvtColor(self.diff, cv2.COLOR_BGR2GRAY)
            self.blur = cv2.GaussianBlur(self.gray, (5, 5), 0)
            #cv2.imshow("gray", self.gray)
            self._, self.thresh = cv2.threshold(self.blur, 20, 255, cv2.THRESH_BINARY)
            #cv2.imshow("thresh", self.thresh)
            self.dilated = cv2.dilate(self.thresh, None, iterations=3)
            #cv2.imshow("dilate", self.dilated)
            self.contours, self._ = cv2.findContours(self.dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in self.contours:
                (self.x, self.y, self.w, self.h) = cv2.boundingRect(contour)
                if cv2.contourArea(contour) < 1000:
                    #cv2.rectangle(self.frame1, (self.x, self.y), (self.x + self.w, self.y + self.h), (0, 0, 255), 3)
                    pass
                else:
                    self.cc += 1
                    cv2.rectangle(self.frame1, (self.x, self.y), (self.x + self.w, self.y + self.h), (0, 255, 0), 3)

            cv2.putText(self.frame1, "Status: {}".format('Movement::' + str(self.cc)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            self.showframe = cv2.resize(self.frame1, (512, 512))
            cv2.imshow("feed", self.showframe)

            if self.cc==0 and self.motion:
                t_int = time.time() + 3
                self.motion = False
                #print('idle timer started:: ', time.time(), t_int)
            elif self.cc>0:
                self.motion = True
            
            if (not self.motion) and time.time()>=t_int:
                self.motion = False
                print('Skipped...')
                break

            self.frame1 = self.frame2
            self.ret, self.frame2 = self.cap.read()

            if cv2.waitKey(4) == 27:
                break
            time.sleep(0.05)

        cv2.destroyAllWindows()
        self.cap.release()
        return self.motion

def start(i,sgnl,index):
    print("\n...Signal ",sgnl," feed is activated...")
    a=activateSignal(index)
    a.display(i)
    print("...Signal ",sgnl,"feed ended...")

d={'s1':18,'s2':18,'s3':18,'s4':18}
k=list(d.keys())
while True:
    for key in d:
        start(d[key], key, k.index(key))
    print('...press q to escape...')
    if input()=='q':
        break