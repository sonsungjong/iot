import board
import busio as io
import adafruit_mlx90614
import cv2
import mediapipe as mp
import numpy as np
from PIL import ImageFont, ImageDraw, Image     # pip3 install pillow
import pygame
import time
import threading

# 눈2, 코, 입을 좌표로 잡아 얼굴로 인식

# 얼굴 디텍션 클래스
class FaceDetector():
    #pos1 = 0
    #pos2 = 0
    # 생성자
    def __init__(self, minDetectionCon = 0.5):
        self.minDetectionCon = minDetectionCon
        self.mpFaceDetection = mp.solutions.face_detection
        self.mpDraw = mp.solutions.drawing_utils
        self.faceDetection = self.mpFaceDetection.FaceDetection(
            self.minDetectionCon)

    # 얼굴찾기 함수
    def findFaces(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.faceDetection.process(imgRGB)
        # print(self.results)
        bboxs = []

        if self.results.detections:
            for id, detection in enumerate(self.results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, ic = img.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                    int(bboxC.width * iw), int(bboxC.height * ih)
                bboxs.append([id, bbox, detection.score])
                # position = (210,180, 40,40)     # x,y, width, height
                if draw:
                    img = self.fancyDraw(img, bbox)
                    # cv2.putText(img, f'{int(detection.score[0]*100)}%', (bbox[0], bbox[1]-20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
                    #self.pos1 = bbox[0]
                    #self.pos2 = bbox[1]-20
        return img, bboxs
    
    # 디텍팅+사각형 도형그리기 함수
    def fancyDraw(self, img, bbox, l=30, t=3, rt=1):
        x, y, w, h = bbox
        x1, y1 = x+w, y+h
        
        # top box
        cv2.rectangle(img, (0,0), (480, 60), (255, 255, 255), -1)
        # bottom box
        cv2.rectangle(img, (0,580), (480, 640), (255, 255, 255), -1)
        # cv2.rectangle(img, bbox, (255, 0, 255), rt)
        # Top Left x,y
        cv2.line(img, (x,y), (x+l, y),(0,255,0),t)
        cv2.line(img, (x,y), (x, y+l),(0,255,0),t)

        # Top Right x1,y
        cv2.line(img, (x1, y), (x1-l, y), (0, 255, 0), t)
        cv2.line(img, (x1, y), (x1, y+l), (0, 255, 0), t)

        # Bottom Left x,y1
        cv2.line(img, (x, y1), (x+l, y1), (0, 255, 0), t)
        cv2.line(img, (x, y1), (x, y1-l), (0, 255, 0), t)

        # Bottom Right x1,y1
        cv2.line(img, (x1, y1), (x1-l, y1), (0, 255, 0), t)
        cv2.line(img, (x1, y1), (x1, y1-l), (0, 255, 0), t)

        # center
        # cv2.circle(img, (int(x+w/2),int(y+h/2)), 2 , (0,255,0), t)
        cv2.line(img, (240, 120), (240, 160), (255, 255, 0), 4)
        cv2.line(img, (220, 140), (260, 140), (255, 255, 0), 4)
        return img

class SoundPut():
    def __init__(self):
        pygame.init()
        self.normal = pygame.mixer.Sound('normal.ogg')
        self.high = pygame.mixer.Sound('recheck.ogg')
    def normalTemp(self):
        self.normal.play()
        time.sleep(2)
        #self.normal.stop()
    
    def highTemp(self):
        self.high.play()
        time.sleep(2)
        #self.high.stop()
        
    def __del__(self):
        pygame.quit()

class FaceDetect():
    flag = 1
    def dclick(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.flag = 0
    
    def faceDetect(self, result):
        self.flag = result
        fontpath = "nanum.ttf"
        temp = 0.0
        i2c = io.I2C(board.SCL, board.SDA, frequency=100000)
        font = ImageFont.truetype(fontpath, 46)     # 폰트, 글씨크기
        sund = SoundPut()
        
        cap = cv2.VideoCapture(0)
        detector = FaceDetector()
        
        # 전체화면으로 설정
        cv2.namedWindow("Image", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, 1)
        cv2.setMouseCallback("Image",self.dclick)
            
        while True:
            
            if self.flag == 0 :
                return self.flag
            
            t = None
            success, img = cap.read()
            if(success):
                img = cv2.resize(img, (480, 640))
                img, bboxs = detector.findFaces(img)
                img = cv2.flip(img, 1)
                img_pil = Image.fromarray(img)
                draw = ImageDraw.Draw(img_pil)
                # 얼굴디텍트 True
                if bboxs:
                    temp = (adafruit_mlx90614.MLX90614(i2c).object_temperature) +1.6
                    time.sleep(0.05)
                    if temp <= 32:
                        draw.text((8, 580), "초점에 이마를 맞춰주세요", font=font, fill=(40,40,40, 1))
                        img = np.array(img_pil)
                    elif temp < 37.5:
                        draw.text((180,15), "{:.1f}°C".format(temp), font=font, fill=(0, 160, 0, 1))
                        draw.text((100, 580), "정상 온도입니다", font=font, fill=(40,40,40, 1))
                        img = np.array(img_pil)
                        t = threading.Thread(target=sund.normalTemp)
                        t.start()
                        
                    else:
                        draw.text((185, 15), "{:.1f}".format(temp), font=font, fill=(0, 0, 255, 1))
                        img = np.array(img_pil)
                        t = threading.Thread(target=sund.highTemp)
                        t.start()
                        
                # 캠화면 출력
                cv2.imshow("Image", img)
                cv2.waitKey(1)
                if(t):
                    time.sleep(2)
                    t = None
            else:
                break
        cap.release()
        cv2.destroyAllWindows()
    
def main():
    fd = FaceDetect()
    fd.faceDetect()

if __name__ == "__main__":
    main()
