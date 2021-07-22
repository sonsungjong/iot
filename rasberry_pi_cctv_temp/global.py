import CCTV
import faceDetect2

def main() :
    cctv = CCTV.CCTV()
    fd = faceDetect2.FaceDetect()
    
    result = 0
    
    while True :
        if result == 1 :
            result = fd.faceDetect(result)
        elif result == 0 :
            result = cctv.Recording(result)
    
    
    
if __name__ == "__main__":
    main()
