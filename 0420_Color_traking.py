import cv2
import numpy as np
import serial
import time

pos = 75
tlt = 75
Command = ''


##sp  = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
sp = serial.Serial('COM3', 115200, timeout=1)

##ser = serial.Serial(
   ## port = 'COM3', baudrate=115200,parity='N',stopbits=1,bytesize=8, timeout=10)
##ser.isOpen()

cap = cv2.VideoCapture(0)  # 카메라 모듈 사용.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

sp.write('pos75'.encode())
sp.write('til75'.encode())
while (1):
    ret, frame = cap.read()  # 카메라 모듈 연속프레임 읽기

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # BGR을 HSV로 변환해줌

    # define range of blue color in HSV
    lower_blue = np.array([100, 100, 120])  # 파랑색 범위
    upper_blue = np.array([150, 255, 255])

    lower_green = np.array([50, 150, 50])  # 초록색 범위
    upper_green = np.array([80, 255, 255])

    lower_red = np.array([150, 50, 50])  # 빨강색 범위
    upper_red = np.array([180, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # 110<->150 Hue(색상) 영역을 지정.
    mask1 = cv2.inRange(hsv, lower_green, upper_green)  # 영역 이하는 모두 날림 검정. 그 이상은 모두 흰색 두개로 Mask를 씌움.
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)  # 흰색 영역에 파랑색 마스크를 씌워줌.

    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    _, bin = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 초록색 사각형 그리기
    COLOR = (0, 255, 0)
    max_area = 0
    max_contour = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100 and area > max_area:  # 일정 크기 이상이면서 가장 큰 컨투어를 찾음
            max_area = area
            max_contour = cnt
    if max_contour is not None:
            x, y, width, height = cv2.boundingRect(max_contour)
            # cv2.rectangle(frame, (x, y), (x + width, y + height), COLOR, 2)

            # if area > 100:  # 일정 크기 이상인 경우에만 사각형을 그림
            x, y, width, height = cv2.boundingRect(cnt)
            # center_x = x + width //
            # center_y = y + height // 2
            # print("center: ( %s, %s )" % (center_x, center_y))
            if x < 100:
                pos = pos + 1
                if(pos > 125):
                    pos = 125
                Command = 'pan' + str(pos)

                # print(pos)
            elif x +width > 500:
                pos = pos - 1
                if(pos < 25):
                    pos = 25
                Command = 'pan' + str(pos)
                # print(pos)
            # else:
            #     pos = _pos
            #     # print("===")

            # _pos = pos

            elif y < 180:
                tlt = tlt - 1
                if(tlt < 25):
                    tlt = 25
                Command = 'tlt' + str(tlt)

                # print("less than 240-40")
            elif y + height> 380:
                tlt = tlt + 1
                if (tlt > 125):
                    tlt = 125
                Command = 'tlt' + str(tlt)
                # print("more than 240+
            else :
                tlt = tlt
                pos = pos
            # else:
            #     tlt = _tlt
            #     # print("===")
            # print(tx_dat)
            print (pos ,tlt)
            sp.write(Command.encode())
            time.sleep(0.1)
            # time.sleep(0.25)
            cv2.rectangle(frame, (x, y), (x + width, y + height), COLOR, 2)

    res1 = cv2.bitwise_and(frame, frame, mask=mask1)  # 흰색 영역에 초록색 마스크를 씌워줌.
    res2 = cv2.bitwise_and(frame, frame, mask=mask2)  # 흰색 영역에 빨강색 마스크를 씌워줌.

    cv2.imshow('frame', frame)  # 원본 영상을 보여줌
    # cv2.imshow('Blue', res)               # 마스크 위에 파랑색을 씌운 것을 보여줌.
    # cv2.imshow('Green', res1)             # 마스크 위에 초록색을 씌운 것을 보여줌.
    # cv2.imshow('red', res2)               # 마스크 위에 빨강색을 씌운 것을 보여줌.

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
