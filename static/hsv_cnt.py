
import cv2
import numpy as np

image = cv2.imread('img/test1.jpg')  # 根据路径读取一张图片，opencv读出来的是BGR模式
cv2.imshow("BGR", image)  # 显示图片

hsv_low = np.array([0, 150, 0])
hsv_high = np.array([255, 255, 255])

# 下面几个函数，写得有点冗余


def h_low(value):
    hsv_low[0] = value


def h_high(value):
    hsv_high[0] = value


def s_low(value):
    hsv_low[1] = value


def s_high(value):
    hsv_high[1] = value


def v_low(value):
    hsv_low[2] = value


def v_high(value):
    hsv_high[2] = value


cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)

cv2.createTrackbar('H low', 'image', 0, 255, h_low)
cv2.createTrackbar('H high', 'image', 0, 255, h_high)
cv2.createTrackbar('S low', 'image', 0, 255, s_low)
cv2.createTrackbar('S high', 'image', 0, 255, s_high)
cv2.createTrackbar('V low', 'image', 0, 255, v_low)
cv2.createTrackbar('V high', 'image', 0, 255, v_high)

while True:
    dst = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    dst = cv2.inRange(dst, hsv_low, hsv_high)  # 通过HSV的高低阈值，提取图像部分区域
    cv2.imshow('dst', dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
