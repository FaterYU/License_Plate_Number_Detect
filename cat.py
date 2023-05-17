import cv2
import matplotlib.pyplot as plt
import numpy as np
img = cv2.imread('img/test1.jpg')
# to rgb
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# to hls
img_hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
# find white color
mask_hls_white = cv2.inRange(img_hls, (0, 150, 0), (255, 255, 255))
# find blue color
mask_hls_blue = cv2.inRange(img_hls, (90, 50, 50), (120, 255, 255))
mask_add = cv2.add(mask_hls_white, mask_hls_blue)
mask_subtract = cv2.subtract(mask_hls_white, mask_hls_blue)
mask_aim = cv2.subtract(mask_add, mask_subtract)
# bitwise_and mask
img_hls_white = cv2.bitwise_and(img, img, mask=mask_hls_white)
img_hls_blue = cv2.bitwise_and(img, img, mask=mask_hls_blue)
img_hls_white_blue_add = cv2.bitwise_and(img, img, mask=mask_add)
img_hls_white_blue_subtract = cv2.bitwise_and(img, img, mask=mask_subtract)
img_hls_white_blue_aim = cv2.bitwise_and(img, img, mask=mask_aim)

img_bgr_white = cv2.cvtColor(img_hls_white, cv2.COLOR_HLS2BGR)
img_bgr_blue = cv2.cvtColor(img_hls_blue, cv2.COLOR_HLS2BGR)
img_bgr_white_blue_add = cv2.cvtColor(
    img_hls_white_blue_add, cv2.COLOR_HLS2BGR)
img_bgr_white_blue_subtract = cv2.cvtColor(
    img_hls_white_blue_subtract, cv2.COLOR_HLS2BGR)
img_bgr_white_blue_aim = cv2.cvtColor(
    img_hls_white_blue_aim, cv2.COLOR_HLS2BGR)

img_hls_white_gray = cv2.cvtColor(img_hls_white, cv2.COLOR_BGR2GRAY)
img_hls_blue_gray = cv2.cvtColor(img_hls_blue, cv2.COLOR_BGR2GRAY)
img_hls_white_blue_add_gray = cv2.cvtColor(
    img_bgr_white_blue_add, cv2.COLOR_BGR2GRAY)
img_hls_white_blue_subtract_gray = cv2.cvtColor(
    img_bgr_white_blue_subtract, cv2.COLOR_BGR2GRAY)
img_hls_white_blue_aim_gray = cv2.cvtColor(
    img_bgr_white_blue_aim, cv2.COLOR_BGR2GRAY)

img_bgr_white_gray = cv2.cvtColor(img_bgr_white, cv2.COLOR_BGR2GRAY)
img_bgr_blue_gray = cv2.cvtColor(img_bgr_blue, cv2.COLOR_BGR2GRAY)
img_bgr_white_blue_add_gray = cv2.cvtColor(
    img_bgr_white_blue_add, cv2.COLOR_BGR2GRAY)
img_bgr_white_blue_subtract_gray = cv2.cvtColor(
    img_bgr_white_blue_subtract, cv2.COLOR_BGR2GRAY)
img_bgr_white_blue_aim_gray = cv2.cvtColor(
    img_bgr_white_blue_aim, cv2.COLOR_BGR2GRAY)

# 图像二值化
img_white_bin = cv2.threshold(
    img_bgr_white_gray, 127, 255, cv2.THRESH_BINARY)[1]
img_blue_bin = cv2.threshold(img_bgr_blue_gray, 127, 255, cv2.THRESH_BINARY)[1]
img_white_blue_add_bin = cv2.threshold(
    img_bgr_white_blue_add_gray, 127, 255, cv2.THRESH_BINARY)[1]
img_white_blue_subtract_bin = cv2.threshold(
    img_bgr_white_blue_subtract_gray, 127, 255, cv2.THRESH_BINARY)[1]
img_white_blue_aim_bin = cv2.threshold(
    img_bgr_white_blue_aim_gray, 50, 255, cv2.THRESH_BINARY)[1]

# 获取 img_white_blue_aim_bin 连通区域
img_white_blue_aim_bin = cv2.bitwise_not(img_white_blue_aim_bin)
img_white_blue_aim_bin = cv2.dilate(img_white_blue_aim_bin, None, iterations=1)
img_white_blue_aim_bin = cv2.erode(img_white_blue_aim_bin, None, iterations=1)
img_white_blue_aim_bin = cv2.bitwise_not(img_white_blue_aim_bin)
contours, hierarchy = cv2.findContours(
    img_white_blue_aim_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
result_bgr = None
for i1 in contours:
    x, y, w, h = cv2.boundingRect(i1)
    if w > 2.2*h:
        print(w, h, w/h, w*h)
        result_bgr = img[y:y+h, x:x+w]
        break
result_bgr_gray = cv2.cvtColor(result_bgr, cv2.COLOR_RGB2GRAY)

reasonable_list = []
reasonable_cnt = []
for i in range(100, 180):

    result_bgr_gray_bin = cv2.threshold(
        result_bgr_gray, i, 255, cv2.THRESH_BINARY)[1]

    # 连通域
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        result_bgr_gray_bin, connectivity=4)

    reasonable_list = []
    cnt = 0
    for i in range(1, num_labels):
        mask = labels == i
        cnt += np.sum(mask)
    for i in range(1, num_labels):
        mask = labels == i
        if np.sum(mask)/cnt < 0.2 and np.sum(mask)/cnt > 0.02:
            reasonable_list.append(i)
    if len(reasonable_list) >= 7:
        print(i)
        break

char_contours = []
for i in reasonable_list:
    output = np.zeros((result_bgr.shape[0], result_bgr.shape[1], 1), np.uint8)
    mask = labels == i
    output[:, :, 0][mask] = 255
    sub_char_contours, hierarchy = cv2.findContours(
        output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for i in sub_char_contours:
        x, y, w, h = cv2.boundingRect(i)
        char_contours.append([x, y, w, h])

for i in char_contours:
    x, y, w, h = i
    cv2.rectangle(result_bgr, (x, y), (x + w, y + h), (0, 255, 0), 1)

plt_rgb = cv2.cvtColor(result_bgr, cv2.COLOR_BGR2RGB)
plt.imshow(plt_rgb)
plt.show()
