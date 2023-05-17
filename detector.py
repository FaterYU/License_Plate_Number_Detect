import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


class Detector():
    def __init__(self):
        self.licence_plate_sig_bin = []
        self.font_ch = []
        self.font_ch_name = []
        self.font_num = []
        self.font_num_name = []
        self.font_word = []
        self.font_word_name = []
        self.font_read()
        self.result = []

    def detect(self, img):
        self.cash_img = img
        # to rgb
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # to hsv
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # to hsl
        img_hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        # find white color
        mask_hsv_white = cv2.inRange(img_hls, (0, 150, 0), (255, 255, 255))
        # find only blue color
        mask_hsv_blue = cv2.inRange(img_hsv, (100, 43, 120), (124, 255, 255))
        mask_add = cv2.add(mask_hsv_white, mask_hsv_blue)
        mask_subtract = cv2.subtract(mask_hsv_white, mask_hsv_blue)
        mask_aim = cv2.subtract(mask_add, mask_subtract)
        # bitwise_and mask
        img_hsv_white = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_hsv_white)
        img_hsv_blue = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_hsv_blue)
        img_hsv_white_blue_add = cv2.bitwise_and(
            img_hsv, img_hsv, mask=mask_add)
        img_hsv_white_blue_subtract = cv2.bitwise_and(
            img_hsv, img_hsv, mask=mask_subtract)
        img_hsv_white_blue_aim = cv2.bitwise_and(
            img_hsv, img_hsv, mask=mask_aim)

        img_bgr_white = cv2.cvtColor(img_hsv_white, cv2.COLOR_HSV2BGR)
        img_bgr_blue = cv2.cvtColor(img_hsv_blue, cv2.COLOR_HSV2BGR)
        img_bgr_white_blue_add = cv2.cvtColor(
            img_hsv_white_blue_add, cv2.COLOR_HSV2BGR)
        img_bgr_white_blue_subtract = cv2.cvtColor(
            img_hsv_white_blue_subtract, cv2.COLOR_HSV2BGR)
        img_bgr_white_blue_aim = cv2.cvtColor(
            img_hsv_white_blue_aim, cv2.COLOR_HSV2BGR)

        img_hsv_white_gray = cv2.cvtColor(img_hsv_white, cv2.COLOR_BGR2GRAY)
        img_hsv_blue_gray = cv2.cvtColor(img_hsv_blue, cv2.COLOR_BGR2GRAY)
        img_hsv_white_blue_add_gray = cv2.cvtColor(
            img_bgr_white_blue_add, cv2.COLOR_BGR2GRAY)
        img_hsv_white_blue_subtract_gray = cv2.cvtColor(
            img_bgr_white_blue_subtract, cv2.COLOR_BGR2GRAY)
        img_hsv_white_blue_aim_gray = cv2.cvtColor(
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
        threshold_value = 30
        img_white_bin = cv2.threshold(
            img_bgr_white_gray, threshold_value, 255, cv2.THRESH_BINARY)[1]
        img_blue_bin = cv2.threshold(
            img_bgr_blue_gray, threshold_value, 255, cv2.THRESH_BINARY)[1]
        img_white_blue_add_bin = cv2.threshold(
            img_bgr_white_blue_add_gray, threshold_value, 255, cv2.THRESH_BINARY)[1]
        img_white_blue_subtract_bin = cv2.threshold(
            img_bgr_white_blue_subtract_gray, threshold_value, 255, cv2.THRESH_BINARY)[1]
        img_white_blue_aim_bin = cv2.adaptiveThreshold(
            img_bgr_white_blue_aim_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 3, 2)

        # img_white_bin_rgb = cv2.cvtColor(img_white_bin, cv2.COLOR_GRAY2BGR)
        # plt.imshow(img_white_bin_rgb)
        # plt.show()
        # img_blue_bin_rgb = cv2.cvtColor(img_blue_bin, cv2.COLOR_GRAY2BGR)
        # plt.imshow(img_blue_bin_rgb)
        # plt.show()
        # img_white_blue_add_bin_rgb = cv2.cvtColor(img_white_blue_add_bin, cv2.COLOR_GRAY2BGR)
        # plt.imshow(img_white_blue_add_bin_rgb)
        # plt.show()
        # img_white_blue_subtract_bin_rgb = cv2.cvtColor(img_white_blue_subtract_bin, cv2.COLOR_GRAY2BGR)
        # plt.imshow(img_white_blue_subtract_bin_rgb)
        # plt.show()
        # img_white_blue_aim_bin_rgb = cv2.cvtColor(img_white_blue_aim_bin, cv2.COLOR_GRAY2BGR)
        # plt.imshow(img_white_blue_aim_bin_rgb)
        # plt.show()

        # findContours
        contours, hierarchy = cv2.findContours(
            img_white_blue_aim_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result_bgr_list = []
        for i1 in contours:
            x, y, w, h = cv2.boundingRect(i1)
            if w > 2.2*h and w < 4*h and w > 50:
                white_img = img_white_bin[y:y+h, x:x+w]
                white_cnt = np.sum(white_img == 255)
                blue_img = img_blue_bin[y:y+h, x:x+w]
                blue_cnt = np.sum(blue_img == 255)
                if(white_cnt < 0.3*w*h and blue_cnt > 0.3*w*h and white_cnt+blue_cnt > 150):
                    result_bgr_list.append(img[y:y+h, x:x+w])
        for result_bgr in result_bgr_list:
            result_bgr_gray = cv2.cvtColor(result_bgr, cv2.COLOR_BGR2GRAY)

            # 直方图
            result_bgr_gray_hist = cv2.calcHist(
                [result_bgr_gray], [0], None, [256], [0, 256])

            reasonable_list = []
            for i in range(60, 180):
                result_bgr_gray_bin = cv2.threshold(
                    result_bgr_gray, i, 255, cv2.THRESH_BINARY)[1]

                # 连通域
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                    result_bgr_gray_bin, connectivity=4)

                output = np.zeros(
                    (result_bgr.shape[0], result_bgr.shape[1], 3), np.uint8)
                for i in range(1, num_labels):
                    mask = labels == i
                    output[:, :, 0][mask] = np.random.randint(0, 255)
                    output[:, :, 1][mask] = np.random.randint(0, 255)
                    output[:, :, 2][mask] = np.random.randint(0, 255)
                # print(num_labels, len(reasonable_list))
                # plt.imshow(output)
                # plt.show()

                reasonable_list = []
                cnt = 0
                for j in range(1, num_labels):
                    mask = labels == j
                    cnt += np.sum(mask)
                for j in range(1, num_labels):
                    mask = labels == j
                    if np.sum(mask)/cnt < 0.2 and np.sum(mask)/cnt > 0.035:
                        reasonable_list.append(j)
                if len(reasonable_list) >= 7 and num_labels - len(reasonable_list) < 20:
                    print(num_labels, len(reasonable_list))
                    # plt.imshow(output)
                    # plt.show()
                    break

            if len(reasonable_list) < 7:
                break

            char_contours = []
            avg_h = 0
            for i in reasonable_list:
                output = np.zeros(
                    (result_bgr.shape[0], result_bgr.shape[1], 1), np.uint8)
                mask = labels == i
                output[:, :, 0][mask] = 255
                sub_char_contours, hierarchy = cv2.findContours(
                    output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for j in sub_char_contours:
                    x, y, w, h = cv2.boundingRect(j)
                    char_contours.append([x, y, w, h])
                    avg_h += h
            avg_h = avg_h/len(reasonable_list)
            # sort by x
            char_contours.sort(key=lambda x: x[0])
            sum_result = np.average(result_bgr_gray)
            entire_result_bgr_gray_bin = cv2.threshold(
                result_bgr_gray, sum_result+0.16*result_bgr.shape[1], 255, cv2.THRESH_BINARY)[1]
            plt.imshow(cv2.cvtColor(
                entire_result_bgr_gray_bin, cv2.COLOR_BGR2RGB))
            plt.show()

            width, height = 0, 0
            for i in char_contours:
                if i[3] < avg_h*0.6 or i[3] > avg_h*1.4:
                    continue
                if i[2] > result_bgr.shape[1]/7:
                    continue
                width = max(width, i[2])
                height = max(height, i[3])
            print(width, height)

            licence_plate_sig_xywh = []
            for i in char_contours:
                if i[3] < avg_h*0.6 or i[3] > avg_h*1.4:
                    continue
                if i[2] > result_bgr.shape[1]/7:
                    continue
                centre_x = i[0]+i[2]/2
                centre_y = i[1]+i[3]/2
                self.licence_plate_sig_bin.append(cv2.bitwise_not(
                    entire_result_bgr_gray_bin[int(centre_y-height/2):int(centre_y+height/2), int(centre_x-width/2):int(centre_x+width/2)]))
                # self.licence_plate_sig_bin.append(cv2.bitwise_not(
                #     entire_result_bgr_gray_bin[i[1]:i[1]+i[3], i[0]:i[0]+i[2]]))
                licence_plate_sig_xywh.append(i)
                # plt.imshow(cv2.cvtColor(
                #     self.licence_plate_sig_bin[-1], cv2.COLOR_BGR2RGB))
                # plt.show()

            avg_w = 0
            avg_h = 0
            avg_delta_x = 0
            for i in licence_plate_sig_xywh[1:]:
                avg_w += i[2]
                avg_h += i[3]
            for i in range(2, len(licence_plate_sig_xywh)-1):
                avg_delta_x += licence_plate_sig_xywh[i+1][0] - \
                    licence_plate_sig_xywh[i][0] - \
                    licence_plate_sig_xywh[i][2]
            avg_w = avg_w/(len(licence_plate_sig_xywh)-1)*1.1
            avg_h = avg_h/(len(licence_plate_sig_xywh)-1)*1.1
            avg_delta_x = avg_delta_x/(len(licence_plate_sig_xywh)-2)
            x = licence_plate_sig_xywh[1][0] - avg_delta_x - avg_w
            y = licence_plate_sig_xywh[1][1]
            x = max(0, x)
            y = max(0, y)
            print(abs(licence_plate_sig_xywh[0][3]-avg_h)/avg_h,
                  abs(licence_plate_sig_xywh[0][2]-avg_w)/avg_w)
            if abs(licence_plate_sig_xywh[0][3]-avg_h)/avg_h > 0.2 or abs(licence_plate_sig_xywh[0][2]-avg_w)/avg_w > 0.3:
                self.licence_plate_sig_bin[0] = cv2.bitwise_not(
                    entire_result_bgr_gray_bin[y:y+int(avg_h), int(x):int(x+avg_w)])

    def font_read(self):
        path = './font'
        sub_path = os.listdir(path)
        for i in sub_path:
            all_files_name = os.listdir(path+'/'+i)
            if i == 'chinese':
                for j in all_files_name:
                    self.font_ch.append(cv2.imdecode(
                        np.fromfile(path+'/'+i+'/'+j, dtype=np.uint8), -1))
                    self.font_ch_name.append(j)
            elif i == 'word':
                for j in all_files_name:
                    self.font_word.append(cv2.imread(path+'/'+i+'/'+j))
                    self.font_word_name.append(j)
                for j in all_files_name:
                    self.font_num.append(cv2.imread(path+'/'+i+'/'+j))
                    self.font_num_name.append(j)
            elif i == 'number':
                for j in all_files_name:
                    self.font_num.append(cv2.imread(path+'/'+i+'/'+j))
                    self.font_num_name.append(j)

    # pHash
    def pHash(self, img):
        # 缩放32*32
        # img = cv2.resize(img, (32, 32))
        # 将灰度图转为浮点型，再进行dct变换
        dct = cv2.dct(np.float32(img))
        # opencv实现的掩码操作
        dct_roi = dct[0:8, 0:8]
        hash = []
        avreage = np.mean(dct_roi)
        for i in range(dct_roi.shape[0]):
            for j in range(dct_roi.shape[1]):
                if dct_roi[i, j] > avreage:
                    hash.append(1)
                else:
                    hash.append(0)
        return hash

    # hamming distance
    def hamming_distance(self, hash1, hash2):
        num = 0
        for i in range(len(hash1)):
            if hash1[i] != hash2[i]:
                num += 1
        return num

    def del_white(self, i):
        for x in range(len(i)):
            col = 0
            for y in range(len(i[x])):
                if i[x][y] == 0:
                    col += 1
            if col != 0:
                i = i[x:, :]
                break
        for x in range(len(i)-1, -1, -1):
            col = 0
            for y in range(len(i[x])):
                if i[x][y] == 0:
                    col += 1
            if col != 0:
                i = i[:x+1, :]
                break
        for y in range(len(i[0])):
            row = 0
            for x in range(len(i)):
                if i[x][y] == 0:
                    row += 1
            if row != 0:
                y = min(y, 2)
                i = i[:, y:]
                break
        for y in range(len(i[0])-1, -1, -1):
            row = 0
            for x in range(len(i)):
                if i[x][y] == 0:
                    row += 1
            if row != 0:
                y = max(y+1, len(i[0])-2)
                i = i[:, :y]
                break
        return i

    def hamming_match(self):
        result = []
        hamming_distance = []
        i = self.licence_plate_sig_bin[0]
        i = self.del_white(i)
        # plt.imshow(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
        # plt.show()
        for j in self.font_ch:
            width, height = i.shape[1], i.shape[0]
            rs_img = cv2.resize(cv2.cvtColor(j, cv2.COLOR_BGR2GRAY),
                                (width, height))
            # cv2.imshow('rs_img', rs_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # res = cv2.matchTemplate(i, rs_img, cv2.TM_CCOEFF_NORMED)
            # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            # print(max_val)

            # hamming_distance.append(self.hamming_distance(
            #     self.pHash(i), self.pHash(rs_img)))

            hamming_distance.append(cv2.norm(i, rs_img, cv2.NORM_HAMMING))
        # print(hamming_distance)
        # for i in range(len(hamming_distance)):
        #     print(self.font_ch_name[i], hamming_distance[i])
        # find min hamming distance
        result.append(
            self.font_ch_name[hamming_distance.index(min(hamming_distance))])
        # print(result)

        i = self.licence_plate_sig_bin[1]
        i = self.del_white(i)
        # plt.imshow(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
        # plt.show()
        hamming_distance = []
        for j in self.font_word:
            width, height = i.shape[1], i.shape[0]
            rs_img = cv2.resize(cv2.cvtColor(j, cv2.COLOR_BGR2GRAY),
                                (width, height))
            rs_img_bin = cv2.threshold(
                rs_img, 100, 255, cv2.THRESH_BINARY)[1]
            # hamming_distance.append(self.hamming_distance(
            #     self.pHash(i), self.pHash(rs_img)))
            hamming_distance.append(cv2.norm(i, rs_img_bin, cv2.NORM_HAMMING))
            # plt.imshow(cv2.cvtColor(rs_img_bin, cv2.COLOR_BGR2RGB))
            # plt.show()
            # plt.imshow(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
            # plt.show()
        # print(hamming_distance)
        # for i in range(len(hamming_distance)):
        #     print(self.font_num_name[i], hamming_distance[i])
        # find min hamming distance
        result.append(
            self.font_word_name[hamming_distance.index(min(hamming_distance))])

        for i in self.licence_plate_sig_bin[2:]:
            i = self.del_white(i)
            # plt.imshow(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
            # plt.show()
            hamming_distance = []
            for j in self.font_num:
                width, height = i.shape[1], i.shape[0]
                rs_img = cv2.resize(cv2.cvtColor(j, cv2.COLOR_BGR2GRAY),
                                    (width, height))
                # hamming_distance.append(self.hamming_distance(
                #     self.pHash(i), self.pHash(rs_img)))
                hamming_distance.append(cv2.norm(i, rs_img, cv2.NORM_HAMMING))
            # print(hamming_distance)
            # for i in range(len(hamming_distance)):
            #     print(self.font_num_name[i], hamming_distance[i])
            # find min hamming distance
            result.append(
                self.font_num_name[hamming_distance.index(min(hamming_distance))])
        for i in range(len(result)):
            result[i] = result[i].split('_')[1].split('.')[0]
        self.result = result

    def orb_match(self):
        orb = cv2.ORB_create()
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # for i in self.licence_plate_sig_bin:
        i = self.licence_plate_sig_bin[0]
        for j in self.font_ch:
            width, height = i.shape[1], i.shape[0]
            j = cv2.resize(cv2.cvtColor(j, cv2.COLOR_BGR2GRAY),
                           (width, height))
            kp1, des1 = orb.detectAndCompute(i, None)
            kp2, des2 = orb.detectAndCompute(j, None)
            print(len(kp1), len(kp2))
            matches = bf.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)
            img3 = cv2.drawMatches(i, kp1, j, kp2,
                                   matches[:10], None, flags=2)
            plt.imshow(cv2.cvtColor(img3, cv2.COLOR_BGR2RGB))
            plt.show()

    def filter_match(self):
        filter = np.array([[0, 1, 0],
                           [1, 1, 1],
                           [0, 1, 0]], np.uint8)
        res = cv2.filter2D(self.licence_plate_sig_bin[3], -1, filter)
        print(res)
        plt.imshow(cv2.cvtColor(res, cv2.COLOR_BGR2RGB))
        plt.show()

    def display(self):
        result_str = ''.join(self.result)
        print(result_str)
        img = self.cash_img
        plt.text(10, 30, result_str, fontsize=20, color='red')
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.show()


if __name__ == '__main__':
    img = cv2.imread('img/problem_test.jpg')
    detector = Detector()
    detector.detect(img)
    print(len(detector.licence_plate_sig_bin))
    detector.hamming_match()
    # detector.orb_match()
    # detector.filter_match()
    detector.display()
