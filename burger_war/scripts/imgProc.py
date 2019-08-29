#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Add ImageProcessing --- START ---
import cv2
import numpy as np
import os

class cImgProc():
    def __init__(self):
        print('Init cImgProc')
        # image load
        self.loadNo = 3  #1:vscode  2:本番  3:実機
        self.load2(self.loadNo)  
        # debug view
        self.debug_view = 2

    def load2(self, num):
        # Get current work directory
        self.cwd = os.getcwd()
        #print('cwd=', self.cwd)

        if num == 1:    #vscode debug
            self.rila_img = cv2.imread("burger_war/scripts/img_rila_w_80x60.jpg", 1)
        elif num == 2:  #本番
            self.rila_img = cv2.imread("../catkin_ws/src/burger_war/burger_war/scripts/img_rila_w_80x60.jpg", 1)
        elif num == 3:  #実機
            self.rila_img = cv2.imread(os.path.dirname(__file__) + "img_rila_w_80x60.jpg", 1)

    # image processing main
    def imageProcess1(self, img, scan):
        # Get input image
        self.img = img

        # Get image size
        self.h = h = img.shape[0]
        self.w = w = img.shape[1]

        # Get 1/2 image (320x240)
        self.img_div2 = cv2.resize(img, (w/2, h/2))

        # Get 1/8 image (80x60)        
        self.img_div8 = cv2.resize(img, (w/8, h/8))

        # -----------------------------------------------------------
        #【python/OpenCV】画像の特定の色を抽出する方法
        #  http://rikoubou.hatenablog.com/entry/2019/02/21/190310
        # -----------------------------------------------------------        
        # Blue extract 
        lower_blue = np.array([16,  0,  0 ])  # BGR
        upper_blue = np.array([255, 16, 16])
        self.blue_mask  = cv2.inRange(self.img_div8, lower_blue, upper_blue)
        self.blue_img   = cv2.bitwise_and(self.img_div8, self.img_div8, mask=self.blue_mask)                

        # Green extract 
        lower_green = np.array([0,  16,  0 ])  # BGR
        upper_green = np.array([16, 255, 16])
        self.green_mask  = cv2.inRange(self.img_div8, lower_green, upper_green)
        self.green_img   = cv2.bitwise_and(self.img_div8, self.img_div8, mask=self.green_mask)

        # Red extract 
        lower_red = np.array([0,  0,  16 ])  # BGR
        upper_red = np.array([16, 16, 255])
        self.red_mask  = cv2.inRange(self.img_div8, lower_red, upper_red)
        self.red_img   = cv2.bitwise_and(self.img_div8, self.img_div8, mask=self.red_mask)

        # -------------------------------------------------------------------------------------
        # Blueのmask領域で最も大きい領域を抽出し、self.blue_centerに重心のx座標を格納(無は-1)
        # -------------------------------------------------------------------------------------        
        # ラベリング参考: http://okkah.hateblo.jp/entry/2018/08/02/163045
        MaxNo = -1  #初期番号
        MaxS  = 8  #初期面積
        # Blueのmaskをラベリング
        label = cv2.connectedComponentsWithStats(self.blue_mask)
        # オブジェクト毎に情報抽出
        n = label[0] - 1
        data = np.delete(label[2], 0, 0)
        center = np.delete(label[3], 0, 0)        
        # オブジェクト情報を用いて、外形描画
        for i in range(n):
            # 面積が大きい場合
            s = data[i][4]
            if MaxS < s:
                # 更新
                MaxNo = i
        # 最大面積の外接矩形を描画
        if -1 < MaxNo:
            i = MaxNo
            x0 = data[i][0]              
            y0 = data[i][1]              
            x1 = data[i][0] + data[i][2] 
            y1 = data[i][1] + data[i][3]
            cv2.rectangle(self.blue_img, (x0, y0), (x1, y1), (255, 255, 255))
            # 各オブジェクトのラベル番号と面積に黄文字で表示
            cv2.putText(self.blue_img, str(s), (x0, y1 + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 255, 255))
            #情報を格納
            self.blue_center = (x0 + x1)/2
            #debug
            cv2.line(self.blue_img, (self.blue_center, 0), (self.blue_center, h/8), (255,255,255), 1)
        else:
            #情報を格納
            self.blue_center = -1            

        # -------------------------------------------------------------------------------------
        # Greenのmask領域で最も大きい領域を抽出し、self.green_centerに重心のx座標を格納(無は-1)
        # -------------------------------------------------------------------------------------
        # ラベリング参考: http://okkah.hateblo.jp/entry/2018/08/02/163045        
        MaxNo = -1  #初期番号
        MaxS  = 3  #初期面積
        # Blueのmaskをラベリング
        label = cv2.connectedComponentsWithStats(self.green_mask)
        # オブジェクト毎に情報抽出
        n = label[0] - 1
        data = np.delete(label[2], 0, 0)
        center = np.delete(label[3], 0, 0)        
        # オブジェクト情報を用いて、外形描画
        for i in range(n):
            # 面積が大きい場合
            s = data[i][4]
            if MaxS < s:
                # 更新
                MaxNo = i
        # 最大面積の外接矩形を描画
        if -1 < MaxNo:
            i = MaxNo
            x0 = data[i][0]              
            y0 = data[i][1]              
            x1 = data[i][0] + data[i][2] 
            y1 = data[i][1] + data[i][3]
            cv2.rectangle(self.green_img, (x0, y0), (x1, y1), (255, 255, 255))
            # 各オブジェクトのラベル番号と面積に黄文字で表示
            cv2.putText(self.green_img, str(s), (x0, y1 + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 255, 255))
            #情報を格納
            self.green_center = (x0 + x1)/2
            #debug
            cv2.line(self.green_img, (self.green_center, 0), (self.green_center, h/8), (255,255,255), 1)
        else:
            #情報を格納
            self.green_center = -1 

        # -------------------------------------------------------------------------------------
        # Redのmask領域で最も大きい領域を抽出し、self.red_centerに重心のx座標を格納(無は-1)
        # -------------------------------------------------------------------------------------
        # ラベリング参考: http://okkah.hateblo.jp/entry/2018/08/02/163045        
        MaxNo = -1  #初期番号
        MaxS  = 1  #初期面積
        # Blueのmaskをラベリング
        label = cv2.connectedComponentsWithStats(self.red_mask)
        # オブジェクト毎に情報抽出
        n = label[0] - 1
        data = np.delete(label[2], 0, 0)
        center = np.delete(label[3], 0, 0)        
        # オブジェクト情報を用いて、外形描画
        for i in range(n):
            # 面積が大きい場合
            s = data[i][4]
            if MaxS < s:
                # 更新
                MaxNo = i
        # 最大面積の外接矩形を描画
        if -1 < MaxNo:
            i = MaxNo
            x0 = data[i][0]              
            y0 = data[i][1]              
            x1 = data[i][0] + data[i][2] 
            y1 = data[i][1] + data[i][3]
            cv2.rectangle(self.red_img, (x0, y0), (x1, y1), (255, 255, 255))
            # 各オブジェクトのラベル番号と面積に黄文字で表示
            cv2.putText(self.red_img, str(s), (x0, y1 + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 255, 255))
            #情報を格納
            self.red_center = (x0 + x1)/2
            #debug
            cv2.line(self.red_img, (self.red_center, 0), (self.red_center, h/8), (255,255,255), 1)
        else:
            #情報を格納
            self.red_center = -1 

        # -------------------------------------------------------------------------------------
        # self.blue_center、self.green_center、self.red_centerの状態をrila画像に枠描画
        # -------------------------------------------------------------------------------------
        # self.rila_imgを更新  
        self.load2(self.loadNo)
        # 定数計算(入力画像の1/8サイズ横幅の1/3)
        ww = w/8/3
        
        if self.red_center != -1:
            cv2.rectangle(self.rila_img, (ww*0, 0), (ww*1, h), (0, 0, 255), 3)
        if self.green_center != -1:
            cv2.rectangle(self.rila_img, (ww*1, 0), (ww*2, h), (0, 255, 0), 3)
        if self.blue_center != -1:
            cv2.rectangle(self.rila_img, (ww*2, 0), (ww*3, h), (255, 0, 0), 3)                        


        # ---------------------------------------------------------------
        # センサーデーター
        # ---------------------------------------------------------------
        # カメラ (Logicool C270)の画角は60度。4:3(640x480)として水平画角は49.5°)
        # 参考：https://www.logicool.co.jp/ja-jp/press/press-releases/7395　→ C270は画角60度
        # 一旦水平50画素のdepth画像を作成し、LiDARの水平50度分を割当
        #  [0]・・[24][25]・・[49]
        #
        # LiDAR (HLS-LFCD2)は上から見ると１２時を基準に半時計回りに360度の点群を出力
        # Detection distance:120mm - 3,500mm、Angular Range:360°、Angular Resolution:1°
        # 参考: http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/
        #       ranges[0]
        #         [1]    [359]
        #      [2]            [358]
        #  [24]                    [359-24]

        # scan data
        # カメラの水平画角分のScan Dataを抽出
        d = [0] * 50  #カメラの水平画角50度に対応
        for i in range(25):
            d[25+i] = range255(scan.ranges[359 -i], 100)  # m→cm変換(x100)
            d[24-i] = range255(scan.ranges[i],      100)  # m→cm変換(x100)
            #print('i, d[i]', i, d[i])

        # 画像配列に格納
        dd = np.zeros((1,50,3), np.uint8) #1x50(行×列)x3の配列作成
        for i in range(50):
            dd.itemset((0,i,0), d[i])
            dd.itemset((0,i,1), d[i])
            dd.itemset((0,i,2), d[i]) 

        # 現在の処理サイズにリサイズ
        self.depth_img = cv2.resize(dd, (80, 60))

        # 中央付近の平均距離を格納
        self.center_depth = (scan.ranges[359-10] + scan.ranges[359-5] + scan.ranges[0] + scan.ranges[5] + scan.ranges[10])/5

        # debug (中心付近に10x10の赤枠描画)
        cv2.rectangle(self.depth_img, (40-5,30-5), (40+5, 30+5), (0,0,255), 2)
        cv2.rectangle(self.depth_img, (40,37), (79, 42), (0,0,0), -1)
        cv2.putText(self.depth_img, str(self.center_depth), (40, 35 + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 255, 255))

       # ---------------------------------------------------------------
        # debug画像の結合
        # ---------------------------------------------------------------
        # debug image
        # cv2.hconcat, cv2.vconcatは同じ画像サイズ(80x60)同士の事
        # Python, OpenCVで画像を縦・横に連結 (hconcat, vconcat, np.tile)
        # https://note.nkmk.me/python-opencv-hconcat-vconcat-np-tile/
        self.debug11_img = cv2.hconcat([self.img_div8, self.rila_img, self.depth_img])
        self.debug12_img = cv2.hconcat([self.red_img, self.green_img, self.blue_img])

        # Python, OpenCVで図形描画（線、長方形、円、矢印、文字など）
        # https://note.nkmk.me/python-opencv-draw-function/
        cv2.rectangle(self.debug12_img, (80*0,0), (80*1-1, 60-1), (0,0,255), 2)
        cv2.rectangle(self.debug12_img, (80*1,0), (80*2-1, 60-1), (0,255,0), 2)
        cv2.rectangle(self.debug12_img, (80*2,0), (80*3-1, 60-1), (255,0,0), 2)
        self.debug1_img = cv2.vconcat([self.debug11_img, self.debug12_img])

# --- test function START ---
#load image
def load():
    # memo
    #img2 = np.zeros((640,480,3), np.uint8)

    # path ~/catkin_ws/src/burger_war
    img = cv2.imread("burger_war/scripts/img_rila.jpg", 1)
    return img

def range255(value, gain):
    v = value * gain
    if v < 0:
        return 0
    elif 255 < v:
        return 255
    else:
        return int(v)

def range255_inv(value, gain):
    v = value * gain
    if v < 0:
        return 255
    elif 255 < v:
        return 0
    else:
        return 255 - int(v)        

if __name__ == '__main__':
    print('main')
# --- test function END ---

# Add ImageProcessing --- END ---