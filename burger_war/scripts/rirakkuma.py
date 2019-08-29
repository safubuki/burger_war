#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------
# ファイル名 ：rirakkuma.py
# 機能概要  ：2019ROBOCON（予選用）
#           ・move_baseを使用
#           ・スタート/ゴール地点は"Start_Goal.csv"ファイルで設定する
# 作成日時  ：2019/08/19
# -----------------------------------------------------------------------

# Import
#   common
import rospy
import math
#   move_base
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
#   Twist
from geometry_msgs.msg import Twist
#   file
import csv  # csv file
import os   # file path
#   euler to quaternio
import tf
from geometry_msgs.msg import Quaternion

# Add ImageProcessing --- START ---
# use LaserScan
from sensor_msgs.msg import LaserScan

# use Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Image Process function
import imgProc        #function
from imgProc import * #class

# Add ImageProcessing --- END ---

import math
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

#camera_fov = 50.0
#camera_width = 640.0


# PythonでEnum的なことを実現
class MoveState():
    STOP         = 0
    RUN          = 1
    DEFENSE      = 2

class RirakkumaBot():
    def __init__(self, bot_name):
        ### Parameter Settings
        # bot name 
        # RESPECT @hotic06 ロボット名の取得の仕方
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name
        print(self.name)    #debug

        # State
        self.move_state      = MoveState.STOP   # 移動状態           ：停止
        # CSV ファイルから取り出したデータ保存用リスト
        self.c_data          = []               # csvデータ
        self.c_data_cnt      = 0                # csvデータ順次取得のためのカウンタ
        # simple/goal用のシーケンス番号 ※これ無いとエラーになるため必要
        self.goal_seq_no     = 0

        ### Publisher を ROS Masterに登録
        # Velocity
        self.vel_pub         = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.pub_goal        = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        ### Subscriber を ROS Masterに登録
        self.sub_goal_result = rospy.Subscriber("move_base/result", MoveBaseActionResult, self.result_callback, queue_size=1)
                # Add ImageProcessing --- START ---
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # for convert image topic to opencv obj
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        #self.image_sub = rospy.Subscriber('/red_bot/image_raw', Image, self.imageCallback)

        #cImgProc instance
        self.proc = cImgProc()
        # Add ImageProcessing --- END ---

        # tf
        #self._tf_listener = tf.TransformListener()
        # Marker
        #self.marker_pub = rospy.Publisher('enemy_position', Marker, queue_size = 1)
        # greenflag
        #self.greenflag = 0


    def calcTwist_rand(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0

        # 更新
        print("random x,th=", x, th)    
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def calcTwist_center(self, center, depth):
        if center != -1:
            val = int(center / 16) #centerを0-4の10段階に
            if   val == 4:
                x  =  0.2
                th = -0.2 

            elif val == 3:
                x  =  0.2
                th = -0.1

            elif val == 2:
                if depth   < 0.3:
                    x = 0.00
                elif depth < 0.5:
                    x = 0.04
                elif depth < 0.7:
                    x = 0.08      
                elif depth < 0.9:
                    x = 0.12                                   
                else:
                    x = 0.20

                th =  0.0

            elif val == 1:
                x  =  0.2
                th =  0.1

            else:
                x  =  0.2
                th =  0.2
        else :
            x  = 0
            th = 0 
        # 更新
        print("blue detect x,th=", x, th)
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist
        '''
        def calcTwist(self):
            # --------------------------------------------------------
            # Control Function by ImageProcessing
            # ----------------------------------------------------------
            # ①青重心座標[有:0-79、 無:-1] self.proc.blue_center  
            # ②緑重心座標[有:0-79、 無:-1] self.proc.green_center        
            # ③赤重心座標[有:0-79、 無:-1] self.proc.red_center
            # ④中央(±10度)の平均距離  [m] self.proc.center_depth
            # ①〜④の情報を使って制御を書く事

            # 青領域検出
            if self.proc.blue_center != -1:
                return self.calcTwist_center(self.proc.blue_center, self.proc.center_depth)

            # ランダム走行
            else:
                return self.calcTwist_rand()  

        # main function
        def strategy(self):
            r = rospy.Rate(1) # change speed 1fps

            target_speed = 0
            target_turn = 0
            control_speed = 0
            control_turn = 0

            while not rospy.is_shutdown():
                twist = self.calcTwist()
                #print(twist)
                self.vel_pub.publish(twist)

                r.sleep()          
        '''            

    # CSVファイルから座標を取得する関数
    def csv_data(self):
        # csvファイルをOpen
        csv_pass = os.path.dirname(__file__) + "/position_list.csv"
        csv_file = open(csv_pass, "r")
        # データ読み込み
        pos_data = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
        # 最初の一行をヘッダーとして取得
        header = next(pos_data)
        # 各行のデータを抜き出し
        for row in pos_data:
            # データ保存用のリストにcsvファイルから取得したデータを保存する
            # appendでリストに別のリストとして要素を追加する
            self.c_data.append(row)

    # "move_base/result" TopicをSubscribeしたときのコールバック関数
    # ゴール座標への到達を検知
    def result_callback(self,goal_result):
        print('result_callback')    # ★★デバッグ
        print(goal_result)          # ★★デバッグ
        if goal_result.status.status == 3:  # ゴールに到着 (★失敗時のAbort:4も対応する)
            if self.move_state == MoveState.RUN:
                self.move_state = MoveState.STOP         # 移動状態を更新：停止

    ### cmd_vel パラメータ設定＆Topic Publish関数 ※その場旋回用
    def vel_ctrl(self, line_x, line_y, ang_z):
        vel_msg = Twist()
        vel_msg.linear.x = line_x
        vel_msg.linear.y = line_y
        vel_msg.angular.z = ang_z
        self.vel_pub.publish(vel_msg)

    def orientstr_to_val(self,str_orient):
        ret_val = 0.0
        if str_orient == "up":
            ret_val = 0.0
        elif str_orient == "upright":
            ret_val = -math.pi/4
        elif str_orient == "right":
            ret_val = -math.pi/2
        elif str_orient == "downright":
            ret_val = -math.pi*3/4
        elif str_orient == "down":
            ret_val = math.pi
        elif str_orient == "downleft":
            ret_val = math.pi*3/4
        elif str_orient == "left":
            ret_val = math.pi/2
        elif str_orient == "upleft":
            ret_val = math.pi/4
        else:
            print("str_orient_error")
        return ret_val

    # "move_base_simple/goal"Topicのパラメータを設定し、Publishする関数 (引数はリスト型で渡す)
    def simple_goal_publish(self,pos_list):
        # Goal Setting
        goal = PoseStamped()
        goal.header.seq = self.goal_seq_no
        goal.header.frame_id = self.name + "/map"         # mapで座標系で指定する
        goal.header.stamp = rospy.Time.now()       # タイムスタンプは今の時間

        self.goal_seq_no += 1                      # シーケンス番号を更新

        # ** 位置座標
        goal.pose.position.x = float(pos_list[0])
        goal.pose.position.y = float(pos_list[1])
        goal.pose.position.z = 0
        # ** 回転方向
        # オイラー角をクォータニオンに変換・設定する
        # RESPECT @hotic06 オイラー角をクォータニオンに変換・設定する
        euler_val = self.orientstr_to_val(pos_list[2])
        quate = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_val)
        goal.pose.orientation.x = quate[0]
        goal.pose.orientation.y = quate[1]
        goal.pose.orientation.z = quate[2]
        goal.pose.orientation.w = quate[3]
        # debug
        print(goal)
        # 実際にTopicを配信する
        self.pub_goal.publish(goal)

    # ロボット動作のメイン処理
    def strategy(self):
        # 起動直後ウェイト
        rospy.sleep(1.0)  # 起動後、ウェイト（調整値）
        while not rospy.is_shutdown():
            if(self.greenflag == 1):
                print("snipe_enemy")
                if(self.move_state == MoveState.STOP):
                    self.greenflag = 0
            elif self.move_state == MoveState.STOP:
                pos_info = self.c_data[self.c_data_cnt]
                if pos_info[3] == "way_point":
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.simple_goal_publish(pos_info)      # 座標は後ほどCSVから取得できるように
                    self.move_state = MoveState.RUN
                    print("run")
                elif pos_info[3] == "turn_r_45":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,-math.pi/4)            # その場でターン
                    print("r_45")
                elif pos_info[3] == "turn_r_90":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,-math.pi/2)            # その場でターン
                    print("r_90")
                elif pos_info[3] == "turn_l_45":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,math.pi/4)            # その場でターン
                    print("l_45")
                elif pos_info[3] == "turn_l_90":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,math.pi/2)            # その場でターン
                    print("l_90")
                elif pos_info[3] == "turn_wait":          # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,0)                 # その場でターンしない
                    print("turn_wait")
                else:
                    self.move_state = MoveState.DEFENSE
                    pass
            else:
                pass

            print('move_state')  # ★★デバッグ
            print(self.move_state)
            rospy.sleep(1)  # 1秒Wait

    # Add ImageProcessing --- START ---
    def lidarCallback(self, data):
        self.scan = data

    # camera image call back sample
    def imageCallback(self, data):
        # comvert image topic to opencv object
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # image processing
        self.proc.imageProcess1(self.img, self.scan)
        #print('cwd=', self.proc.cwd)

        if self.proc.debug_view == 1:
            cv2.imshow("Camera", self.proc.img_div2)            
            cv2.waitKey(1)

        if self.proc.debug_view == 2:
            #cv2.imshow('rila', self.proc.rila_img)
            #cv2.imshow("div2", self.proc.img_div2)            
            #cv2.imshow("div8", self.proc.img_div8)
            #cv2.imshow("red", self.proc.red_img)
            #cv2.imshow("green", self.proc.green_img)
            #cv2.imshow("blue", self.proc.blue_img)                        
            #cv2.imshow("Camera", self.proc.img)            
            cv2.imshow("debug1", self.proc.debug1_img)                        
            cv2.waitKey(1)
        # green_index = self.proc.green_center
        # if green_index != -1:
        #     green_distance = self.proc.depth_img.item(0,green_index,0)
        # else:
        #     green_distance = 0

    # Add ImageProcessing --- END ---

    # Add
        # # 緑の検出
        # hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV_FULL)
        # bgrLower = np.array([80, 50, 50])
        # bgrUpper = np.array([110, 255, 255])
        # hsv_mask = cv2.inRange(hsv_img, bgrLower, bgrUpper)
        # rects = []
        # labels, contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        # for i in range(0, len(contours)):
        #     if len(contours[i]) > 0:

        #         rect = contours[i]
        #         x, y, w, h = cv2.boundingRect(rect)
        #         rects.append([x,y,w,h])
        # #print(rects)
        # if len(rects) != 0:
        #     if(self.greenflag ==0):
        #         self.greenflag = 1
        #         # rectsから緑全体をもってくる
        #         Max_left = 640.0
        #         Max_right = 0.0
        #         for i in rects:
        #             if (i[0] - (i[2] / 2.0)) <= Max_left:
        #                 Max_left = (i[0] - (i[2] / 2.0))
        #             if (i[0] + (i[2] / 2.0)) >= Max_right:
        #                 Max_right = (i[0] + (i[2] / 2.0))
        #         #print(Max_left)
        #         #print(Max_right)
        #         angle_left = (Max_left - (camera_width / 2.0)) * (camera_fov / camera_width)
        #         angle_right = (Max_right - (camera_width / 2.0)) * (camera_fov / camera_width)

        #         angle_adjust = (angle_left + angle_right) / 2.0
                
        #         # robot正面から何度の方向に緑の物体が存在するか計算
        #         angle = (rects[0][0] - (camera_width / 2.0)) * (camera_fov / camera_width)
        #         print("#####角度#####")
        #         print(angle)
        #         print(angle_adjust)
        #         # rectの大きさまで考慮する必要ありか？
        #         # lidarの点群からおおよその距離を算出
        #         # if angle >= 0:
        #         #     distance = self.scan.ranges[int(angle)]
        #         # else:
        #         #     distance = self.scan.ranges[int(359 + angle)]

        #         if angle_adjust >= 0:
        #             distance = self.scan.ranges[int(angle_adjust)]
        #         else:
        #             distance = self.scan.ranges[int(359 + angle_adjust)]
        #         print("#####距離#####")
        #         print(distance)
        #         # robotから見た座標値を算出　前がx軸、左がy軸
        #         # robot_x = math.cos(math.radians(angle)) * distance
        #         # robot_y = -math.sin(math.radians(angle)) * distance

        #         robot_x = math.cos(math.radians(angle_adjust)) * distance
        #         robot_y = -math.sin(math.radians(angle_adjust)) * distance

        #         print("#####x軸######")
        #         print(robot_x)
        #         print("#####y軸######")
        #         print(robot_y)
                
        #         ######要修正######
                
        #         # 地図座標系に変換
        #         #listener = tf.TransformListener()
        #         #listener.waitForTransform("/red_bot/map","/red_bot/base_footprint",rospy.Time(0),rospy.Duration(4.0))
        #         laser_point = PointStamped()
        #         laser_point.header.frame_id = "/red_bot/base_link"
        #         laser_point.header.stamp = rospy.Time(0)
        #         laser_point.point.x = robot_x
        #         laser_point.point.y = robot_y
        #         laser_point.point.z = 0.0
        #         p = PointStamped()
        #         p = self._tf_listener.transformPoint("/red_bot/map", laser_point)
        #         # 方向と位置をゴールとして指定
        #         # 一旦方向は無視して位置でデバッグ
        #         print("#####x_map#####")
        #         print(p.point.x)
        #         print("#####y_map#####")
        #         print(p.point.y)
        #         #self.setGoal(p.point.x,p.point.y,0)
        #         marker_data = Marker()
        #         marker_data.header.frame_id = "/red_bot/map"
        #         marker_data.header.stamp = rospy.Time.now()
        #         marker_data.ns = "text"
        #         marker_data.id = 0
        #         marker_data.action = Marker.ADD
        #         marker_data.type = 9
        #         marker_data.text = "Enemy"
        #         marker_data.pose.position.x = p.point.x
        #         marker_data.pose.position.y = p.point.y
        #         marker_data.pose.position.z = 0.0
        #         marker_data.pose.orientation.x = 0.0
        #         marker_data.pose.orientation.y = 0.0
        #         marker_data.pose.orientation.z = 0.0
        #         marker_data.pose.orientation.w = 0.0
        #         marker_data.color.r = 1.0
        #         marker_data.color.g = 0.0
        #         marker_data.color.b = 0.0
        #         marker_data.color.a = 1.0
        #         marker_data.scale.x = 1.0
        #         marker_data.scale.y = 0.1
        #         marker_data.scale.z = 0.1
        #         self.marker_pub.publish(marker_data)

        #         # Goal Setting
        #         goal = PoseStamped()
        #         #goal.header.seq = self.goal_seq_no
        #         goal.header.frame_id = self.name + "/map"         # mapで座標系で指定する
        #         goal.header.stamp = rospy.Time.now()       # タイムスタンプは今の時間

        #         # ** 位置座標
        #         goal.pose.position.x = p.point.x
        #         goal.pose.position.y = p.point.y
        #         goal.pose.position.z = 0
        #         # ** 回転方向
        #         # オイラー角をクォータニオンに変換・設定する
        #         # RESPECT @hotic06 オイラー角をクォータニオンに変換・設定する
        #         #euler_val = self.orientstr_to_val(pos_list[2])
        #         #quate = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_val)
        #         goal.pose.orientation.x = 0
        #         goal.pose.orientation.y = 0
        #         goal.pose.orientation.z = 0
        #         goal.pose.orientation.w = 1.0
        #         # debug
        #         print(goal)
        #         # 実際にTopicを配信する
        #         self.pub_goal.publish(goal)


    

if __name__ == "__main__":
    rospy.init_node('rirakkuma_node')
    bot = RirakkumaBot('rirakkuma')
    bot.csv_data()
    bot.strategy()