#!/usr/bin/python
# -*- coding: utf-8 -*-
from socket import *
import socket
import math
import threading
import struct
from time import sleep
import datetime
import numpy as np
# import cv2
from PIL import Image
from io import BytesIO
import os
from PIL import ImageFile

import rospy
from group6_interface.msg import AgentData, AgentsData, TargetData, TargetsData

ImageFile.LOAD_TRUNCATED_IMAGES = True
global fig_data_temp
global target_position_temp

# target_data_temp = np.zeros((2,48),dtype=int)
# from yolo import YOLO


# 设置IP地址、两个服务器端口7号
Missile_ID = int(input('请输入导弹序列号'))
server_ip = "192.168.1.102"
# server2_ip = "192.168.1.165"
server_port = 6001
client_ip = "192.168.1.101"
client_port = 9201
server_whole_port = 3333
client_whole_port = 3334
# print(Missile_ID, server_port, client_port)
print(Missile_ID)
# Image_Width = 640
# Image_Height = 480
Packet_Size = 10000
Head_Size = 58
# Channel_Num = 120
# Buffer_Size = 600  # 图片缓冲区大小
global SendOrder
global DetectImage
global IsConnected
global target_attack
target_attack = []
DetectImage = 1
SendOrder = 0
# 服务器2函数，接收消息、输出、应答
def receive_msg(rec_msg):

    # path = "N:\\pic_full\\"
    # os.mkdir(path + str(Missile_ID))
    # msg_fig = []
    # path = "C:\\Users\\yibox\\Desktop\\yolo3-pytorch-master\\1111111111"
    global fig_data_temp
    global target_position_temp
    global DetectImage
    global IsConnected
    global SendOrder
    global target_attack
    fig_data_temp = []
    # target_position_temp = []
    # sleep(1)
    while True:
        # mkfile_time = format(datetime.datetime.now().strftime('%Y%m%d%H%M%S%f'))
        # msg_data, msg_addr = rec_msg.recvfrom(Packet_Size + Head_Size)


        send_attack = []
        msg_data, msg_addr = rec_msg.recvfrom(100000)
        if len(msg_data) > Head_Size and DetectImage == 1:
            # print(msg_data[0:30])
            # print(struct.calcsize('<IIIIIII'))

            msg_head = struct.unpack_from('<IffffffffffIII', msg_data, 2)
            print(msg_head)
            # Pic_Id = msg_head[0]
            # pos_x = msg_head[1]
            # pos_y = msg_head[2]
            # pos_z = msg_head[3]
            # # print(pos_x, pos_y, pos_z)
            # angle1 = msg_head[5]
            # Pic_No = msg_head[11]
            # Pic_Size = msg_head[12]
            # Pic_Offset = msg_head[13]
            # if Pic_Id == Missile_ID:
            #     bytes_stream = BytesIO(msg_data[58:])
            #     result = Image.open(bytes_stream)
            #     capture_img = cv2.cvtColor(np.asarray(result), cv2.COLOR_RGB2BGR)
            #     frame = Image.fromarray(np.uint8(capture_img))
            #     frame = np.array(
            #         yolo.detect_image(frame, crop=False, count=False))


            #     cv2.imshow("capture_img0", frame)
            #     cv2.waitKey(1)


            # print(Pic_Id, Pic_No, pos_x, pos_z)
            # 窗口1
            # 筛选特定的信道00
            # if Pic_Id == Missile_ID + 10000:
            #     bytes_stream = BytesIO(msg_data[58:])
            #     result = Image.open(bytes_stream)
            #
            #     capture_img = cv2.cvtColor(np.asarray(result), cv2.COLOR_RGB2BGR)
            #     kernel = np.ones((2, 2), np.uint8)
            #     lower = np.array([180])
            #     upper = np.array([255])
            #     gray_img = cv2.cvtColor(capture_img, cv2.COLOR_BGR2GRAY)
            #     mask = cv2.inRange(gray_img, lowerb=lower, upperb=upper)
            #     mask = cv2.dilate(mask, kernel, iterations=5)
            #     contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #     # print(type(contours))
            #     fig_data_temp.append(contours)
            #     # print(len(fig_data_temp))
            # if Pic_Id == Missile_ID:
            #     if len(fig_data_temp) >= 2:
            #         bytes_stream = BytesIO(msg_data[58:])
            #         try:
            #             result = Image.open(bytes_stream)
            #             capture_img = cv2.cvtColor(np.asarray(result), cv2.COLOR_RGB2BGR)
            #         except:
            #             print('Open Error! Try again!')
            #             continue
            #         # print(fig_data_temp[-1])
            #         bb = []
            #         for c in fig_data_temp[-1]:
            #             aa = []
            #             xx, yy, ww, hh = cv2.boundingRect(c)
            #             xx = int(xx * 1.125)
            #             yy = int(yy * 1.125)
            #             ww = int(ww * 1.125)
            #             hh = int(hh * 1.125)
            #             # print(xx, yy, ww, hh)
            #             cv2.rectangle(capture_img, (xx, yy), (xx + ww, yy + hh), (0, 255, 0), 2)
            #             midx = xx + ww / 2
            #             midy = yy + hh / 2
            #             resx = ((midx - 360) * 0.22222222)  # 370   219*164
            #             resy = ((-midy + 270) * 0.22222222)  # 270   160*120    279
            #             sin = math.sin(math.radians(angle1))
            #             cos = math.cos(math.radians(angle1))
            #             basexx = resx * cos + resy * sin + pos_x
            #             baseyy = resy * cos - resx * sin + pos_z
            #             # print(basexx, baseyy)
            #             # aa.append((basexx, baseyy))
            #             print(target_position_temp)
            #
            #             for k in range(len(target_position_temp[0])):
            #                 a = math.sqrt(
            #                     ((baseyy - float(target_position_temp[3][k])) * (
            #                                 baseyy - float(target_position_temp[3][k]))) + (
            #                             (basexx - float(target_position_temp[2][k])) * (
            #                                 basexx - float(target_position_temp[2][k]))))
            #
            #                 aa.append(a)#从48个目标中选出最符合的目标
            #                 # print(aa)
            #             min = np.argmin(aa)
            #             # print(min)
            #             #弹与目标的距离
            #             # 目标优先级，攻击那个目标（先以距离优先，优先攻击距离巡飞弹近的目标）
            #             b = math.sqrt(
            #                 ((pos_z - float(target_position_temp[3][min])) * (
            #                         pos_z - float(target_position_temp[3][min]))) + (
            #                         (pos_x - float(target_position_temp[2][min])) * (
            #                         pos_x - float(target_position_temp[2][min]))))
            #             bb.append(b)
            #             send_attack.append(target_position_temp[0][min])
            #         # print(bb)
            #         if bb:
            #             L_min = np.argmin(bb)
            #         # print(L_min)
            #             target_attack = send_attack[L_min]
            #             print(target_attack)
            #         cv2.imshow("capture_img0", capture_img)
            #         cv2.waitKey(1)

def receive_msg_whole(rec_msg_whole):


    # udp_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # local_addr = ("127.0.0.2", 9112)
    # udp_receive.bind(local_addr)


    global fig_data_temp
    global target_position_temp
    global DetectImage
    global target_attack
    global IsConnected

    Missile_pub = rospy.Publisher("/MissileData", AgentsData, queue_size=1)
    Target_pub = rospy.Publisher("/TargetData", TargetsData, queue_size=1)

    # sleep(1)
    while True:
        msg_dataself, msg_addrself = rec_msg_whole.recvfrom(100000)
        if len(msg_dataself) < Head_Size:
            continue

        msg_head = struct.unpack_from('<IIIIIIIII', msg_dataself, 2)
        print(msg_head)

        target_data_temp = np.zeros((4, int((len(msg_dataself) - 38 - 56 * msg_head[-1]) / 52)),
                                    dtype=int)
        
        agents_msg = AgentsData()
        for i in range(msg_head[-1]):
            agent_msg = AgentData()
            result1 = struct.unpack_from('<IIIfffIIfff', msg_dataself, 38 + 56 * i)
            agent_msg.missile_camp = result1[0]
            agent_msg.missile_type = result1[1]
            agent_msg.missile_id = result1[2]
            agent_msg.x = result1[3]
            agent_msg.y = result1[4]
            agent_msg.z = result1[5]
            agent_msg.rot_x = result1[6]
            agent_msg.rot_y = result1[7]
            agent_msg.rot_z = result1[8]
            agent_msg.uav_weapon_num = result1[9]
            agent_msg.uav_fuel_oil_num = result1[10]
            agents_msg.agents_data.append(agent_msg)
        rospy.loginfo(agents_msg)
            # print(result1)

        targets_msg = TargetsData()
        for j in range(int((len(msg_dataself) - 38 - 56 * msg_head[-1]) / 52)):
            target_msg = TargetData()
            result2 = struct.unpack_from('<IIIIIfffIIfff', msg_dataself, 38 + 56 * msg_head[-1] + 52 * j)
            target_msg.target_camp = result2[0]
            target_msg.target_type = result2[1]
            target_msg.target_id = result2[2]
            target_msg.destroy_count = result2[3]
            target_msg.remain_misslile_count = result2[4]
            target_msg.x = result2[5]
            target_msg.y = result2[6]
            target_msg.z = result2[7]
            target_msg.rot_x = result2[8]
            target_msg.rot_y = result2[9]
            target_msg.rot_z = result2[10]
            target_msg.target_weapon_num = result2[11]
            target_msg.target_fuel_oil_num = result2[12]
            targets_msg.targets_data.append(target_msg)
            
            # print(result2)

            target_data_temp[0][j] = result2[2]
            target_data_temp[1][j] = result2[3] 
            target_data_temp[2][j] = result2[5]
            target_data_temp[3][j] = result2[7]
        target_position_temp = target_data_temp
        rospy.loginfo(targets_msg)

        Missile_pub.publish(agents_msg)
        Target_pub.publish(targets_msg)
        rospy.sleep(0.5)


#专门再设一个标志位，开始图像目标检测和暂停目标检测
# 单独再开一个线程，专门有一个函数，设置一个标志位 0不发送消息 1攻击指令 2取消攻击
# 对巡飞弹 0是巡飞盘旋 1是攻击 2是撤销命令
def send_msg_order(send_msg_order):
    global SendOrder
    global DetectImage
    global target_attack
    global IsConnected
    global target_position_temp
    sleep(0.5)
    while True:
        print(target_attack)
        if target_attack:
            print('target_attack is not none')
        #     SendOrder = 1
        #在飞临攻击目标的过程中，目标已被摧毁的情况

            count = 0
            for i in range(len(target_position_temp[0])):
                if target_position_temp[0][i] == target_attack:
                    count = count + 1
            # # print(9999999999999999999999999)
            if count != 0:
                SendOrder = 1
            else:
                SendOrder = 2
        else:
            SendOrder = 0


        if SendOrder == 0:
            pass
            # send_msg_order.sendto((Missile_ID + "0").encode("utf-8"), (server_ip, server_port))
            # DetectImage = 1
            # print(7777777777777777777)
        if SendOrder == 1:
            # pass
            # if target_attack:
            DetectImage = 0  # 发现目标之后，关闭图像检测
            # print((str(Missile_ID) + "," + str(target_attack) + "," + "1"))
            send_msg_order.sendto(b'\x01\x00' + (str(Missile_ID) + "," + str(target_attack) + "," + "1").encode("utf-8"), (server_ip, server_whole_port))
            print(f"消息{Missile_ID},{target_attack},1已发送到 {server_ip}:{server_whole_port}")
            # DetectImage = 0
            # print(88888888888888888888)
        if SendOrder == 2: #解除对指定目标的攻击指令
            send_msg_order.sendto(
                b'\x01\x00' + (str(Missile_ID) + "," + str(target_attack) + "," + "2").encode("utf-8"),
                (server_ip, server_whole_port))
            print(f"消息{Missile_ID},{target_attack},2已发送到 {server_ip}:{server_whole_port}")
            DetectImage = 1
            # send_msg_order.sendto((Missile_ID + "攻击的目标序号" + "2").encode("utf-8"), (server_ip, server_port))
            # DetectImage = 1
            # print(99999999999999999999)

def broadcast_msg(bro_msg):
    global IsConnected
    while True:
        bro_msg.sendto("0".encode("utf-8"), (server_ip, server_port))
        bro_msg.sendto("0".encode("utf-8"), (server_ip, server_whole_port))
        sleep(2)

# 主函数 创建服务器、绑定端口、创建运行两个进程、调用上面两个函数
def main():
    global DetectImage
    global IsConnected
    global fig_data_temp
    global target_position_temp
    global target_attack

    # 创建套接字
    rec_msg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rec_msg_whole = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bro_msg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bro_msg_order = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    bro_msg.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    bro_msg_order.setsockopt(SOL_SOCKET, SO_BROADCAST, 20)
    rec_msg.setsockopt(SOL_SOCKET, SO_RCVBUF, 2 ** 28)
    # rec_msg_whole.setsockopt(SOL_SOCKET, SO_RCVBUF, 2 ** 28)

    local_addr = ("192.168.1.101", 3334)
    rec_msg_whole.bind(local_addr)


    # 绑定地址端口
    rec_msg.bind((client_ip, client_port))
    # rec_msg_whole.bind((client_ip, client_whole_port))

    # 创建进程
    t_recmsg = threading.Thread(target=receive_msg, args=(rec_msg,))
    t_recmsg_whole = threading.Thread(target=receive_msg_whole, args=(rec_msg_whole,))
    t_bromsg = threading.Thread(target=broadcast_msg, args=(bro_msg,))
    t_bromsg_order = threading.Thread(target=send_msg_order, args=(bro_msg_order,))

    # 开始进程
    t_recmsg_whole.start()
    t_bromsg.start()
    # t_bromsg_order.start()
    # t_recmsg.start()


if __name__ == '__main__':
    # yolo = YOLO()
    rospy.init_node("companyToResearchGroup6", anonymous=True)
    main()