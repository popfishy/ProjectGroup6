import socket
import struct
import time
import threading
import rospy
from group6_interface.msg import AgentData, AgentsData, TargetData, TargetsData, RegionData, RegionsData, ControlData, ControlsData, TrajectoryData


class UavSimDataSend():

    def __init__(self) :
        self.UBUNTU_IP = "192.168.1.101" 
        self.Sim_IP = "192.168.1.102"   # 仿真系统IP
        self.UAV_PORT = 12380 
        self.control_data_sub = rospy.Subscriber("/control_data", ControlsData, self.control_callback)

    def control_callback(self, msg: ControlsData):
        uav_num = len(msg.controls_data)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        for i in range(uav_num):
            missile_id = msg.controls_data[i].missile_id
            x = msg.controls_data[i].x
            z = msg.controls_data[i].z
            y = msg.controls_data[i].y
            rot_x = msg.controls_data[i].rot_x
            rot_y = msg.controls_data[i].rot_y
            rot_z = msg.controls_data[i].rot_z

            # 构建二进制数据包
            message = f"MissileTrans,{missile_id},{x},{z},{y},{0},{0},{0},{1}"
            packet = message.encode('utf-8')
            sock.sendto(packet, (self.Sim_IP, self.UAV_PORT))
            # rospy.loginfo(f"Sent Control_UAV command for UAV {missile_id}.")
        # rospy.loginfo(msg.controls_data)
        # self.control_data_sub.unregister()
        
    def send_uav_data(self, command, struct_data_list):
        # 创建UDP套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            # 打包命令和数量
            packed_command = struct.pack('i', command)
            packed_count = struct.pack('i', len(struct_data_list))

            # 打包结构体数据
            packed_structs = b''
            for struct_data in struct_data_list:
                packed_structs += struct.pack('i3f4f', *struct_data)

            # 拼接所有数据
            packed_data = packed_command + packed_count + packed_structs
            print(packed_data)
            # 发送数据
            sock.sendto(packed_data, (self.Sim_IP, self.UAV_PORT))
            print(f"发送数据: 命令={command}, 结构体数量={len(struct_data_list)}")

        finally:
            # 关闭套接字
            sock.close()

if __name__ == "__main__":
    rospy.init_node("uav_sim_data_send")
    data_send = UavSimDataSend()
    rospy.spin()