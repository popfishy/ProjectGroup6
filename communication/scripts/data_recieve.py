import socket
import struct
import time
import threading
import rospy
from group6_interface.msg import AgentData, AgentsData, TargetData, TargetsData, RegionData, RegionsData
from scipy.spatial.transform import Rotation as R
import numpy as np
import random


class DateType:
    UAV: int = 0
    TARGET: int = 1
    REGION: int = 2


class UavSimDataRecieve:
    UBUNTU_IP = "192.168.1.101"
    Sim_IP = "192.168.1.102"  # 仿真系统IP
    UAV_PORT = 12380
    SUBSCRIPTION_CMD = struct.pack("<I", 1)
    UNSUBSCRIPTION_CMD = struct.pack("<I", 2)
    UAVSEND_CMD = 101
    TARGET_CMD = 102

    missile_pub = rospy.Publisher("/missile_data", AgentsData, queue_size=1)
    target_pub = rospy.Publisher("/target_data", TargetsData, queue_size=1)
    region_pub = rospy.Publisher("/region_data", RegionsData, queue_size=1)

    def __init__(self) -> None:
        pass

    def init_uavs(self):
        # 创建 UDP 套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 发送订阅命令
        sock.sendto(self.SUBSCRIPTION_CMD, (self.Sim_IP, self.UAV_PORT))
        print("Sent subscription command.")

        data_type = -1
        while data_type != DateType.UAV:
            data, addr = sock.recvfrom(100000)  # 设置接收缓冲区
            data_type = struct.unpack("<I", data[0:4])[0]

        packed_data_list = []
        cnt = struct.unpack("<I", data[4:8])[0]  # 获取数量
        side_length = 500  # 正方形边长
        num_rows = int(np.sqrt(cnt))
        num_cols = int((cnt + num_rows - 1) // num_rows)  # 向上取整

        # for i in range(cnt):
        #     start_index = 8 + i * 44  # 每个巡飞弹数据块的起始位置
        #     unpacked_data = struct.unpack("<1I9f1I", data[start_index : start_index + 44])
        #     id = unpacked_data[0]
        #     x, y, z = unpacked_data[3:6]
        #     rx, ry, rz, rw = unpacked_data[6:10]
        #     packed_data = (id, x, y, z, rx, ry, rz, rw)
        #     packed_data_list.append(packed_data)
        # self.send_uav_data(self.UAVSEND_CMD, packed_data_list)
        # sock.sendto(self.UNSUBSCRIPTION_CMD, (self.Sim_IP, self.UAV_PORT))

        for i in range(cnt):
            start_index = 8 + i * 44  # 每个巡飞弹数据块的起始位置
            unpacked_data = struct.unpack("<1I9f1I", data[start_index : start_index + 44])
            id = unpacked_data[0]
            row = i // num_cols
            col = i % num_cols
            x = 4500 + col * (side_length / (num_cols - 1))
            z = 750 - row * (side_length / (num_rows - 1))
            y = unpacked_data[4]
            rx, ry, rz, rw = unpacked_data[6:10]
            packed_data = (id, x, y, z, rx, ry, rz, rw)
            packed_data_list.append(packed_data)
        self.send_uav_data(self.UAVSEND_CMD, packed_data_list)
        sock.sendto(self.UNSUBSCRIPTION_CMD, (self.Sim_IP, self.UAV_PORT))

    def init_targets(self):
        # 创建 UDP 套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 发送订阅命令
        sock.sendto(self.SUBSCRIPTION_CMD, (self.Sim_IP, self.UAV_PORT))
        print("Sent subscription command.")

        data_type = -1
        while data_type != DateType.TARGET:
            data, addr = sock.recvfrom(100000)  # 设置接收缓冲区
            data_type = struct.unpack("<I", data[0:4])[0]

        packed_data_list = []
        cnt = struct.unpack("<I", data[4:8])[0]  # 获取数量

        target_position = self.init_target_position(cnt)

        for i in range(cnt):
            start_index = 8 + i * 44  # 每个目标的起始位置
            unpacked_data = struct.unpack("<1I9f1I", data[start_index : start_index + 44])
            id = unpacked_data[0]
            x, y, z = target_position[i + 1][0], unpacked_data[4], target_position[i + 1][1]
            rx, ry, rz, rw = unpacked_data[6:10]
            packed_data = (id, x, y, z, rx, ry, rz, rw)
            packed_data_list.append(packed_data)
        self.send_uav_data(self.TARGET_CMD, packed_data_list)
        sock.sendto(self.UNSUBSCRIPTION_CMD, (self.Sim_IP, self.UAV_PORT))

    def get_uav_data(self):
        # 创建 UDP 套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 发送订阅命令
        sock.sendto(self.SUBSCRIPTION_CMD, (self.Sim_IP, self.UAV_PORT))
        print("Sent subscription command.")
        while True:
            # 接收数据
            data, addr = sock.recvfrom(100000)  # 设置接收缓冲区
            data_type = struct.unpack("<I", data[0:4])[0]
            if data_type == DateType.UAV:
                agents_msg = AgentsData()
                cnt = struct.unpack("<I", data[4:8])[0]  # 获取数量
                for i in range(cnt):
                    start_index = 8 + i * 44  # 每个巡飞弹数据块的起始位置
                    unpacked_data = struct.unpack("<1I9f1I", data[start_index : start_index + 44])
                    agent_msg = AgentData()
                    agent_msg.is_destroy = unpacked_data[10]
                    if not agent_msg.is_destroy:
                        agent_msg.missile_id = unpacked_data[0]
                        agent_msg.missile_flight_speed = unpacked_data[1]
                        agent_msg.missile_distance_flown = unpacked_data[2]
                        agent_msg.x, agent_msg.y, agent_msg.z = unpacked_data[3:6]  # 位置 x, y, z
                        agent_msg.rot_x, agent_msg.rot_y, agent_msg.rot_z, agent_msg.rot_w = unpacked_data[6:10]
                        agents_msg.agents_data.append(agent_msg)
                self.missile_pub.publish(agents_msg)

            elif data_type == DateType.TARGET:
                targets_msg = TargetsData()
                cnt = struct.unpack("<I", data[4:8])[0]  # 获取数量
                for i in range(cnt):
                    start_index = 8 + i * 44  # 每个巡飞弹数据块的起始位置
                    unpacked_data = struct.unpack("<2I8f1I", data[start_index : start_index + 44])

                    target_msg = TargetData()
                    target_msg.is_destroy = unpacked_data[10]
                    target_msg.target_id = unpacked_data[0]
                    target_msg.target_type = unpacked_data[1]
                    target_msg.target_move_speed = unpacked_data[2]
                    target_msg.x, target_msg.y, target_msg.z = unpacked_data[3:6]  # 位置 x, y, z
                    target_msg.rot_x, target_msg.rot_y, target_msg.rot_z, target_msg.rot_w = unpacked_data[6:10]
                    target_msg.is_destroy = unpacked_data[10]
                    targets_msg.targets_data.append(target_msg)
                self.target_pub.publish(targets_msg)

            elif data_type == DateType.REGION:
                regions_msg = RegionsData()
                cnt = struct.unpack("<I", data[4:8])[0]  # 获取数量
                for i in range(cnt):
                    start_index = 8 + i * 36
                    unpacked_data = struct.unpack("<12f", data[start_index : start_index + 48])

                    region_msg = RegionData()
                    region_msg.position1.x, region_msg.position1.y, region_msg.position1.z = unpacked_data[0:3]
                    region_msg.position2.x, region_msg.position2.y, region_msg.position2.z = unpacked_data[3:6]
                    region_msg.position3.x, region_msg.position3.y, region_msg.position3.z = unpacked_data[6:9]
                    region_msg.position4.x, region_msg.position4.y, region_msg.position4.z = unpacked_data[9:12]
                    regions_msg.regions_data.append(region_msg)
                self.region_pub.publish(regions_msg)
            rospy.sleep(0.005)

    def send_uav_data(self, command, struct_data_list):
        # 创建UDP套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            # 打包命令和数量
            packed_command = struct.pack("i", command)
            packed_count = struct.pack("i", len(struct_data_list))

            # 打包结构体数据
            packed_structs = b""
            for struct_data in struct_data_list:
                packed_structs += struct.pack("i3f4f", *struct_data)

            # 拼接所有数据
            packed_data = packed_command + packed_count + packed_structs
            # 发送数据
            sock.sendto(packed_data, (self.Sim_IP, self.UAV_PORT))
            print(f"发送数据: 命令={command}, 结构体数量={len(struct_data_list)}")

        finally:
            # 关闭套接字
            sock.close()

    def init_target_position(self, cnt):
        # 设置随机数种子（可选）
        random.seed(42)
        # 假设60个值是从1到60的整数
        values = list(range(1, cnt + 1))
        # 随机打乱值
        random.shuffle(values)

        # 计算每个子列表的长度
        sublist_lengths = [
            int(cnt * 1 / 10),
            int(cnt * 1 / 10),
            int(cnt * 6 / 10),
            cnt - int(cnt * 1 / 10) - int(cnt * 1 / 10) - int(cnt * 6 / 10),
        ]

        # 将值分成4个子列表，每个子列表15个值
        sublists = []
        start_index = 0
        for length in sublist_lengths:
            sublists.append(values[start_index : start_index + length])
            start_index += length

        # 定义四个长方形区域的坐标范围
        rectangles = [
            {"name": "Rectangle 1", "x_range": (8081.0, 9519.0), "y_range": (2802.0, 3752.0)},
            {"name": "Rectangle 2", "x_range": (6007, 7141), "y_range": (1840, 2908)},
            {"name": "Rectangle 3", "x_range": (7352, 8882), "y_range": (1579, 2745)},
            {"name": "Rectangle 4", "x_range": (7352, 9145), "y_range": (120, 1532)},
        ]
        # 用于存储每个区域的坐标
        coordinates = {rect["name"]: set() for rect in rectangles}

        # 用于存储最终结果的字典
        result = {}

        # 将值分配到每个区域，并生成随机坐标
        for i, sublist in enumerate(sublists):
            for value in sublist:
                while True:
                    x = random.uniform(rectangles[i]["x_range"][0], rectangles[i]["x_range"][1])
                    y = random.uniform(rectangles[i]["y_range"][0], rectangles[i]["y_range"][1])
                    coord = (round(x, 2), round(y, 2))
                    if coord not in coordinates[rectangles[i]["name"]]:
                        coordinates[rectangles[i]["name"]].add(coord)
                        result[value] = coord
                        break
                # print(f"{rectangles[i]['name']}: Value {value} at ({x:.2f}, {y:.2f})")

        # 返回结果字典
        return result


def euler_to_quaternion(r, p, y):
    """
    将欧拉角 (roll, pitch, yaw) 转换为四元数。

    参数:
    r (float): 滚转角 (roll) (绕 x 轴旋转)
    p (float): 俯仰角 (pitch) (绕 y 轴旋转)
    y (float): 偏航角 (yaw) (绕 z 轴旋转)

    返回:
    np.array: 对应的四元数 (x, y, z, w)
    """
    # 创建旋转对象
    rotation = R.from_euler("xyz", [r, p, y])
    # 转换为四元数
    quaternion = rotation.as_quat()  # 返回 (x, y, z, w) 格式
    return quaternion


if __name__ == "__main__":
    rospy.init_node("uav_sim_data_recieve")

    data_recieve = UavSimDataRecieve()
    data_recieve.init_uavs()
    data_recieve.init_targets()
    t_recmsg = threading.Thread(target=data_recieve.get_uav_data)
    t_recmsg.start()

    # 绑定其他端口
    # t_recimg = threading.Thread(target = data_recieve.get_image_data)
    # t_recimg.start()
