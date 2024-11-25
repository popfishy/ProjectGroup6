import numpy as np
import networkx as nx
import rospy
import sys
import time

np.set_printoptions(threshold=np.inf)
from scipy import integrate
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
import math

tra_frame = 0


class GVF_ode:
    # 初始化输入无人机id列表，无人机数量，初始坐标
    def __init__(self, plane_id, uav_num, x_coords, y_coords, x_init, y_init, z_init):
        self.id = plane_id
        self.x_coords = x_coords
        self.y_coords = y_coords

        self.uav_num = int(uav_num)
        self.is_finish_pub = False

        self.global_paths = []

        # params
        self.n = 3
        self.alpha, self.beta, self.k1, self.k2, self.k3, self.kc = 1, 1, 1, 1, 1, 1
        self.tmp = np.linspace(0, 2 * np.pi, int(uav_num) + 1)

        # these delta make a square
        self.delta1 = np.array([x_coords])
        self.delta2 = np.array([y_coords])
        self.manual_v = np.array([[0, 0, 0, 1, 1]])

        # graph
        self.A = np.roll(np.eye(int(uav_num), int(uav_num)), 1, axis=1)
        self.A = self.A + self.A.T
        self.L = 2 * np.eye(int(uav_num), int(uav_num)) - self.A
        self.G = nx.from_numpy_array(self.A)

        # ODE settomgs
        self.tspan = np.array([0, 40])
        self.x1_init = np.array(x_init).reshape(1, -1)
        self.x2_init = np.array(y_init).reshape(1, -1)
        self.x3_init = np.array(z_init).reshape(1, -1)
        self.w1_init = self.delta1  # if initial w values have the desired distances, then the performanc is better.
        self.w2_init = self.delta2  # if initial w values have the desired distances, then the performanc is better.

        # p_init
        self.p_init = []
        for i in range(int(uav_num)):
            self.p_init.append(self.x1_init[0][i])
            self.p_init.append(self.x2_init[0][i])
            self.p_init.append(self.x3_init[0][i])
            self.p_init.append(self.w1_init[0][i])
            self.p_init.append(self.w2_init[0][i])

        ##### trajectory(x y z t)
        self.trajectory_list = []
        self.orientation_list = []
        self.p_final = None

        # 路径计数用
        self.cnt = -1

    #
    def update_waypoint(self, path):
        # self.trajectory_list = [[0, 0, 0, 0], [1000, 1000, 50, 50], [1000, 2000, 100, 100], [0, 3000, 50, 50]]
        self.trajectory_list = path
        self.orientation_list = [0] * (len(self.trajectory_list) - 1)

    ### system equations
    def cal_covf(self, pos_all, n, uav_num, A, L, delta1, delta2):
        pos_all_array = np.array(pos_all)
        tmp = pos_all_array.reshape(uav_num, -1)
        tmp = np.transpose(tmp)

        w1 = tmp[n, :]
        w2 = tmp[n + 1, :]

        hatw1 = (w1 - delta1).T
        hatw2 = (w2 - delta2).T

        vec1 = np.zeros((n + 2, 1))
        vec1[n] = 1
        vec1[n + 1] = 0

        vec2 = np.zeros((n + 2, 1))
        vec2[n] = 0
        vec2[n + 1] = 1

        covf1 = np.kron(-np.dot(L, hatw1), vec1)
        covf2 = np.kron(-np.dot(L, hatw2), vec2)

        covf_all = covf1 + covf2
        return covf_all

    def cal_pfvf(self, pos_all, n, uav_num, manual_v):
        len_pos = len(pos_all)
        m_v1 = manual_v[0][0]
        m_v2 = manual_v[0][1]
        m_v3 = manual_v[0][2]
        # m_v3 = self.trajectory_list[tra_frame + 1][2]
        # m_v4 = manual_v[0][3]
        m_v4 = (self.trajectory_list[tra_frame + 1][1] - self.trajectory_list[tra_frame][1]) / self.trajectory_list[
            tra_frame + 1
        ][3]
        # m_v5 = manual_v[0][4]
        m_v5 = (
            -(self.trajectory_list[tra_frame + 1][0] - self.trajectory_list[tra_frame][0])
            / self.trajectory_list[tra_frame + 1][3]
        )

        if uav_num != len_pos // (n + 2):
            print("Error! N is not correct!")
            return None

        pfvf_all = np.zeros((len_pos, 1))
        e_all = np.zeros((1, n * uav_num))

        for i in range(uav_num):
            j = i * (n + 2)
            l = i * n
            x1 = pos_all[j]
            x2 = pos_all[j + 1]
            x3 = pos_all[j + 2]
            w1 = pos_all[j + 3]
            w2 = pos_all[j + 4]
            scaled_w1 = self.beta * w1
            scaled_w2 = self.beta * w2

            # only need to change this part
            f1w = scaled_w1
            f2w = scaled_w2
            f3w = self.trajectory_list[tra_frame + 1][2]

            # only need to change this part
            phi1 = self.alpha * (x1 - f1w)
            phi2 = self.alpha * (x2 - f2w)
            phi3 = self.alpha * (x3 - f3w)
            sign = (-1) ** n
            v = np.array(
                [
                    sign * m_v5 - self.k1 * phi1,
                    sign * (-m_v4) - self.k2 * phi2,
                    -self.k3 * phi3,
                    sign * m_v5 + self.k1 * phi1,
                    sign * -m_v4 + self.k2 * phi2,
                ]
            )

            pfvf_all[j : j + n + 2, 0] = v
            phi = np.array([phi1, phi2, phi3])
            e_all[0, l : l + n] = phi

        return pfvf_all, e_all

    def multipf(self, t, p, n, uav_num, A, L, delta1, delta2, manual_v):
        [pfvf, e_all] = self.cal_pfvf(p, n, int(uav_num), manual_v)
        copf = self.cal_covf(p, n, int(uav_num), A, L, delta1, delta2)
        vf = pfvf + self.kc * copf
        dxidt = vf
        return dxidt, e_all

    def calculate_path(self, timestep=None):
        global tra_frame
        for tra_frame in range(len(self.trajectory_list) - 1):
            ### ode45
            t0, tf = 0, self.trajectory_list[tra_frame + 1][3]
            # numpoints计算 距离/速度*HZ
            numpoints = int(
                math.sqrt(
                    (self.trajectory_list[tra_frame + 1][0] - self.trajectory_list[tra_frame][0]) ** 2
                    + (self.trajectory_list[tra_frame + 1][1] - self.trajectory_list[tra_frame][1]) ** 2
                )
                / (41.33)
                * 30
            )
            # print("numpoints:", numpoints)
            if numpoints == 0:
                print(self.trajectory_list[tra_frame + 1][0], self.trajectory_list[tra_frame][0])
                print(self.trajectory_list[tra_frame + 1][1], self.trajectory_list[tra_frame][1])

            if (self.trajectory_list[tra_frame + 1][0] - self.trajectory_list[tra_frame][0]) != 0:
                self.orientation_list[tra_frame] = -math.atan(
                    (self.trajectory_list[tra_frame + 1][1] - self.trajectory_list[tra_frame][1])
                    / (self.trajectory_list[tra_frame + 1][0] - self.trajectory_list[tra_frame][0])
                )
            else:
                # TODO： 这个角度是干啥的？？
                self.orientation_list[tra_frame] = math.pi / 2

            t = np.linspace(t0, tf, numpoints)
            p_init = self.p_init
            if tra_frame > 0:
                p_init = self.p_final.T
                p_init = p_init.squeeze()
            p = np.zeros((len(t), len(p_init)))
            p[0, :] = p_init

            r = integrate.ode(self.multipf).set_integrator("dopri5")
            r.set_initial_value(p_init, t0)
            r.set_f_params(self.n, int(self.uav_num), self.A, self.L, self.delta1, self.delta2, self.manual_v)
            for i in range(1, len(t)):
                p[i, :] = r.integrate(t[i])
                if not r.successful():
                    raise RuntimeError("Could not integrate")

            self.p_final = p[numpoints - 1 :]
            # p_array = np.array(p)
            # for i in range(self.uav_num):
            #     # 使用切片快速获取 x, y, z
            #     x = p_array[:, i * 5] + self.trajectory_list[0][0]  # 获取所有时间步的 x 值
            #     y = p_array[:, i * 5 + 1] + self.trajectory_list[0][1]  # 获取所有时间步的 y 值
            #     z = p_array[:, i * 5 + 2]  # 获取所有时间步的 z 值

            # for i in range(len(x)):

            #     pass

            self.global_paths.append(p)
