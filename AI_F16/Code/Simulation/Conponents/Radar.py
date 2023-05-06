import numpy as np
from random import random
import rticonnextdds_connector as rti
from os import path 
from math import cos, sin, atan, sqrt, radians
# from vpython import *

# 目前均在雷达坐标系下完成扫描, 需要转换至地面坐标系 
# 当前实现的是单目标跟踪扫描, 无法实现多目标跟踪

class Radar:
    def __init__(self):
        
        # 雷达最大探测距离[m]
        self.max_detected_distance = 50000    
        # 雷达最小探测距离[m]
        self.min_detected_distance = 500
        # 雷达扫描角速度[rad/s]
        self.Vscan = radians(100)
        # 雷达发现目标后截获该目标的概率
        self.prob = 0.8

        # 雷达的搜索模式，0为关闭，1为超视距模式， 2为视距内模式
        self.SearchMode = 0
        # 雷达的扫描模式，0为正常扫描，1为锁定
        self.ScanMode = 0

        # 雷达方位扫描范围, 根据搜索模式[10, 30, 60, 120]选其一[°]
        self.AziRange = radians(60)
        # 雷达俯仰扫描范围, 根据搜索模式[4.5, 60]选其一[°]
        self.EleRange = radians(4.5)
        # 雷达瞬时全视场[°]
        self.Alf = 2*self.AziRange*self.EleRange

        # 雷达当前方位角
        self.Azi = -self.AziRange
        # 雷达当前俯仰角
        self.Ele = 0

        # 雷达扫描行数，偶数
        self.Hlines = 2
        # 雷达波束张角[°]
        self.filed = radians(10)
        
        # 雷达行扫描耗时
        self.azi_time = 2*self.AziRange/self.Vscan
        # 雷达列扫描耗时
        self.ele_time = self.EleRange/self.Vscan
        # 雷达扫描周期
        self.T = self.Hlines*self.azi_time + 2*(self.Hlines - 1)*self.ele_time

        # 雷达波束轴坐标
        self.AxisPos = [self.max_detected_distance*sin(self.AziRange), self.max_detected_distance*cos(self.AziRange), 0]

        # 雷达探测到的目标信息
        self.TarInfo = None

        self.radius = self.max_detected_distance
        self.azi_1 = self.Azi - self.filed/2
        self.azi_2 = self.Azi + self.filed/2

        # DDS
        self.file_path = path.dirname(path.realpath(__file__))
        self.pub_connector = rti.Connector("MyParticipantLibrary::MyPubParticipant", self.file_path + "\ShapeExample.xml")
        self.sub_connector = rti.Connector("MyParticipantLibrary::MySubParticipant", self.file_path + "\ShapeExample.xml")

        self.output = self.pub_connector.get_output("MyPublisher::MyRadarWriter")
        # self.output.instance.set_dictionary({"scan_mode":self.ScanMode, "search_mode":self.SearchMode, "target_position":self.TarInfo})
        # self.output.write()


    # 计算目标与本机距离信息
    def distance(self, my_aircraft_position, target_position):
        # 地面坐标系下的双机距离信息
        Rx = float(target_position[0] - my_aircraft_position[0])
        Ry = float(target_position[1] - my_aircraft_position[1])
        Rz = float(target_position[2] - my_aircraft_position[2])
        dist = sqrt(Rx**2 + Ry**2 + Rz**2)
        return [dist, Rx, Ry, Rz]

  
    # 扫描主函数
    def scan(self, t):
        # 订阅载机信息
        input = self.sub_connector.get_input("MySubscriber::MyAircraftReader")
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            base_position = [data['state'][i] for i in [10,9,11]]
            base_velocity = data['state'][0]
            # print('得到飞机坐标{}, 速度{}'.format(base_position, base_velocity))
        # 订阅导弹信息
        input = self.sub_connector.get_input("MySubscriber::MyMissileReader")
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            tgt_position = data['position']
            tgt_velocity = data['velocity']

        dist = self.distance(base_position, tgt_position)

        # 搜索模式
        if dist[0] > 15000:
            # 超视距
            self.SearchMode = 1
            self.AziRange = radians(60)
            self.EleRange = radians(4.5)
        else:
            # 视距内
            self.SearchMode = 2
            self.AziRange = radians(10)
            self.EleRange = radians(60)

        # 计算视线角
        q_z = atan(sqrt(dist[1]**2 + dist[2]**2)/dist[3]) if dist[3]!=0 else 0
        q_x = atan(dist[2]/dist[1]) if dist[1]!=0 else np.pi/2
        
        # 判断目标是否被雷达扫描到
        if (abs(self.Azi - (np.pi/2 - q_x)) <= self.filed or abs(self.Ele - (np.pi/2 - q_z)) <= self.filed) and dist[0] < self.max_detected_distance:
        # if  dist[0] < self.max_detected_distance:    
            # 生成一个随机数作为截获判断
            # if np.random.rand(1) > 1-self.prob:
                self.ScanMode = 1       # 锁定目标
                # 雷达波束持续照射目标
                # self.AxisPos = [dist[0]*sin(q_z)*cos(q_x), dist[0]*sin(q_z)*sin(q_x), dist[0]*cos(q_z)]
                self.AxisPos = tgt_position
                self.TarInfo = tgt_position
        else:
            self.ScanMode = 0       

        # 如果雷达处于扫描状态，根据时间步长更新雷达方位角和俯仰角，以及波束中心轴坐标
        if self.ScanMode == 0:
            tsl = t%self.T 
            if tsl != 0:
                t_top = tsl - (self.Hlines - 1)*(self.azi_time + self.ele_time) 
                if  t_top <= self.azi_time and t_top >= 0:
                    # 雷达扫描至最顶行
                    self.Azi -= self.Vscan*t_top
                elif t_top > self.azi_time:
                    # 雷达俯仰角开始降低
                    self.Ele -= self.Vscan*t_top
                else:
                    # 雷达俯仰角在上升过程
                    col_count = tsl/(self.azi_time + self.ele_time) + 1
                    t_in = tsl%(self.azi_time + self.ele_time)
                    if col_count%2 != 0:    
                        # 奇数行
                        if t_in >= self.azi_time:
                            # 此时雷达俯仰角增大
                            self.Ele += self.Vscan*(t_in - self.azi_time)
                        else:
                            # 此时雷达从左至右扫描
                            self.Azi += self.Vscan*t_in
                    else:
                        # 偶数行
                        if t_in >= self.azi_time:
                            # 此时雷达俯仰角增大
                            self.Ele += self.Vscan*(t_in - self.azi_time)
                        else:
                            # 此时雷达从右至左扫描
                            self.Azi -= self.Vscan*t_in
            # 更新波束轴坐标
            self.AxisPos = [base_position[0] + self.max_detected_distance*cos(self.Ele)*sin(self.Azi), base_position[1] + self.max_detected_distance*cos(self.Ele)*cos(self.Azi), base_position[2] + self.max_detected_distance*sin(self.Ele)]
        
        self.radius = self.distance(base_position, self.AxisPos)[0]
        self.azi_1 = q_x-self.filed/2
        self.azi_2 = q_x+self.filed/2

        # 发布雷达信息
        self.output.instance.set_dictionary({"scan_mode":self.ScanMode, "search_mode":self.SearchMode, "radius":self.radius, 
                                             "azi_angle1":self.azi_1, "azi_angle2":self.azi_2, "target_position":self.TarInfo})
        self.output.write()

# class UAV:
#     def __init__(self, position, speed):
#         self.position = position   
#         self.speed = speed   

# if __name__ == '__main__':
#     # 仿真步长[s]
#     step = 0.05
#     # 仿真时长[s]
#     T = 50
#     # 仿真时间[s]
#     t = 0

#     # 载机坐标
#     my_position = np.array([0, 0, 0])
#     # 目标坐标
#     target_position = np.array([15000, 20000, 5000])

#     my_aircraft = UAV(my_position, 0)
#     target = UAV(target_position, [1800, 2800, 180])

#     # 雷达初始化
#     radar = Radar()

#     # 雷达波束坐标记录
#     radar_position = radar.AxisPos
#     # 目标坐标记录
#     target_position = target_position
#     # 雷达方位角
#     radar_azi= radar.Azi
#     # 雷达俯仰角
#     radar_ele = radar.Ele

#     # 三维雷达和目标
#     scene = canvas()
#     radar_beam = cone(canvas=scene, pos=vector(0, radar.max_detected_distance, 0), axis=vector(0, -radar.max_detected_distance, 0), radius=radar.max_detected_distance*tan(radar.filed))
#     target_obj = sphere(canvas=scene, pos=vector(target_position[0], target_position[1], target_position[2]), radius=1000, color=color.red)

#     # 开始仿真
#     while t<T:
#         # 目标运动变化
#         target.position[0] = target.position[0] - target.speed[0]*step
#         target.position[1] = target.position[1] - target.speed[1]*step
#         target.position[2] = target.position[2] - target.speed[2]*step


#         # 雷达扫描
#         radar.mode(my_aircraft, target)
#         radar.SightAngle(my_aircraft, target)
#         radar.scan(t+step)

#         # 时间更新
#         t += step

#         # 记录坐标
#         radar_position = np.vstack((radar_position, radar.AxisPos))
#         target_position = np.vstack((target_position, target.position))
#         radar_azi = np.vstack((radar_azi, radar.Azi))
#         radar_ele = np.vstack((radar_ele, radar.Ele))
    


#     for rpos, tgt in zip(radar_position, target_position):

#         target_obj.pos = vector(tgt[0], tgt[1], tgt[2])
#         radar_beam.pos = vector(rpos[0], rpos[1], rpos[2])
#         radar_beam.axis = vector(-rpos[0], -rpos[1], -rpos[2])
#         # time.sleep(0.05)
#         sleep(0.05) 








            

    
    