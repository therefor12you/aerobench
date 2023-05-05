import numpy as np
import rticonnextdds_connector as rti
from os import path 

# 飞机高能装备的热模型
class Laser:
    def __init__(self):
        # 材料厚度[m]
        self.L = 0.01
        # 材料微元数
        self.n = 10
        # 材料热导率[W/m°C]
        self.k = 0.456
        # 比热容[J/kg°C]
        self.C_rho = 1190
        # 材料密度[kg/m^3]
        self.rho = 1865
        # 换热面积[m^2]
        self.A = 0.1
        # 初始温度[°C]
        self.T0 = 30
        # absorptivity
        self.alpha_d = 0.4
        # 对流系数[W/m^2°C]
        self.h = 500
        # 材料微元
        self.dx = self.L/self.n
        # conduction coefficient
        self.alpha = self.k/(self.rho*self.C_rho)
        # 热容值
        self.C = self.L*self.A*self.rho*self.C_rho
        # 材料温度分布
        self.T = np.ones(self.n)*self.T0
        # 温度微分矩阵
        self.dTdt = np.empty(self.n)
        # 前板发热温度
        self.T_front = self.T0
        # 后板发热温度
        self.T_rear = self.T0
        # 热量[J]
        self.Q = 0
        # 发射一次需要的功率[J/m^2]
        self.power = 1000000
        self.power_pause = 0
        # 最大温度约束[°C]
        self.temp_max = 500
        # 发射指令
        self.launch_flag = False

        self.file_path = path.dirname(path.realpath(__file__))
        self.pub_connector = rti.Connector("MyParticipantLibrary::MyPubParticipant", self.file_path + "\ShapeExample.xml")
        self.sub_connector = rti.Connector("MyParticipantLibrary::MySubParticipant", self.file_path + "\ShapeExample.xml")

        # 发布信息
        self.output = self.pub_connector.get_output("MyPublisher::MyLaserWriter")
        self.output.instance.set_boolean("launch_flag", self.launch_flag)
        self.output.instance.set_number("out_thermal", self.Q)
        self.output.instance.set_dictionary({"temperature":self.T.tolist()})
        self.output.write()
        

    # 发射决策，及发射后的热变化
    def thermal(self, dt):

        # 订阅电池信息
        input = self.sub_connector.get_input("MySubscriber::MyBatteryReader")
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            self.launch_flag = data['laser_flag']

        # 判断是否满足发射条件
        if self.launch_flag:
                self.power_pause = self.power
                print('激光武器已发射')
        else:
            self.power_pause = 0
            
        
        # *******************热变化*******************
        # 前板发热温度边界
        self.T_fronts = (self.alpha_d*self.power_pause + self.h*self.T0 + self.k/self.dx*self.T[1])/(self.h + self.k/self.dx)
        # 后板发热温度边界
        self.T_rears = (self.h*self.T0 + self.k/self.dx*self.T[self.n-2])/(self.h + self.k/self.dx)
        # 仿真计算
        for i in range(1, self.n-1):
            self.dTdt[i] = self.alpha*(-(self.T[i]-self.T[i-1])/self.dx**2+(self.T[i+1]-self.T[i])/self.dx**2)
        self.dTdt[0] = self.alpha*(-(self.T[0]-self.T_fronts)/self.dx**2+(self.T[1]-self.T[0])/self.dx**2)
        self.dTdt[self.n-1] = self.alpha*(-(self.T[self.n-1]-self.T[self.n-2])/self.dx**2+(self.T_rears-self.T[self.n-1])/self.dx**2)
        self.T = self.T + self.dTdt*dt
        self.T_front = self.T[0]
        self.T_rear = self.T[-1]
        self.Q = self.C*(self.T_front-self.T_rear)

        # # 传输能量
        # self.power_pause += self.power*dt 

        # 发布信息
        # output = self.pub_connector.get_output("MyPublisher::MyLaserWriter")
        self.output.instance.set_boolean("launch_flag", self.launch_flag)
        self.output.instance.set_number("out_thermal", self.Q)
        self.output.instance.set_dictionary({"temperature":self.T.tolist()})
        self.output.write()
        print('高能装备散热{}J, 已发布'.format(self.Q))
        # sleep(0.5) # Write at a rate of one sample every 0.5 seconds, for ex.
        # print("Exiting...")
        self.output.wait() # Wait for all subscriptions to receive the data before exiting

