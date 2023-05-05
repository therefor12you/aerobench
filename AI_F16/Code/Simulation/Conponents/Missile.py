import numpy as np
from math import sin, cos, atan
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import rticonnextdds_connector as rti
from os import path 

# 中远距导弹建模仿真

# 参考文献：
# [1] 张建伟, 黄树彩, 韩朝超. 基于Matlab的比例导引弹道仿真分析[J/OL]. 战术导弹技术, 2009(03): 60-64. 
# [2] 李志文. 战斗机空战建模及其视景仿真软件的研制与开发[D/OL]. 西北工业大学, 2003[2023-03-04].


# 目前不足点如下:
# 1. 应当增加导弹属性：导弹最大飞行时间tmax,导弹引信解除时间tf, 导弹最大飞行速度vmax, 导弹最大飞行加速度amax
# 2. 应当增加导弹的动力学模型和运动学模型

class Missile:
    def __init__(self):
       
        # 状态：'Closed','Activated'和'Boom' 
        self.status = "Closed"   

        # 导弹初始速度[m/s]
        self.speed = 600
        # 导弹最大飞行速度[m/s]
        self.v_max = 1000
        # 导弹最大飞行加速度[m/s^2]
        self.a_max = 50
        # 导弹最大飞行时间[s]
        self.t_max = 20
        # 导弹引信解除时间[s]
        self.t_f = 1.5
        # 导弹质量
        self.m = 0
        # 导弹推力
        self.F = 0
        # 导弹阻力
        self.D = 0 

        # 导弹位置
        self.position = np.array([1000,5000,1000])
        # 导弹三轴速度
        self.velocity = np.array([0,0,0])

        # 比例导引法的比例系数，可以是变比例的
        self.k = 1.5

        # 导弹在地面系下的俯仰角和方位角
        self.theta = np.pi/3     # 导弹俯仰角
        self.psi = np.pi      # 导弹方位角

        # 导弹的过载指令
        self.ny = 0     # 侧向过载
        self.nz = 0     # 法向过载

        # 测量到的目标俯仰角
        self.q_h = 0
        # 测量到的目标俯仰角
        self.q_v = 0

        # DDS
        self.file_path = path.dirname(path.realpath(__file__))
        self.pub_connector = rti.Connector("MyParticipantLibrary::MyPubParticipant", self.file_path + "\ShapeExample.xml")
        self.sub_connector = rti.Connector("MyParticipantLibrary::MySubParticipant", self.file_path + "\ShapeExample.xml")

        self.output = self.pub_connector.get_output("MyPublisher::MyMissileWriter")
        self.output.instance.set_dictionary({"position":self.position.tolist(), "velocity":self.speed})
        self.output.write()
        # self.output.wait() # Wait for all subscriptions to receive the data before exiting

    # 比例导引律
    def png(self, step, t):

        g = 9.81

        # 订阅目标信息
        input = self.sub_connector.get_input("MySubscriber::MyAircraftReader")
        # input.wait_for_publications() # wait for at least one matching publication
        # input.wait() # wait for data on this input
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            tgt_position = np.array([data['state'][i] for i in [10,9,11]])


        # 计算新的水平面内目标视线角
        if tgt_position[0] - self.position[0] == 0:
            q_h_new = 0
        else:
            q_h_new = atan((tgt_position[1] - self.position[1])/(tgt_position[0] - self.position[0]))

        # 计算新的垂直面内目标视线角
        if np.sqrt((tgt_position[0] - self.position[0])**2 + (tgt_position[1] - self.position[1])**2) == 0:
            q_v_new = np.pi/2
        else:
            q_v_new = atan((tgt_position[2] - self.position[2])/np.sqrt((tgt_position[0] - self.position[0])**2 + (tgt_position[1] - self.position[1])**2))

        # # 计算新的水平面内目标视线角
        # if target.position[0] - self.position[0] == 0:
        #     q_h_new = 0
        # else:
        #     q_h_new = atan((target.position[1] - self.position[1])/(target.position[0] - self.position[0]))

        # # 计算新的垂直面内目标视线角
        # if np.sqrt((target.position[0] - self.position[0])**2 + (target.position[1] - self.position[1])**2) == 0:
        #     q_v_new = np.pi/2
        # else:
        #     q_v_new = atan((target.position[2] - self.position[2])/np.sqrt((target.position[0] - self.position[0])**2 + (target.position[1] - self.position[1])**2))

        # # 比例导引法计算导弹过载指令ny和nz
        # self.ny = self.k*epsilon_d*self.speed*cos(self.theta)/g
        # self.nz = self.k*q_d*self.speed*cos(self.epsilon - self.psi)/g + cos(self.theta)

        # 计算导弹的加速度、俯仰角速度和方位角速度
        self.thrust(t)
        self.drag()
        self.mass(t)
        speed_d = (self.F - self.D)/self.m - g*sin(self.theta)

        theta_d = q_v_new - self.q_v
        psi_d = q_h_new - self.q_h
        self.q_h = q_h_new 
        self.q_v = q_v_new
        
        # 根据导弹的加速度、俯仰角速度和方位角速度更新导弹的速度、俯仰角、方位角，进而得到导弹运动坐标
        self.speed = self.speed + speed_d*step
        self.theta = self.theta + self.k*theta_d
        self.psi = self.psi + self.k*psi_d

        self.position[0] = self.position[0] + self.speed*step*cos(self.theta)*cos(self.psi)
        self.position[1] = self.position[1] + self.speed*step*cos(self.theta)*sin(self.psi)
        self.position[2] = self.position[2] + self.speed*step*sin(self.theta)

        self.velocity[0] = self.speed*cos(self.theta)*cos(self.psi)
        self.velocity[1] = self.speed*cos(self.theta)*sin(self.psi)
        self.velocity[2] = -self.speed*sin(self.theta)
        
        # 发布信息
        self.output.instance.set_dictionary({"position":self.position.tolist(), "velocity":self.speed})
        self.output.write()
        print('导弹坐标{}, 速度{}, 已发布'.format(self.position, self.speed))
        self.output.wait() # Wait for all subscriptions to receive the data before exiting

        # return self.position, self.speed

    # 推力计算
    def thrust(self, t):
        if t > self.t_max:
            self.F = 0
        else:
            # 单位[N]
            self.F = 40000

    # 导弹被击中
    def be_hitted(self):
        self.psi = -np.pi*np.random.normal(2, 0, 1)

    # 阻力计算
    def drag(self):

        rho = 1.2
        r = 0.2
        A = np.pi*r*r

        Ma = self.speed/340

        # 气动参数
        if Ma >= 0 and Ma < 0.9: 
            Cd = 0.2
        elif Ma < 1.1:
            Cd = 0.2 + 0.075/0.2*(Ma-0.9)
        elif Ma < 3:
            Cd = 0.275 - 0.075/1.9*(Ma-1.1)
        elif Ma > 3:
            Cd = 0.2
        else:
            assert 'Error: Invalid Vmag to compute drag coefficient'
            Cd = None

        # 阻力的简化计算
        self.D = 0.5*rho*self.speed**2*A*Cd
        
    # 质量变化计算
    def mass(self, t):
        # 单位[kg]
        m0 = 700
        mf = 90

        if t >= 0 and t <= self.t_max: 
            self.m = m0 - ((m0-mf)/self.t_max)*t
        elif t > self.t_max:
            self.m = mf
        else:
            assert 'Error in computing m in mass'
            self.m = None


# 载机测试用类
class Baseaircraft:
    def __init__(self):
        self.flag = 0
        self.id = 0
        self.euler_angles = np.array([np.pi/3, np.pi/3])
        self.position = None


# 目标测试用类 
class Target:
    def __init__(self, speed, position):
        self.speed = speed
        self.position = position
        self.euler_angles = np.array([np.pi/3, np.pi/2])

        # # 分割线---------------------------------------------------------------
        # # 目标速度在垂直方向上的夹角
        # self.sigma_v = np.pi/3
        # # 目标速度在水平面上的投影与x轴的夹角
        # self.sigma_h = 2*np.pi/3

    def tgt_update(self, step):
        self.position[0] = self.position[0] + self.speed*step*sin(self.euler_angles[0])*cos(self.euler_angles[1])
        self.position[1] = self.position[1] + self.speed*step*sin(self.euler_angles[0])*sin (self.euler_angles[1])
        self.position[2] = self.position[2] + self.speed*step*cos(self.euler_angles[0])


# 实例化验证
if __name__ == '__main__':
   
    # 目标
    target = Target(300, np.array([250, 100, 50]))

    # 导弹
    missile = Missile()
    missile.status = 'Activated'

    t = 0
    step = 0.05

    missile_pos = missile.position
    target_pos = target.position

    # 开始仿真
    while t <= missile.t_max:
        t += step
        # 更新导弹运动
        missile.png(target, step, t)
        
        # 更新目标运动
        target.tgt_update(step)

        # 记录数据
        target_pos = np.vstack((target_pos, target.position))
        missile_pos = np.vstack((missile_pos, missile.position))
        print('导弹俯仰角{}, 导弹偏航角{}, 导弹速度{}'.format(missile.theta, missile.psi, missile.speed))
        
    

    # 绘制动态图
    fig, ax = plt.subplots()

    x_missile = []
    y_missile = []
    x_target = []
    y_target = []
    
    for i in range(0, len(missile_pos), 5):
        x_missile.append(missile_pos[i, 0])
        y_missile.append(missile_pos[i, 1])

        x_target.append(target_pos[i, 0])
        y_target.append(target_pos[i, 1])
        plt.cla()           #清除之前的图
        plt.ion()           #打开交互模式(不阻塞)
        plt.plot(x_missile, y_missile, 'r', lw=2, label='Missile')
        plt.plot(x_target, y_target, 'b', lw=2, label='Target')
        plt.legend(['Missile', 'Target'])
        plt.grid()
        # plt.legend()
        plt.pause(0.01)
        plt.show()

    # 绘制参数图


    

    
    



    

    