import numpy as np
import rticonnextdds_connector as rti
from AI_F16.Code.Utils.distance import distance
from os import path 
import matplotlib.pyplot as plt 

# 飞机电池充放电模型
# [1] COSTA J O, FREITAS D C C, SILVA H S, 等. POLYNOMIAL APPROXIMATION OF DISCHARGE CURVE OF A LEAD-ACID BATTERY MODEL[J]. 2017.
# https://www.electricaltechnology.org/2013/03/easy-charging-time-formula-for.html

class Battery:
    def __init__(self):
        # discharge/chgarge current[A]
        self.current = 2.5
        # full voltage[V]
        self.vol_full = 12.8
        # voltage at the end of the exponential zone[V]
        self.vol_exp = 12.6
        # voltage at the end of the nominal zone[V]
        self.vol_nom = 11.8
        # nominal zone capacity[Ah]
        self.Q_full = 36
        self.Q_exp = 1.25
        self.Q_nom = 25
        
        self.voltage = self.vol_full
        self.soc = self.voltage/self.vol_full
        self.soc_low_limitted = 0.2

        # 充放电标志, True为放电
        self.charge_flag = True
        # 最小发射距离[m]
        self.Rmin = 2000
        # 给激光武器的信号
        self.laser_flag = False
        # 放电结束时间
        self.end_discharge_time = 0
        # 充电结束时间
        self.end_charge_time = 0

        self.file_path = path.dirname(path.realpath(__file__))
        self.pub_connector = rti.Connector("MyParticipantLibrary::MyPubParticipant", self.file_path + "\ShapeExample.xml")
        self.sub_connector = rti.Connector("MyParticipantLibrary::MySubParticipant", self.file_path + "\ShapeExample.xml")

        self.output = self.pub_connector.get_output("MyPublisher::MyBatteryWriter")
        self.output.instance.set_number("soc", self.soc)
        self.output.instance.set_number("soc_low_limitted", self.soc_low_limitted)
        self.output.instance.set_boolean("charge_flag", self.charge_flag)
        self.output.instance.set_boolean("laser_flag", self.laser_flag)
        self.output.instance.set_number("voltage", self.voltage)
        self.output.instance.set_number("full_voltage", self.vol_full)
        self.output.write()
        # self.output.wait()
        
    def dis_and_charge(self, t):
        # 充放电公式参数
        Ri = 0.0245
        A = self.vol_full - self.vol_exp
        B = 3/self.Q_exp
        K = (self.vol_full - self.vol_nom + A*(np.exp(-B*self.Q_nom) - 1))*(self.Q_full - self.Q_nom)/self.Q_nom
        E_0 = self.vol_full + K + Ri - A

        # *******************发射决策*******************
        # 订阅导弹信息
        input = self.sub_connector.get_input("MySubscriber::MyMissileReader")
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            tgt_position = data['position']
            tgt_velocity = data['velocity']
        # 订阅载机信息
        input = self.sub_connector.get_input("MySubscriber::MyAircraftReader")
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            base_position = [data['state'][i] for i in [10,9,11]]
            base_velocity = data['state'][0]
            # print('得到飞机坐标{}, 速度{}'.format(base_position, base_velocity))

        # 放电
        if self.charge_flag and self.soc > self.soc_low_limitted:
            if distance(base_position, tgt_position) < self.Rmin:
                self.laser_flag = True
                self.voltage = self.voltage - 0.8*self.vol_full
            else:
                self.voltage = E_0 - K*(self.Q_full/(self.Q_full - self.current*((t-self.end_charge_time)/60))) - Ri + A*np.exp(-B*self.current*((t-self.end_charge_time)/60))
            # self.voltage = E_0 - K*(self.Q_full/(self.Q_full - self.current*((t-self.end_charge_time)))) - Ri + A*np.exp(-B*self.current*((t-self.end_charge_time)))
            self.soc = self.voltage/self.vol_full
            self.end_discharge_time = t
        # 充电
        else:
            self.laser_flag = False
            self.charge_flag = False
            self.voltage = self.voltage + K*self.current*((t-self.end_discharge_time)/60)/(self.Q_full - self.current*((t-self.end_discharge_time)/60)) - A*(np.exp(-B*self.current*((t-self.end_discharge_time)/60)) - 1)
            self.soc = self.voltage/self.vol_full
            if self.voltage - self.vol_full > -0.01:            
                self.charge_flag = True
                self.end_charge_time = t
            else:
                self.charge_flag = False

        # 发布信息
        self.output.instance.set_number("soc", self.soc)
        self.output.instance.set_number("soc_low_limitted", self.soc_low_limitted)
        self.output.instance.set_boolean("charge_flag", self.charge_flag)
        self.output.instance.set_boolean("laser_flag", self.laser_flag)
        self.output.instance.set_number("voltage", self.voltage)
        self.output.instance.set_number("full_voltage", self.vol_full)
        self.output.write()
        print('电池soc:{}, 已发布'.format(self.soc))
        # sleep(0.5) # Write at a rate of one sample every 0.5 seconds, for ex.
        # print("Exiting...")
        self.output.wait() # Wait for all subscriptions to receive the data before exiting

# 实例化验证

def main():
    battery = Battery()
    t_max = 20
    step = 0.1
    t = 0
    times = np.linspace(0, 20, 200)
    voltage = []
    while(t<t_max):
        battery.dis_and_charge(t)
        voltage.append(battery.voltage)
        t += step
    
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)
    plt.xticks(fontsize = 20)
    plt.yticks(fontsize = 20)
    ax.plot(times, voltage, linestyle = "-")
    ax.set_title('Voltage vs Time',fontsize = 30) 
    ax.set_xlabel('t/h',fontsize = 20) 
    ax.set_ylabel('voltage/V', fontsize = 20)

    fig.savefig('D:\\LZP_HP\\yanjiusheng\\TX\\DoD\\aerobench\\AI_F16\\Code\\Test\\view_results\\discharge.jpg')

if __name__ == '__main__':
    main()
