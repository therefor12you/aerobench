from examples.waypoint.waypoint_autopilot import WaypointAutopilot
from Alarm import Alarm
from Missile import Missile
from Radar import Radar
from Weapon import Weapon
class F16Aircraft:
    def __init__(self, flag, id, fuel, state) -> None:

        self.flag = flag    # 身份0为红，1为蓝
        self.id = id    # 编号
        self.state = state  # 飞行状态
        self.position = state[:, 9:11][:, ::-1]     # 坐标
        self.speed = state[0]       # 速度
        self.euler_angles = state[3:5]      # 本机欧拉角
        self.fuel = fuel    # 油量
        self.hp = 100       # 飞机生命值，以上两项归到state?
        self.radar = Radar()    # 雷达
        self.missle = Missile()  # 导弹
        self.weapon = Weapon()  # 火控系统
        self.ap = WaypointAutopilot()   # 自动驾驶仪
        self.res =  {}      # 存储仿真结果


    
    def path_plan(self):
        
        return
