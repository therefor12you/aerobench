# 针对导弹和激光武器的火控系统

class Weapon:
    def __init__(self, base_aircraft) -> None:
        self.flag = base_aircraft.flag    #身份标识
        self.id = base_aircraft.id    #编号
        self.missle_number = base_aircraft.missle_number      # 武器系统导弹数量

    # 导弹攻击区计算
    def missile_attack_zone(self, base_aircraft):
        # [1] 李志文. 战斗机空战建模及其视景仿真软件的研制与开发[D]. 西北工业大学, 2003[2023-03-04].
        # [1] 解增辉, 刘占辰, 李伟, 等. 关于某型空空近距导弹攻击区的计算仿真[J]. 弹箭与制导学报, 2004(S9): 521-523.

        # 最大允许发射距离
        D_max = 
        # 最小允许发射距离
        D_min = 
        # 最小目标观测角

        # 最大目标观测角

        return 
    
    # 激光武器攻击区计算
    def laser_attack_zone(self, base_aircraft):
        
        return 

    # 发射判定
    def missile_lauch(self):
        # 根据攻击区得出的杀伤概率进行发射判定

        return
    
    # 杀伤判定
    def boom(self):
        # 根据TX导弹攻击规则计算杀伤程度
        
        return 