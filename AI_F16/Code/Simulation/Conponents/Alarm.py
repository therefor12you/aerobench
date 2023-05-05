class TgtInfo:
    def __init__(self, flag, id, pos, speed) -> None:
        self.flag = flag    # 身份
        self.id = id    # 编号
        self.pos= pos  # 位置
        self.speed = speed  #速 度
        


class Alarm:
    def __init__(self) -> None:
        self.number = 0
        self.Tgt = []
        pass

    def scan(self):
        
        return