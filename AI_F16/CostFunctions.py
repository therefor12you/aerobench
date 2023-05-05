import numpy as np
from FunFunctions import Euclid, get_vector, vectors_angle, normalization

# 第二部分
# 实现航迹点评价函数和路径点评价函数，这部分需要模块化，并进行重写

# 2.1 航迹点评价函数
def fitness_wpt(ex_point, point, end, out_info):

    # outside_info = [threat_radius, threat_point, omega, fly_limits, start_point]

    # 距离终点的距离代价
    distance = Euclid(point, end)/Euclid(out_info[4], end)
    # 受威胁代价
    if Euclid(point, out_info[1])>out_info[0]:
        threat = 0
    else:
        threat = out_info[0]/(Euclid(point, out_info[1]) + out_info[0])

    return out_info[2][0]*(distance**2) + out_info[2][1]*(threat**2)


