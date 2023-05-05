import numpy as np

from operator import itemgetter
from CostFunctions import fitness_wpt
from FunFunctions import Euclid
from math import acos, sqrt, atan
from Sphere import sphere

# 路径点决策函数，还需重写

# 参数初始化
def next_wpt(ex_point, end, outside_info):
    # outside_info = [threat_radius, threat_point, omega, fly_limits]
    
    points = valid_point(ex_point, end, outside_info)

    best_wpt = selectBest(points)

    return best_wpt['Point']
    

# 适应度评价函数
def evaluate(ex_point, point, end, outside_info):
    fitness = fitness_wpt(ex_point, point, end, outside_info)
    return fitness

# 从种群中选择最好的个体
def selectBest(points):
    # 对整个种群按照成本从小到大排序，返回成本最小的个体
    pop_sorted = sorted(points, key=itemgetter("Fitness"), reverse=False)  
    return pop_sorted[0]

# 生成满足约束的航迹点
def valid_point(ex_point, end, outside_info):

    end_xy = np.array([end[0], end[1]])

    # 约束条件
    fly_limits = outside_info[3]
    # 偏转角约束
    alpha_max = fly_limits[0]
    # 俯仰角约束
    beta_max = fly_limits[1]
    # 最小航迹段长度约束
    L_min = fly_limits[2]
    # 最小航迹段高度约束
    h_min = fly_limits[3]

    # 基于前一点生成备选点
    posible_points = sphere(ex_point, L_min)
    x = posible_points[0]
    y = posible_points[1]
    z = posible_points[2]

    points = []
    points.append({'Point': ex_point, 'Fitness': 10000})

    # 在备选点中选择满足约束的点
    for i in range(x.shape[0]):
        for j in range(x.shape[1]):
            point = np.array([x[i,j],y[i,j],z[i,j]])
            point_xy = np.array([x[i,j],y[i,j]])
            wpt_back_xy = np.array([ex_point[0],ex_point[1]])
            vector_1 = point_xy - wpt_back_xy
            vector_2 = end_xy - point_xy

            if(vector_1[0]==0 and vector_1[1]==0):
                alpha_i = 0
                beta_i = np.pi/2
            else:
                # 计算第i个点的偏转角
                alpha_i = acos(np.dot(vector_1, vector_2)/(sqrt(vector_1[0]**2+vector_1[1]**2)*sqrt(vector_2[0]**2+vector_2[1]**2)))
                # 计算第i个点的俯仰角
                beta_i = atan(abs(point[2]-ex_point[2])/sqrt(vector_1[0]**2+vector_1[1]**2))
                # 计算第i个点与前一点的高度差
            dh_i = abs(point[2] - ex_point[2])
            
            if(alpha_i<alpha_max and beta_i<beta_max and point[2]>0 and dh_i<h_min):
                fitness = evaluate(ex_point, point, end, outside_info)
                points.append({'Point': point, 'Fitness': fitness})
    
    return points

    # # 航迹平滑
    # def smooth(self, path):
        
    #     # 邻域点数
    #     N = self.parameter[7]
    #     # 总点数
    #     N_wpts = path.shape[0]

    #     # N点滑动平均平滑
    #     for i in range(N_wpts):
    #         if i == 0 or i >= N_wpts-1:
    #             path[i,:] =  path[i,:]
    #         elif i == 1:
    #             path[i,:] = (path[i-1,:] + path[i,:] + path[i+1,:])/3
    #         else:
    #             for j in range(i-N, i+N):
    #                 path[i,:] += path[j,:]
    #             path[i,:] = path[i,:]/(2*N+1)
    #     return path

