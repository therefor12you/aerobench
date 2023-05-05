from Simulation.Models.F16Aircraft import F16Aircraft
from Simulation.FlightCalculate import run_f16_sim


# Control.py是仿真控制端，控制整个仿真系统的运行，根据空战逻辑规定仿真控制逻辑


# 仿真想定中的初始条件生成
def generate_states(nums):
    # 输入：红方飞机数量，蓝方飞机数量
    # 输出：红方飞机初始状态，蓝方飞机初始状态

    return 


if __name__ == '__main__':
    
    # 1. 进入战区，输入作战想定，并根据作战想定布置战场初始情况
    red_nums = int(input('请设置红方飞机数量: \n'))
    blue_nums = int(input('请设置蓝方飞机数量: \n'))
    missle_nums = int(input('请设置导弹数量: \n'))
    simulation_time = int(input('请设置仿真时长(单位/s): \n'))
    fuel = int(input('请设置油量(单位/L): \n'))

    if red_nums < 0  or blue_nums < 0  or missle_nums < 0 or simulation_time < 0 or fuel < 0:
        assert '设置量必须为非负数'
    
    # 随机生成初始的飞机状态
    nums = [red_nums, blue_nums]
    states_list = generate_states(nums)

    # 往仿真模块传递的参数
    parameters = [states_list, missle_nums, simulation_time, fuel]

    # 实例化战机
    aircrafts = []
    for i in range(nums[0]):
        aircrafts.append(F16Aircraft(0, i+1, fuel, states_list[0][i]))
    for i in range(nums[1]):
        aircrafts.append(F16Aircraft(1, i+1, fuel, states_list[1][i]))
        
    # 2. 开始作战仿真，其他的作战仿真控制在run_f16_sim函数中
    tmax = 20
    step = 1/20
    run_f16_sim(aircrafts, tmax, step, extended_states=True)

    # 3. 仿真结束，绘制结果
    
    
    

    