import sys
import numpy as np
from numpy import deg2rad
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D  # 空间三维画图
from aerobench.AI_F16.two_f16sim import Two_f16_sim
from aerobench.AI_F16.F16AutoPilot import F16AutoPilot
from aerobench.visualize import plot, anim3d
from aerobench.AI_F16.Viewer import ScenarioViewer


# 规划解算函数
def simulate(state_list, outside_info, filename, tmax, step):

    ap_blue = F16AutoPilot(540, 0, 0, outside_info, stdout=True)
    ap_red = F16AutoPilot(950, 0, 1, outside_info, stdout=True)

    # tmax = 150
    # step = 1/30
    extended_states = True
    res_blue, res_red = Two_f16_sim(state_list[0], state_list[1], tmax, ap_blue, ap_red, step=step, extended_states=extended_states, integrator_str='euler')

    print(f"Waypoint simulation completed in {round(res_blue['runtime'], 2)} seconds (extended_states={extended_states})")

    if filename.endswith('.mp4'):
        skip_override = 4
    elif filename.endswith('.gif'):
        skip_override = 15
    else:
        skip_override = 30

    anim_lines = []
    modes_blue = res_blue['modes']
    modes_red = res_red['modes']
    modes_blue = modes_blue[0::skip_override]
    modes_red = modes_red[0::skip_override]

    def init_extra(ax):
        'initialize plot extra shapes'

        l1, = ax.plot([], [], [], 'bo', ms=8, lw=0, zorder=50)
        anim_lines.append(l1)

        l2, = ax.plot([], [], [], 'lime', marker='o', ms=8, lw=0, zorder=50)
        anim_lines.append(l2)

        return anim_lines

    def update_extra(frame):
        'update plot extra shapes'

        # mode_names = ['Waypoint 1', 'Waypoint 2', 'Waypoint 3', 'Waypoint 4', 'Waypoint 5', 'Waypoint 6','Waypoint 6']
        mode_names = ['Waypoint ' + str(i+1) for i in range(len(ap_blue.waypoints))]

        done_xs = []
        done_ys = []
        done_zs = []

        blue_xs = []
        blue_ys = []
        blue_zs = []

        for i, mode_name in enumerate(mode_names):
            if modes_blue[frame] == mode_name:
                blue_xs.append(ap_blue.waypoints[i][0])
                blue_ys.append(ap_blue.waypoints[i][1])
                blue_zs.append(ap_blue.waypoints[i][2])
                break

            done_xs.append(ap_blue.waypoints[i][0])
            done_ys.append(ap_blue.waypoints[i][1])
            done_zs.append(ap_blue.waypoints[i][2])

        anim_lines[0].set_data(blue_xs, blue_ys)
        anim_lines[0].set_3d_properties(blue_zs)

        anim_lines[1].set_data(done_xs, done_ys)
        anim_lines[1].set_3d_properties(done_zs)

    return res_blue, res_red, init_extra, update_extra, skip_override

# 主函数
def main():
    # 若输入了文件名，则生成动态图
    if len(sys.argv) > 1 and (sys.argv[1].endswith('.mp4') or sys.argv[1].endswith('.gif')):
        filename = sys.argv[1]
        print(f"saving result to '{filename}'")
    else:
        filename = ''
        print("Plotting to the screen. To save a video, pass a command-line argument ending with '.mp4' or '.gif'.")

    # 红蓝方飞机初始状态
    state_red = [1500, deg2rad(2.15), 0, 0, 0, 0, 0, 0, 0, 5000, 2000, 3000, 9]
    state_blue = [700, deg2rad(2.15), 0, 0, 0, 0, 0, 0, 0, 30000, 30000, 4000, 9]
    states_list = [state_blue, state_red]
    
    # 解算时间和步长
    tmax = 170
    step = 0.01

    # 飞行约束
    fly_limits = [np.pi/4, np.pi/6, 5000, 400]
    # 权重
    omega = [0.05, 0.95]
    # 威胁半径
    threat_radius = 15000
    # 外部参数
    outside_info  = [threat_radius, [], omega, fly_limits, [30000, 30000, 4000]]
    

    # 仿真
    res_blue, res_red, init_extra, update_extra, skip_override = simulate(states_list, outside_info, filename, tmax, step)
    

    # # 画图
    # fig = plt.figure()
    # # ax = fig.add_subplot(projection='3d')
    # blue_xdata = [res_blue['states'][i][10] for i in range(len(res_blue['states']))]
    # blue_ydata = [res_blue['states'][i][9] for i in range(len(res_blue['states']))]
    # blue_vdata = [res_blue['states'][i][0] for i in range(len(res_blue['states']))]
    # red_xdata = [res_red['states'][i][10] for i in range(len(res_red['states']))]
    # red_ydata = [res_red['states'][i][9] for i in range(len(res_red['states']))]
    # red_vdata = [res_red['states'][i][0] for i in range(len(res_red['states']))]

    # blue_x = []
    # blue_y = []
    # red_x = []
    # red_y = []
    # index = list(range(0, len(blue_xdata), 50))
    # for i in index:
    #     blue_x.append(blue_xdata[i])
    #     blue_y.append(blue_ydata[i])
    #     red_x.append(red_xdata[i])
    #     red_y.append(red_ydata[i])
    #     plt.plot(blue_x, blue_y, 'blue', linestyle='-', marker='o')
    #     plt.plot(red_x, red_y, 'red', linestyle='-', marker='o')
    #     plt.legend([f'F16 velocity: {round(blue_vdata[i],2)}ft/s', f'Missle velocity: {round(red_vdata[i],2)}ft/s'])
    #     plt.pause(0.5)
    # plt.show()

    # anim3d.make_anim(res_blue, 'f16_blue.gif', f16_scale=140, viewsize=10000, viewsize_z=8000, trail_pts=np.inf,
    # elev=54, azim=-200, skip_frames=skip_override,
    # chase=True, fixed_floor=True, init_extra=init_extra, update_extra=update_extra)

    # anim3d.make_anim(res_red, 'f16_red.gif', f16_scale=140, viewsize=10000, viewsize_z=8000, trail_pts=np.inf,
    # elev=54, azim=-200, skip_frames=skip_override,
    # chase=True, fixed_floor=True, init_extra=init_extra, update_extra=update_extra)

    # 画图
    filename = 'D://LZP_HP//yanjiusheng//TX//DoD//AeroBenchVVPython//图表与结果//双机在线规划//F16_Missle.gif'
    Viewer = ScenarioViewer(res_blue, res_red, filename)
    Viewer.summary_video(bounds=((-50000, 40000), (-20000, 150000)), msize=0.002)

    # 画内外环控制变量图
    file_path = 'D://LZP_HP//yanjiusheng//TX//DoD//AeroBenchVVPython//图表与结果//双机在线规划//'
    # 内环
    plot.plot_inner_loop(res_blue)
    filename = file_path + 'blue_inner_loop.png'
    plt.savefig(filename)
    print(f"Made {filename}")

    plot.plot_inner_loop(res_red)
    filename = file_path + 'red_inner_loop.png'
    plt.savefig(filename)
    print(f"Made {filename}")

    # 外环
    plot.plot_outer_loop(res_blue)
    filename = file_path + 'blue_outer_loop.png'
    plt.savefig(filename)
    print(f"Made {filename}")

    plot.plot_outer_loop(res_red)
    filename = file_path + 'red_outer_loop.png'
    plt.savefig(filename)
    print(f"Made {filename}")


if __name__ == '__main__':
    main()
