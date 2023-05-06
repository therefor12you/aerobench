import numpy as np
from svgpath2mpl import parse_path  # type: ignore
import matplotlib as mpl  # type: ignore
# import seaborn as sns
import matplotlib.pyplot as plt  # type: ignore
from matplotlib.collections import LineCollection  # type: ignore
import matplotlib.animation as animation  # type: ignore
from numpy import sin, cos


class ScenarioViewer:
    # f16飞机图标
    @staticmethod
    def create_f16_marker(theta):
        t = parse_path(
            """M 372.74513,979.43631 C 371.65529,965.92359 369.95779,948.56184 368.97287,940.85467 L 367.18212,926.84165 L 349.19283,925.96456 L 331.20356,925.08746 L 327.87949,913.19903 C 325.96331,906.34592 322.47402,900.90981 319.64122,900.36428 C 315.82936,899.63016 248.84768,915.74382 215.29064,925.46768 C 210.6617,926.80902 210.08428,924.54182 210.1256,905.18649 L 210.17207,883.39515 L 262.24096,800.13058 L 314.30987,716.86597 L 314.19174,691.43277 L 314.07357,665.99953 L 231.55162,750.23936 C 186.16455,796.57125 147.65634,834.42935 145.97784,834.36847 C 140.86463,834.18303 125.69304,770.54619 127.92525,758.64747 C 129.05661,752.61685 147.83573,720.75969 172.55121,682.94354 C 200.18189,640.66685 229.45816,590.48182 255.60553,540.57291 L 295.98723,463.49405 L 298.08663,429.99131 L 300.18604,396.48856 L 317.26626,396.48856 L 334.34644,396.48856 L 336.24026,382.67182 C 337.28184,375.07257 338.13405,356.2719 338.13405,340.89249 C 338.13405,306.85496 346.08391,196.28826 350.30292,171.6479 C 353.90673,150.60058 374.67947,74.862184 376.84833,74.862184 C 379.01998,74.862184 399.79443,150.62154 403.38857,171.6479 C 407.7049,196.89905 415.54975,303.20976 415.5849,336.92811 C 415.60112,352.48827 416.45944,372.25489 417.49232,380.85393 L 419.37025,396.48856 L 436.49812,396.48856 L 453.62594,396.48856 L 455.70958,429.99131 L 457.79321,463.49405 L 498.3161,540.92263 C 523.99736,589.99277 553.30295,640.40526 578.3308,678.56664 C 623.49827,747.43608 627.00218,753.61651 627.00218,764.41604 C 627.00218,776.60416 611.0839,834.24507 607.68646,834.35927 C 606.02576,834.41508 567.53592,796.55697 522.15341,750.23015 L 439.6398,665.99953 L 439.55978,691.31272 L 439.47978,716.62591 L 491.54868,799.89049 L 543.61757,883.15508 L 543.61757,905.06645 C 543.61757,924.52436 543.03402,926.80873 538.40603,925.46768 C 514.86526,918.64623 439.34504,899.77426 435.58838,899.77426 C 430.94458,899.77426 429.87625,901.89355 426.20575,918.38691 C 424.45136,926.27017 423.67573,926.57646 405.46648,926.57646 L 386.54971,926.57646 L 384.7414,940.72207 C 383.7468,948.50215 382.04138,965.92359 380.95153,979.43631 C 379.86169,992.94911 378.01527,1004.005 376.84833,1004.005 C 375.68138,1004.005 373.83496,992.94911 372.74513,979.43631 z """)
        t.vertices -= np.array(t.vertices).mean(axis=0)
        theta *= -1
        theta += np.pi
        t.vertices = [(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta)) for x, y in t.vertices]
        return t

    # 障碍物图标
    @staticmethod
    def create_balloon_marker():
        t = parse_path(
            """M53.083,5c-20.547,0-37.204,8.896-37.204,31.387c0,13.679,18.33,31.619,29.373,43.503    c1.214,1.306,2.343,2.542,3.347,3.688c0.292,0.334,0.524,0.654,0.725,0.967c0.115,0.178,0.275,0.32,0.474,0.394    c1,0.369,2.112,0.578,3.286,0.578s2.285-0.209,3.286-0.578c0.199-0.073,0.359-0.215,0.474-0.394    c0.201-0.313,0.433-0.633,0.725-0.967c1.004-1.146,2.133-2.382,3.347-3.688c11.044-11.883,29.373-29.824,29.373-43.503    C90.287,13.896,73.631,5,53.083,5z""")
        t.vertices -= np.array(t.vertices).mean(axis=0)
        # theta *= -1
        theta = np.pi
        t.vertices = [(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta)) for x, y in t.vertices]
        return t

    
    def __init__(self, res, filenames):

        self.result = res
        self.filename = filenames

        self.aircraft_states = np.array(res['aircraft_states'])
        self.missile_pos = np.array(res['missile_pos'])
        # print(self.missile_pos)
        # 飞机输出信息中y在前x在后,需要反转
        self.aircraft_position = self.aircraft_states[:, 9:11][:, ::-1]
        self.aircraft_velocity = self.aircraft_states[:,0]
        self.aircraft_heading = self.aircraft_states[:,5]
        self.missile_position = self.missile_pos[:,0:2]
        # print(self.missile_position)

        self.missile_velocity = res['missile_vel']
        self.laser_thermal = res['laser_thermal']
        self.battery_soc = res['battery_soc']
        self.radar_radius = res['radar_radius']
        self.radar_azi = res['radar_azi']
        self.time = res['times']

        # self.f16_states = np.array(res_f16['states'])
        # self.f16_pos = self.f16_states[:, 9:11][:, ::-1]
        # self.f16_v = self.f16_states[:, 0]
        # self.f16_alt = self.f16_states[:, 11]

        # self.missle_states = np.array(res_mimssle['states'])
        # self.missle_pos = self.missle_states[:, 9:11][:, ::-1]
        # self.missle_v = self.missle_states[:, 0]
        # self.missle_alt = self.missle_states[:, 11]

        # self.times = np.array(res_f16['times'])
        # self.filename = filename
        
        # self.f16_heading = np.array(res_f16['states'])[:, 5]
        # self.missle_heading = np.array(res_mimssle['states'])[:, 5]

        # self.f16_end_point = np.array([-20000, 50000])

    def key_variable_plot(self):
        # 绘制一些关键变量变化
        time = self.result['times']
        soc = self.result['battery_soc']
        thermal = self.result['laser_thermal']

        fig = plt.figure(figsize=(15, 10))
        ax1 = plt.subplot(121)
        plt.plot(time, soc, linestyle = "-")
        plt.xlabel('t/s')
        plt.ylabel('soc')
        plt.title('SoC vs Time')
        ax2 = plt.subplot(122)
        plt.plot(time, thermal, linestyle = "-")
        plt.xlabel('t/s')
        plt.ylabel('Q/J')
        plt.title('Q vs Time')

        fig.savefig(self.filename[1])

    def compute_bounds(self):
        
        xmin, xmax = min(self.aircraft_position[:, 0]), max(self.aircraft_position[:, 0])
        ymin, ymax = min(self.aircraft_position[:, 1]), max(self.aircraft_position[:, 1])
        xmin -= (xmax - xmin) * 0.05
        xmax += (xmax - xmin) * 0.05
        ymin -= (ymax - ymin) * 0.05
        ymax += (ymax - ymin) * 0.05
        return (xmin, xmax), (ymin, ymax)
        # return min(xmin, ymin), max(xmax, ymax)

    # def summary_video(self, bounds=((-10000, 10000), (0, 20000)), msize=0.001):
    def summary_video(self, bounds=((-7500, 7500), (0, 7500)), msize=0.001):
        fig = plt.figure(figsize=(10, 10))
        
        xbs, ybs = self.compute_bounds()
        pbounds = min(xbs[0], ybs[0]), max(xbs[1], ybs[1])
        ax = plt.axes(xlim=bounds[0], ylim=bounds[1])
        # ax.legend(['Aircraft', 'Missile'])
        # ax = plt.axes(xlim=pbounds[0], ylim=pbounds[1])

        # # 画障碍物
        # ax.plot(*self.f16_end_point, 
        #         marker=self.create_balloon_marker(),
        #         c='grey',
        #         markersize=30,
        #         label="End Point")

        v_aircraft_text = ax.text(0.05, 0.95, "", transform=ax.transAxes, fontsize=20)
        soc_aircraft_text = ax.text(0.05, 0.91, "", transform=ax.transAxes, fontsize=20)
        # status_laser_text  = ax.text(0.05, 0.81, "", transform=ax.transAxes, fontsize=10)
        thermal_laser_text = ax.text(0.05, 0.87, "", transform=ax.transAxes, fontsize=20)
        v_missile_text = ax.text(0.55, 0.95, "", transform=ax.transAxes, fontsize=20)

        lines = []
        markers = []
        wedges = []
    
        line_aircraft, = ax.plot([], [], 'b', lw=2)
        lines.append(line_aircraft)
        line_missile, = ax.plot([], [], 'r', lw=2)
        lines.append(line_missile)
        line_laser, = ax.plot([], [], 'g', lw=2)
        lines.append(line_laser)
        marker_aircraft = ax.scatter([], [], marker='x', s=msize, zorder=300, color='b')
        markers.append(marker_aircraft)
        marker_missile = ax.scatter([], [], marker='x', s=msize*10, zorder=300, color='r')
        markers.append(marker_missile)
        wedge_radar, = ax.fill([], [], color='b')
        wedges.append(wedge_radar)

        # 更新轴的界限，以便在更新曲线时自动调整
        ax.relim()
        ax.autoscale_view()
        skip_size = 2
        x_data1 = []
        y_data1 = []
        x_data2 = []
        y_data2 = []
        x_data3 = []
        y_data3 = []

        # animation function
        def animate(num):
            num *= skip_size
            # t is a parameter
            # x, y values to be plotted

            # 飞机轨迹
            x, y = self.aircraft_position[num]
            x_data1.append(x)
            y_data1.append(y)
            lines[0].set_data(x_data1, y_data1)
            markers[0].set_offsets(np.c_[[x_data1[-1]], [y_data1[-1]]])
            markers[0].set_paths([self.create_f16_marker(self.aircraft_heading[num])])
            
            # 雷达波束扇形
            theta = np.linspace(self.radar_azi[num][0], self.radar_azi[num][1], 100)
            r = self.radar_radius[num]
            x = np.append(x, r * np.cos(np.radians(theta)))
            y = np.append(y, r * np.sin(np.radians(theta)))
            wedges[0], = ax.fill(x, y, color='b')
            ax.clear()

            # 导弹轨迹
            x, y = self.missile_position[num]
            x_data2.append(x)
            y_data2.append(y)
            lines[1].set_data(x_data2, y_data2)
            markers[1].set_offsets(np.c_[[x_data2[-1]], [y_data2[-1]]])
            markers[1].set_paths([self.create_balloon_marker()])

            # Laser轨迹
            if self.laser_thermal[num] > 0:
                x_data3 = [x_data1[-1], x_data2[-1]]
                y_data3 = [y_data1[-1], y_data2[-1]]
                lines[2].set_data(x_data3, y_data3)

        
            # 其他信息
            v_aircraft_text.set_text('v_aircraft = {:.2f} m/s'.format(self.aircraft_velocity[num]))
            soc_aircraft_text.set_text('soc = {:.6f} '.format(self.battery_soc[num]))
            # status_laser_text.set_text('laser flag = {}'.format(self.))
            thermal_laser_text.set_text('Q = {:.2f} J'.format(sum(self.laser_thermal[:num])))
            v_missile_text.set_text('v_missile = {:.2f} m/s'.format(self.missile_velocity[num]))

            return lines, markers, wedges,

        # setting a title for the plot
        plt.grid()
        plt.legend(['Aircraft','Missile','Laser'])
        ax.set_aspect('equal', 'datalim')
        plt.xlabel("East / West Position (m)")
        plt.ylabel("North / South Position (m)")

        # call the animator
        animate_obj = animation.FuncAnimation(fig, animate,
                                       frames=len(self.aircraft_position) // skip_size,
                                       interval=60)
        animate_obj.save(self.filename[0], dpi=60, writer='imagemagick')
        print("动图绘制完毕")
        return 1