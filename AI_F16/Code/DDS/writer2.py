import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 材料厚度[m]
L = 0.01
# 材料微元数
n = 10
# 材料微元
dx = L/n
# 初始温度[°C]
T0 = 30
# 材料热导率[W/m°C]
k = 0.456
# 比热容[J/kg°C]
C_rho = 1190
# 材料密度[kg/m^3]
rho = 1865

alpha = k/(rho*C_rho)

# 换热面积[m^2]
A = 0.1
# 换热器热容[J/°C]
C = L*A*rho*C_rho

# 仿真时间
t_final = 30
dt = 0.05
t = np.arange(0, t_final, dt)

x = np.linspace(dx/2, L-dx/2, n)
T = np.ones(n)*T0
dTdt = np.empty(n)

Tend_list = np.ones(len(t))*T0
T0_list = np.ones(len(t))*T0
T_x = [T]
dT_list = np.zeros(len(t))
Q = np.ones(len(t))*T0

# absorptivity
alpha_d = 0.4
# 对流系数[W/m^2K]
h = 500
# 激发能量和激发时刻
power_pause = 0
pause_index = np.arange(50,70,1)
laser_power = 0

for j in range(1, len(t)):
    # 边界条件[°C]
    T1s = (alpha_d*power_pause + h*T0 + k/dx*T[1])/(h + k/dx)
    T2s = (h*T0 + k/dx*T[n-2])/(h + k/dx)
    if j in pause_index:
        power_pause = 1000000
        laser_power += power_pause*A
    else:
        power_pause = 0
    for i in range(1, n-1):
        dTdt[i] = alpha*(-(T[i]-T[i-1])/dx**2+(T[i+1]-T[i])/dx**2)
    dTdt[0]= alpha*(-(T[0]-T1s)/dx**2+(T[1]-T[0])/dx**2)
    dTdt[n-1]= alpha*(-(T[n-1]-T[n-2])/dx**2+(T2s-T[n-1])/dx**2)
    T = T + dTdt*dt
    T0_list[j] = T[0]
    Tend_list[j] = T[-1]
    dT_list[j] = T[0] - T[-1]
    Q[j] = C*(T[0] - T[-1])
    T_x.append(T)

fig, ax = plt.subplots()
line, = ax.plot(x, T_x[0])
ax.set_xlabel('x/(m)')
ax.set_ylabel('Temperature(C)')
ax.set_ylim([10, 200])

def init():
    line.set_ydata(T_x[0])
    return line, 

def func_animate(i):
    line.set_ydata(T_x[i])
    return line, 

ani = animation.FuncAnimation(fig, func_animate, np.arange(1, len(T_x)), init_func=init, interval=20, blit=True)
ani.save('D:\\LZP_HP\\yanjiusheng\\TX\\DoD\\aerobench\\AI_F16\\Code\\Test\\view_results\\Temperature.gif', dpi=100, writer='imagemagick')
plt.show()

print('激发能量值{}J'.format(laser_power))
print('散热量{}J'.format(sum(Q[pause_index])))

plt.figure(1)
plt.plot(t, T0_list)
plt.plot(t, Tend_list)
plt.legend(['T_FB','T_RB'])
plt.xlabel('t(s)')
plt.ylabel('Temperature(C)')
plt.show()
plt.figure(2)
plt.plot(t, Q)
plt.xlabel('t(s)')
plt.ylabel('Thermal(J)')
plt.show()




    
