import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

fig, ax = plt.subplots()
# 设置图形的标题和坐标轴标签
ax.set_title('Wedge')
ax.set_xlabel('X')
ax.set_ylabel('Y')

# 设置扇形的中心点和半径
center = (0, 0)
radius = 1
x = []
y = []

# 绘制扇形
wedge, = ax.fill(x, y, color='blue')

def animate(num):
    ax.clear()
    num += 2
    # 设置扇形的起始角度和结束角度
    theta1, theta2 = num, num + 45

    # 生成扇形的数据
    theta = np.linspace(theta1, theta2, 100)
    x = np.append(center[0], radius * np.cos(np.radians(theta)))
    y = np.append(center[1], radius * np.sin(np.radians(theta)))

    wedge, = ax.fill(x, y, color='blue')
    
    return wedge,

ani = animation.FuncAnimation(fig, animate, frames=50 // 2, interval=60)
plt.show()