points = []
with open('/home/thj/Desktop/TestProject/test_curvefitting/newfitting/trajectory.txt', 'r') as f:
    for line in f:
        x, y, angle = map(float, line.strip().split())
        points.append((x, y, angle))

import matplotlib.pyplot as plt
import numpy as np

x = [p[0] for p in points]
y = [p[1] for p in points]
angle = [p[2] for p in points]

fig, ax = plt.subplots()
ax.plot(x, y, 'b-', label='trajectory')

# 绘制箭头
dx = np.cos(angle)
dy = np.sin(angle)
# ax.quiver(x, y, dx, dy)

ax.set_xlabel('x')
ax.set_ylabel('y')


# plt.show()
history_x = []
history_y = []
inflection_points_x = []
inflection_points_y = []
def plot_arc(center_x, center_y, radius, start_x, start_y, end_x, end_y):
    # 生成圆弧上的点
    start_angle = np.arctan2(start_y - center_y, start_x - center_x)
    end_angle = np.arctan2(end_y - center_y, end_x - center_x)
    angle = np.linspace(start_angle, end_angle, 100)  # 可以调整点的数量
    x = center_x + radius * np.cos(angle)
    y = center_y + radius * np.sin(angle)
    # print(x)

    # 绘制起始点和终点
    # ax.plot(start_x, start_y, 'ro')
    # ax.plot(end_x, end_y, 'go')
    # 绘制圆弧
    

    # 绘制圆心
    # plt.plot(center_x, center_y, 'bo', label='Center')

    # 设置坐标轴等比例显示
    plt.axis('equal')
    # 更新历史绘制数据
    # history_x.append(x)
    # history_y.append(y)
    history_x.extend(x)
    history_y.extend(y)


with open('curve.txt', 'r') as file:
    for line in file:
        # 解析每一行的圆弧信息
        info = line.strip().split(' ')
        if(len(info) == 7):
            center_x = float(info[0])
            center_y = float(info[1])
            radius = float(info[2])
            start_x = float(info[3])
            start_y = float(info[4])
            end_x = float(info[5])
            end_y = float(info[6])
            # print("cent ", center_x, " ", center_y)
            # 调用函数绘制圆弧
            plot_arc(center_x, center_y, radius, start_x, start_y, end_x, end_y)
        elif(len(info) == 3):
            inflection_points_x.append(float(info[1]))
            inflection_points_y.append(float(info[2]))
# 显示图形
ax.plot(history_x, history_y,label='curve fit')
        # print(center_x, " ", center_y, " ", radius, " ", start_x, " ", start_y, " ", end_x, " ", end_y)
#显示折点
ax.plot(inflection_points_x, inflection_points_y,'ro', label='inflection_points')
ax.legend()
plt.show()

