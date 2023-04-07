import numpy as np
import matplotlib.pyplot as plt

plt.ioff()
# 保存历史绘制数据的列表
history_x = []
history_y = []
def plot_arc(center_x, center_y, radius, start_x, start_y, end_x, end_y):
    # 生成圆弧上的点
    start_angle = np.arctan2(start_y - center_y, start_x - center_x)
    end_angle = np.arctan2(end_y - center_y, end_x - center_x)
    angle = np.linspace(start_angle, end_angle, 100)  # 可以调整点的数量
    x = center_x + radius * np.cos(angle)
    y = center_y + radius * np.sin(angle)
    # print(x)

    # 绘制起始点和终点
    plt.plot(start_x, start_y, 'ro', label='Start')
    plt.plot(end_x, end_y, 'go', label='End')
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
    # print(len(history_x))
    # for i in len(history_x):
    
    # plt.show()
    # print(x)

    # plt.legend()
    # 强制绘图窗口重新绘制
    # plt.show()

# 读取文件
with open('curve.txt', 'r') as file:
    
    for line in file:
        # 解析每一行的圆弧信息
        info = line.strip().split(' ')
        
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
# 显示图形
plt.plot(history_x, history_y)
        # print(center_x, " ", center_y, " ", radius, " ", start_x, " ", start_y, " ", end_x, " ", end_y)
plt.show()
plt.close()
