import serial
import struct
import numpy as np
from decimal import Decimal, ROUND_HALF_UP
import decimal
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# 创建全局变量用于存储数据
data_list = [0] * 10
# 创建锁，用于确保数据的安全访问
data_lock = threading.Lock()


def read_serial_data():
    global data_list
    ser = serial.Serial('COM8', baudrate=921600, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    while True:
        data = bytearray()
        while True:
            byte = ser.read(1)
            if byte == b'\x8A':
                break
            data.extend(byte)

        start_index = data.find(b'\x01')
        if start_index != -1:
            for i in range(1, 10):
                data_to_convert = data[start_index + 2 + (i - 1) * 4:start_index + 6 + (i - 1) * 4]
                try:
                    value = struct.unpack('<f', data_to_convert)[0]
                    float_value = Decimal(str(value)).quantize(Decimal('0.00'), rounding=ROUND_HALF_UP)
                    temp = float_value
                except (struct.error, decimal.InvalidOperation) as e:
                    # 处理异常，例如跳过这个数据点或记录错误日志
                    # print(f"异常发生: {e}")
                    continue  # 跳过这个数据点

                with data_lock:
                    data_list[i] = temp
            #     print(f"数据{i}:", float_value, end=' ')
            # print()  # 换行


def update_plot(frame):
        with data_lock:
            if data_list[8] != data_list[9]:

                # ax.cla()
                ax.clear()
                for text in fig.texts:
                    text.remove()
                for text in fig.texts:
                    text.remove()
                for text in fig.texts:
                    text.remove()
                for text in fig.texts:
                    text.remove()
                # 绘制坐标轴箭头
                origin = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
                final = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]]

                # 定义四个点的坐标
                points = np.array([[11.0, 11.0, -float(data_list[1])],
                                   [11.0, -11.0, -float(data_list[2])],
                                   [-11.0, -11.0, -float(data_list[3])],
                                   [-11.0, 11.0, -float(data_list[4])]])

                final, normal = update_drone(final, points, float(data_list[9]), float(data_list[8]))

                ax.quiver(*origin[0], *final[0], color='b', label='X-Axis')
                ax.quiver(*origin[1], *final[1], color='g', label='Y-Axis')
                ax.quiver(*origin[2], *final[2], color='r', label='Z-Axis')

                # 提取坐标
                x = points[:, 0]
                y = points[:, 1]
                z = points[:, 2]
                ax.plot_trisurf(x, y, z, color='red')

                # 创建一些坐标点，以便绘制平面
                xx, yy = np.meshgrid(range(-15, 15), range(-15, 15))
                zz = (-normal[0] * xx - normal[1] * yy) / normal[2] - float(data_list[7])

                # 绘制平面
                ax.plot_surface(xx, yy, zz, alpha=0.5)

                texts = [f"Angle of the downward plane:         {data_list[5]} °"
                         ,f"Angle of rotation required:               {data_list[6]} °"
                         , f"Pitch angle:                                   {data_list[8]} °"
                         , f"Roll angle:                                     {data_list[9]} °"
                         ,f"Average distance from the plane:    {data_list[7]} cm"
                         ,f"Distance 1 from the plane:              {data_list[1]} cm"
                         ,f"Distance 2 from the plane:              {data_list[2]} cm"
                         ,f"Distance 3 from the plane:              {data_list[3]} cm"
                         ,f"Distance 4 from the plane:              {data_list[4]} cm"]

                positions = [(0.05, 0.95), (0.05, 0.92), (0.05, 0.89), (0.05, 0.86), (0.05, 0.83), (0.05, 0.80), (0.05, 0.77), (0.05, 0.74), (0.05, 0.71)]
                for text, position in zip(texts, positions):
                    fig.text(position[0], position[1], text, fontsize=12, color='green', transform=fig.transFigure)

                # 添加标签
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                # 设置坐标轴范围和网格
                ax.set_xlim(-40, 40)
                ax.set_ylim(-40, 40)
                ax.set_zlim(-100, 0)

                # print(data_list[1])

                # 显示图例
                ax.legend()


def update_drone(original_attitude, points, delta_pitch, delta_roll):
    # 将角度转换为弧度
    delta_pitch = np.radians(delta_pitch)
    delta_roll = np.radians(delta_roll)
    # 计算绕X轴和绕Y轴的旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(delta_pitch), -np.sin(delta_pitch)],
                    [0, np.sin(delta_pitch), np.cos(delta_pitch)]])

    R_y = np.array([[np.cos(delta_roll), 0, np.sin(delta_roll)],
                    [0, 1, 0],
                    [-np.sin(delta_roll), 0, np.cos(delta_roll)]])

    # 计算新的姿态信息
    new_attitude = np.dot(R_y, np.dot(R_x, original_attitude))

    vectorA = points[1]-points[0]
    vectorB = points[2]-points[0]
    n = np.cross(vectorA, vectorB)
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(-delta_pitch), -np.sin(-delta_pitch)],
                    [0, np.sin(-delta_pitch), np.cos(-delta_pitch)]])

    R_y = np.array([[np.cos(-delta_roll), 0, np.sin(-delta_roll)],
                    [0, 1, 0],
                    [-np.sin(-delta_roll), 0, np.cos(-delta_roll)]])
    attitude = np.dot(R_y, np.dot(R_x, n))

    return new_attitude, attitude


# 创建并启动串口通信线程
serial_thread = threading.Thread(target=read_serial_data)
serial_thread.start()

# 创建3D图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 使用FuncAnimation来更新3D图
ani = FuncAnimation(fig, update_plot, frames=None, interval=100, cache_frame_data=False)


# 显示图形
plt.show()

# 等待串口通信线程结束
serial_thread.join()
