import matplotlib.pyplot as plt

# 轨迹生成时间和轨迹长度数据
time_consumed = [1.792505, 4.695244, 20.182650, 22.005418, 21.966697, 44.610600, 63.963041, 85.703577, 113.669435, 157.089192]

trajectory_length = [1.369165, 10.978164, 20.608061, 24.143233, 37.702294, 42.651028, 47.613271, 57.205295, 69.037826, 78.317236]

# 绘制折线图
plt.plot(trajectory_length, time_consumed, marker='o', linestyle='-', color='b')

# 设置图表标题和标签
# plt.title('Time vs. Trajectory Length')
plt.xlabel('Trajectory Length (m)')
plt.ylabel('Time Consumed (ms)')

# 显示网格
plt.grid(False)

# 展示图表
plt.show()