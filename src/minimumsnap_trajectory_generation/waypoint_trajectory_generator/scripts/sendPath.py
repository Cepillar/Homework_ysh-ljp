import rospy
import random
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import time
def calculate_path_length(path_points):
    """
    计算路径点的总长度
    :param path_points: 路径点列表
    :return: 总路径长度
    """
    length = 0.0
    for i in range(1, len(path_points)):
        x1, y1 = path_points[i-1].pose.position.x, path_points[i-1].pose.position.y
        x2, y2 = path_points[i].pose.position.x, path_points[i].pose.position.y
        length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return length

def generate_random_path(x_range, y_range, num_points, target_length):
    """
    根据目标路径长度生成随机路径点
    :param x_range: x轴范围 (min, max)
    :param y_range: y轴范围 (min, max)
    :param num_points: 路径点数目
    :param target_length: 目标路径长度
    :return: 路径点列表
    """
    # 生成随机的路径点
    path_points = []
    for _ in range(num_points):
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        z = 0  # 假设z轴高度在0到2之间
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        path_points.append(pose)

    # 计算路径的初始长度
    current_length = calculate_path_length(path_points)
    
    # 根据目标路径长度调整路径点间的间距
    scaling_factor = target_length / current_length if current_length != 0 else 1.0
    
    # 调整路径点位置
    for i in range(1, len(path_points)):
        x1, y1 = path_points[i-1].pose.position.x, path_points[i-1].pose.position.y
        x2, y2 = path_points[i].pose.position.x, path_points[i].pose.position.y
        
        # 计算当前点与前一个点的距离
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
        # 根据缩放因子调整点的位置
        scale_x = (x2 - x1) * scaling_factor
        scale_y = (y2 - y1) * scaling_factor
        
        path_points[i].pose.position.x = x1 + scale_x
        path_points[i].pose.position.y = y1 + scale_y

    # 再次计算路径长度，确保接近目标长度
    final_length = calculate_path_length(path_points)
    rospy.loginfo(f"Final path length: {final_length:.2f}")
    
    return path_points

def send_path(path_points):
    """
    发送路径点到ROS的/path话题
    :param path_points: 路径点列表
    """
    # 初始化ROS节点
    
    
    # 创建Path消息
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"  # 可以根据实际情况修改frame_id
    
    # 将路径点添加到Path消息中
    path_msg.poses = path_points
    
    # 创建Publisher，发布路径点到/path话题
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    
    # 等待连接并发布路径
    rospy.sleep(1)  # 等待1秒钟，确保Publisher已连接
    rospy.loginfo("Publishing random path...")
    path_pub.publish(path_msg)
    # rospy.spin()

if __name__ == '__main__':
    try:
        # 指定x, y范围和路径点数量
        x_range = (-10, 10)
        y_range = (-10, 10)
        num_points = 10
        rospy.init_node('random_path_publisher', anonymous=True)
        # 生成随机路径点
        for i in range(1,100,10):
            path_points = generate_random_path(x_range, y_range, max(num_points,i),i)

            # 发送路径点到ROS
            send_path(path_points)
            # time.sleep(2.0)
    except rospy.ROSInterruptException:
        pass