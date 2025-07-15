import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer # 坐标监听器
from tf_transformations import quaternion_from_euler # 欧拉角转四元数函数
import os
import time
# 语音服务接口
from autopatrol_interfaces.srv import SpeechText
# 用于ROS图像消息
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    """
    机器人自主巡逻节点
    从配置文件中读取waypoints, 并且依次导航到每个点
    到达每个点后播报一段语音, 并且记录此处的图像
    """
    def __init__(self):
        super().__init__('patrol_node')

        # 声明相关参数, 之后yaml配置文件直接launch
        self.declare_parameter('initial_point', [0.0,0.0,0.0])
        self.declare_parameter('target_points', [0.0,0.0,0.0, 1.0,1.0,1.57])
        self.declare_parameter('img_save_path', '')
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.img_save_path_ = self.get_parameter('img_save_path').value

        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        self.speech_client_ = self.create_client(SpeechText, 'speech_text')
        self.cv_bridge_ = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.img_callback, 1)

    def img_callback(self, msg):
        self.latest_img_ = msg

    def record_img(self):
        if self.latest_img_ is not None:
            pose = self.get_current_pose()
            self.img_save_path_ = self.get_parameter('img_save_path').value
            
            # 检查目录是否存在, 如果不存在则创建它
            try:
                os.makedirs(self.img_save_path_, exist_ok=True)
            except OSError as e:
                self.get_logger().error(f'创建图片保存目录失败: {e}')
                return # 若创建失败, 则无法继续, 直接返回
            
            cv_image = self.cv_bridge_.imgmsg_to_cv2(self.latest_img_, 'bgr8') # ROS图像转为CV图像
            
            # 生成完整的文件路径
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f'img_{timestamp}_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.jpg'
            full_path = os.path.join(self.img_save_path_, filename)

            cv2.imwrite(full_path, cv_image)

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        return PoseStamped对象
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 返回顺序是x,y,z,w
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active() # 等待导航可用

    def get_target_points(self):
        """
        通过参数值获取目标点集合
        """
        points =[]
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'获取到目标点{index}->{x},{y},{yaw}')
        return points

    def nav_to_pose(self, target_point):
        """
        导航点到目标
        """    
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f'剩余距离: {feedback.distance_remaining}')
        result = self.getResult()
        self.get_logger().info(f'导航结果: {result}')


    def get_current_pose(self):
        """
        获取当前机器人位置
        """
        while rclpy.ok():
            try:
                result = self.buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
                t = result.transform
                self.get_logger().info(f'平移:{t.translation}')
                return t
            except Exception as e:
                self.get_logger().warn(f'获取坐标变化失败,原因:{str(e)}')

    def speech_text(self, text):
        """
        调用语音服务来播放指定的文本
        这是一个同步调用, 会等待服务完成
        """
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待语音播放服务可用...')
        
        # 构造请求
        request = SpeechText.Request()
        request.text = text

        # 异步调用
        future = self.speech_client_.call_async(request)
        # 使用 spin_until_future_complete 等待服务调用完成, 这会阻塞当前函数, 直到收到响应或超时
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            response = future.result()
            if response.result:
                self.get_logger().info(f'语音合成成功:{text}')
            else:
                self.get_logger().warn(f'语音合成失败:{text}')
        else:
            self.get_logger().warn('语音合成服务响应失败')

def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text('Initializing position.')
    # rclpy.spin(patrol)
    patrol.init_robot_pose()
    patrol.speech_text('Initialization complete.')

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x,y,yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x,y,yaw)
            patrol.speech_text(f'Navigate to the target point at {x},{y}.')
            patrol.nav_to_pose(target_pose)
            patrol.speech_text(f'Reach target point {x},{y}, Preparing to capture an image.')
            patrol.record_img()
            patrol.speech_text(f'Image capture complete.')
            time.sleep(1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()