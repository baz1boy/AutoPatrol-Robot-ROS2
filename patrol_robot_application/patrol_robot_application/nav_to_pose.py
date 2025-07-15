from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

# 单点导航
def main():
    rclpy.init()
    nav = BasicNavigator() # 创建导航控制器对象
  
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0 # 其他orientation x,y,z 默认为0, 表示朝正前方
    
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # 等待导航可用
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    
    nav.goToPose(goal_pose)
    
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f'剩余距离: {feedback.distance_remaining}')
        # nav.cancelTask()
    result = nav.getResult()
    nav.get_logger().info(f'导航结果: {result}')

    # rclpy.spin(nav)
    # rclpy.shutdown()
