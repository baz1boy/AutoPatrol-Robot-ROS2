from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator()

    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0 # 其他orientation x,y,z 默认为0, 表示朝正前方
    
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # 等待导航可用
    
    goal_poses = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = nav.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 1.0
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = nav.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.0
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)

    nav.followWaypoints(goal_poses)
    
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f'路点编号: {feedback.current_waypoint}')
        # nav.cancelTask()
    result = nav.getResult()
    nav.get_logger().info(f'导航结果: {result}')

    # rclpy.spin(nav)
    # rclpy.shutdown()
