import rclpy 
from rclpy.node import Node
from rclpy.time import Time, Duration
from tf2_ros import TransformListener, Buffer # 坐标监听器
from tf_transformations import euler_from_quaternion # 欧拉角转四元数函数

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = Buffer() # 存储从 TF 系统中接收到的坐标变换数据
        self.listener_ = TransformListener(self.buffer_, self)  # 持续监听 TF 广播，将数据写入 Buffer
        self.timer_ = self.create_timer(1.0, self.get_transfrom)    # 每 1 秒调用一次 get_transfrom 方法

    def get_transfrom(self):
        """
        实时查询坐标关系
        result.header.frame_id       → 'map'
        result.child_frame_id        → 'base_footprint'
        result.transform.translation → 平移向量(x, y, z)
        result.transform.rotation    → 四元数(x, y, z, w)
        """
        try:
            result = self.buffer_.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=1.0))
            t = result.transform
            self.get_logger().info(f'平移:{t.translation}')
            self.get_logger().info(f'旋转:{t.rotation}')
            rotation_euler = euler_from_quaternion([
                t.rotation.x,
                t.rotation.y,
                t.rotation.z,
                t.rotation.w
            ])
            self.get_logger().info(f'旋转rpy:{rotation_euler}')
        except Exception as e:
            self.get_logger().warn(f'获取坐标变化失败,原因:{str(e)}')

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()