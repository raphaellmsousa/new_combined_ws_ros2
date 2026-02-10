import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math
import time

# Nome da classe consistente com o nome do arquivo e o executável
class SimpleOdomPublisherNode(Node):
    def __init__(self):
        # Nome do nó ROS 2 consistente com o executável e o launch file
        super().__init__('simple_odom_publisher_node')
        self.get_logger().info('Simple Odometry Publisher Node started (circular motion).')

        # --- Parâmetros de Movimento Circular ---
        self.linear_speed = 0.2  # m/s (velocidade linear constante)
        self.angular_speed = 0.5 # rad/s (velocidade angular constante)

        # --- Publicadores e Broadcast de TF ---
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Variáveis de Estado da Odometria ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # --- Timer para Publicação ---
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if abs(self.angular_speed) < 1e-6:
            self.x += self.linear_speed * math.cos(self.theta) * dt
            self.y += self.linear_speed * math.sin(self.theta) * dt
        else:
            R = self.linear_speed / self.angular_speed
            icc_x = self.x - R * math.sin(self.theta)
            icc_y = self.y + R * math.cos(self.theta)

            self.x = icc_x + (self.x - icc_x) * math.cos(self.angular_speed * dt) - \
                     (self.y - icc_y) * math.sin(self.angular_speed * dt)
            self.y = icc_y + (self.x - icc_x) * math.sin(self.angular_speed * dt) + \
                     (self.y - icc_y) * math.cos(self.angular_speed * dt)
            self.theta += self.angular_speed * dt

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.linear_speed
        odom_msg.twist.twist.angular.z = self.angular_speed

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomPublisherNode() # Instancia a classe com o nome correto
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

