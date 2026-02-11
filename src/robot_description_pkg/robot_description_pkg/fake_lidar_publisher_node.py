import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeLidarPublisherNode(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher_node')

        self.publisher_ = self.create_publisher(LaserScan, 'scan_fake', 10)

        # 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Fake Lidar Publisher iniciado, publicando em /scan')

        # Configuração do "sensor"
        self.angle_min = -math.pi / 2.0   # -90°
        self.angle_max =  math.pi / 2.0   # +90°
        self.angle_increment = math.radians(1.0)  # 1° por amostra
        self.range_min = 0.1
        self.range_max = 10.0

        self.num_readings = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        )

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'  # exatamente o nome do link no URDF

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        ranges = []
        intensities = []

        # Cenário simples: "parede" a 2m entre -30° e +30°, resto vazio (max range)
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment

            if -math.radians(30) < angle < math.radians(30):
                ranges.append(2.0)
            else:
                ranges.append(self.range_max)

            intensities.append(100.0)

        scan.ranges = ranges
        scan.intensities = intensities

        self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
