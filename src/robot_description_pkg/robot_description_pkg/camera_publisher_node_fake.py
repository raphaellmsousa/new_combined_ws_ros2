import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.get_logger().info('Camera Publisher Node started.')

        self.publisher_ = self.create_publisher(Image, 'camera_fake', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # Publica a 10 Hz
        self.bridge = CvBridge()
        self.image_counter = 0

    def timer_callback(self):
        # Cria uma imagem de teste simples (por exemplo, um gradiente ou uma cor sólida)
        # Para este exemplo, vamos criar uma imagem com um gradiente de cor
        width, height = 320, 240
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Desenha um gradiente simples para mostrar que a imagem está mudando
        for i in range(height):
            img[i, :, 0] = int(255 * (i / height)) # Canal Azul
            img[i, :, 1] = int(255 * (1 - (i / height))) # Canal Verde
            img[i, :, 2] = 100 # Canal Vermelho constante

        # Adiciona um texto para indicar que é uma imagem de teste e um contador
        cv2.putText(img, f"Camera Test Image {self.image_counter}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        self.image_counter += 1

        # Converte a imagem OpenCV para uma mensagem ROS 2 Image
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link' # Importante: o frame da câmera no URDF

        self.publisher_.publish(img_msg)
        # self.get_logger().info('Publishing camera image')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    try:
        rclpy.spin(camera_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

