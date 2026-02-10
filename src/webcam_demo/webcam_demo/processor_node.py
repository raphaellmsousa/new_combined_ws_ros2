# FILE NAME: processor_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        # Assina o tópico da câmera raw
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.br = CvBridge()

        # NOVO: Publicador para a imagem processada (tons de cinza)
        self.processed_publisher_ = self.create_publisher(Image, 'camera/image_processed', 10) # <--- ADICIONE ESTA LINHA

        self.get_logger().info('Nó do processador iniciado, assinando camera/image_raw e publicando em camera/image_processed')

    def listener_callback(self, data):
        self.get_logger().info('Recebendo frame e convertendo para cinza')
        current_frame = self.br.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Publica a imagem processada no novo tópico
        processed_msg = self.br.cv2_to_imgmsg(gray_frame, encoding="mono8") # 'mono8' para imagens em tons de cinza
        self.processed_publisher_.publish(processed_msg) # <--- ADICIONE ESTA LINHA

        # Exibe localmente em uma janela do OpenCV (opcional, mas útil para depuração)
        cv2.imshow("Webcam em Tons de Cinza", gray_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando nó do processador...')
    finally:
        cv2.destroyAllWindows() # Garante que a janela do OpenCV seja fechada
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

