import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Publicador de imagem
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        # Timer para captura (10 FPS para manter estabilidade)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Inicializa a captura da webcam
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        # Nome da janela fixo para evitar múltiplas instâncias
        self.window_name = "Transmitindo: Imagem Original"
        self.get_logger().info('Nó da câmera iniciado com visualização local!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 1. Publica para o ROS (para o nó processor ler)
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

            # 2. Exibe localmente em uma única janela fixa
            cv2.imshow(self.window_name, frame)

            # 3. Processa frames (1ms é suficiente para manter o fluxo)
            cv2.waitKey(1)

            self.get_logger().info('Frame publicado e exibido')
        else:
            self.get_logger().warn('Falha ao capturar frame da webcam')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando nó da câmera...')
    finally:
        # Garante o fechamento correto dos recursos
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

