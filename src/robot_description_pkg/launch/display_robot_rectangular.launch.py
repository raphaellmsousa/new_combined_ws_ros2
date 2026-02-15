# FILE NAME: display_robot_rect.launch.py
import os
from ament_index_python.packages import get_package_share_directory
import launch # Importa o módulo launch para usar launch.conditions.IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue # Para usar Command

def generate_launch_description():
    # Nome do seu pacote
    package_name = 'robot_description_pkg'

    # Caminho para o diretório share do seu pacote
    pkg_share_dir = get_package_share_directory(package_name)

    # Caminho para o arquivo URDF/XACRO do seu robô retangular
    urdf_file_path = os.path.join(pkg_share_dir, 'urdf', 'my_rectangular_robot.urdf.xacro') # <--- AQUI ESTÁ A MUDANÇA

    # Caminho para o arquivo de configuração do RViz
    rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'robot_display.rviz')

    # --- Argumentos de Launch para controle de sensores e RViz ---
    # Estes são os valores que o launch file espera receber
    use_jsp_gui     = LaunchConfiguration('use_jsp_gui',     default='true')
    use_rviz        = LaunchConfiguration('use_rviz',        default='true')
    use_fake_lidar  = LaunchConfiguration('use_fake_lidar',  default='true')
    use_fake_camera = LaunchConfiguration('use_fake_camera', default='false')
    use_real_camera = LaunchConfiguration('use_real_camera', default='true')

    # Processa o arquivo XACRO para gerar o URDF final
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    # Nó para publicar as transformações do robô (TFs)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Nó para controlar as juntas do robô via GUI (útil para depuração do URDF)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(use_jsp_gui)
    )

    # Nó do RViz2 para visualização
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path], # Carrega o arquivo de configuração do RViz
        condition=launch.conditions.IfCondition(use_rviz)
    )

    # Nó de odometria simplificado (movimento circular)
    simple_odom_publisher_node = Node(
        package=package_name,
        executable='simple_odom',
        name='simple_odom_publisher_node',
        output='screen',
        parameters=[]
    )

    # Nó da câmera fake
    fake_camera_publisher_node = Node(
        package=package_name,
        executable='fake_camera_publisher',
        name='fake_camera_publisher_node',
        output='screen',
        parameters=[],
        remappings=[
            ('/camera_fake', '/fake_camera/image_raw'),
        ],
        condition=launch.conditions.IfCondition(use_fake_camera)
    )

    # Lidar simulado
    fake_lidar_publisher_node = Node(
        package='robot_description_pkg',
        executable='fake_lidar',
        name='fake_lidar_publisher_node',
        output='screen',
        condition=launch.conditions.IfCondition(use_fake_lidar)
    )

    # Nó para a câmera real do pacote webcam_demo
    real_camera_node = Node(
        package='webcam_demo',
        executable='camera',
        name='camera_node',
        output='screen',
        parameters=[],
        condition=launch.conditions.IfCondition(use_real_camera)
    )

    # Nó para processar a imagem da câmera real
    processor_node = Node(
        package='webcam_demo',
        executable='processor',
        name='processor_node',
        output='screen',
        parameters=[],
        condition=launch.conditions.IfCondition(use_real_camera)
    )

    # ESTE É O ÚNICO RETURN LaunchDescription. Ele inclui TODOS os argumentos e TODOS os nós.
    return LaunchDescription([
        # Declaração dos argumentos de launch
        DeclareLaunchArgument(
            'use_jsp_gui',
            default_value='true',
            description='Set to "true" to launch joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Set to "true" to launch RViz2'
        ),
        DeclareLaunchArgument(
            'use_fake_lidar',
            default_value='true',
            description='Set to "true" to use fake lidar; false to use real lidar (if configured)'
        ),
        DeclareLaunchArgument(
            'use_fake_camera',
            default_value='false',
            description='Set to "true" to launch fake camera'
        ),
        DeclareLaunchArgument(
            'use_real_camera',
            default_value='true',
            description='Set to "true" to launch real camera'
        ),

        # Inclusão dos nós (com suas condições)
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        simple_odom_publisher_node,
        fake_lidar_publisher_node,
        fake_camera_publisher_node,
        real_camera_node,
        processor_node,
    ])
