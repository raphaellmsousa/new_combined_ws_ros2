# FILE NAME: display_robot.launch.py
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

    # Caminho para o arquivo URDF/XACRO do seu robô
    urdf_file_path = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf.xacro')

    # Caminho para o arquivo de configuração do RViz
    rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'robot_display.rviz')

    # Argumento para usar o joint_state_publisher_gui
    use_jsp_gui = LaunchConfiguration('use_jsp_gui', default='true')

    # Argumento para usar o RViz
    use_rviz = LaunchConfiguration('use_rviz', default='true')

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
    # Usa o executável 'simple_odom' e o nome do nó ROS 2 'simple_odom_publisher_node'
    simple_odom_publisher_node = Node(
        package=package_name,
        executable='simple_odom', # <--- Executável conforme definido em setup.py
        name='simple_odom_publisher_node',  # <--- Nome do nó ROS 2
        output='screen',
        parameters=[
            # Você pode adicionar parâmetros aqui se o seu nó de odometria precisar
        ]
    )

    # REMOVA COMPLETAMENTE A DEFINIÇÃO DO NÓ DA CÂMERA FAKE
    # camera_publisher_node = Node(
    #     package=package_name,
    #     executable='camera_publisher',
    #     name='camera_publisher_node',
    #     output='screen',
    #     parameters=[]
    # )
    
   # Lidar simulado - CORREÇÃO AQUI: Atribuir a uma variável
    fake_lidar_publisher_node = Node( # <--- ATRIBUÍDO A UMA VARIÁVEL
        package='robot_description_pkg',
        executable='fake_lidar',
        name='fake_lidar_publisher_node',
        output='screen',
    )
    
    # NOVO: Nó para a câmera real do pacote webcam_demo
    real_camera_node = Node(
        package='webcam_demo', # <--- Pacote da câmera real
        executable='camera',   # <--- Executável da câmera real (definido em webcam_demo/setup.py)
        name='camera_node',    # <--- Nome do nó ROS 2 da câmera real
        output='screen',
        parameters=[
            # Se o seu nó de câmera real precisar de parâmetros (ex: device_id, resolução), adicione aqui
            # Exemplo: {'device_id': 0, 'frame_width': 640, 'frame_height': 480}
        ]
    )
    
    # NOVO: Nó para processar a imagem da câmera real
    processor_node = Node(
        package='webcam_demo', # <--- Pacote do processador
        executable='processor', # <--- Executável do processador (definido em webcam_demo/setup.py)
        name='processor_node',  # <--- Nome do nó ROS 2 do processador
        output='screen',
        parameters=[
            # Quaisquer parâmetros que o nó do processador possa precisar
        ]
    )

    # ESTE É O ÚNICO RETURN LaunchDescription. Ele inclui TODOS os argumentos e TODOS os nós.
    return LaunchDescription([
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
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        simple_odom_publisher_node,
        fake_lidar_publisher_node,
        # REMOVA ESTA LINHA: camera_publisher_node,
        real_camera_node, # <--- ADICIONE ESTA LINHA para lançar a câmera real
        processor_node,
    ])

