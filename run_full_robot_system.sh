#!/bin/bash

# --- Configurações ---
# Caminho para a instalação base do ROS 2 Jazzy
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"

# Caminho para o setup do seu workspace ROS 2
WORKSPACE_SETUP="$HOME/new_combined_ws/install/setup.bash"

# Nome do pacote principal do seu robô
ROBOT_PACKAGE="robot_description_pkg"

# Nome do launch file principal do seu robô (que agora não terá o LiDAR fake)
ROBOT_LAUNCH_FILE="display_robot.launch.py"

# Nome do pacote do RPLIDAR
RPLIDAR_PACKAGE="rplidar_ros"

# Nome do launch file do RPLIDAR (que você já criou, por exemplo, rplidar_a1_launch.py)
# Certifique-se de que este launch file está configurado com o frame_id correto (ex: lidar_link)
RPLIDAR_LAUNCH_FILE="rplidar_a1_launch.py" # Ou rplidar_with_rviz_launch.py se preferir

# Parâmetros para o RPLIDAR (ajuste conforme necessário)
RPLIDAR_SERIAL_PORT="/dev/ttyUSB0"
RPLIDAR_BAUDRATE="115200"
RPLIDAR_FRAME_ID="lidar_link" # <--- MUITO IMPORTANTE: Deve ser o mesmo do seu URDF!

# --- Execução ---

echo "--- Sourciando ambiente ROS 2 Jazzy ---"
if [ -f "$ROS_DISTRO_SETUP" ]; then
    source "$ROS_DISTRO_SETUP"
    echo "Ambiente ROS 2 Jazzy sourciado com sucesso."
else
    echo "ERRO: Arquivo de setup do ROS 2 Jazzy não encontrado em $ROS_DISTRO_SETUP"
    exit 1
fi

echo "--- Sourciando ambiente do workspace ($HOME/new_combined_ws) ---"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
    echo "Ambiente do workspace sourciado com sucesso."
else
    echo "AVISO: Arquivo de setup do workspace não encontrado em $WORKSPACE_SETUP. Certifique-se de que o workspace foi compilado."
    # Não saímos aqui, mas o launch file do seu robô pode falhar se o workspace não estiver sourciado.
fi

echo "--- Compilando o workspace (se houver mudanças) ---"
# Opcional: Adicionar um colcon build aqui para garantir que as últimas mudanças sejam compiladas
# colcon build --symlink-install --packages-up-to $ROBOT_PACKAGE $RPLIDAR_PACKAGE
# Ou um build completo:
# colcon build --symlink-install
# Por simplicidade, vamos assumir que você compila manualmente quando necessário.

echo "--- Iniciando o sistema do robô (odometria, câmera, RViz, etc.) ---"
# O comando ros2 launch é executado em segundo plano (&) para que o script possa continuar
ros2 launch "$ROBOT_PACKAGE" "$ROBOT_LAUNCH_FILE" &
ROBOT_LAUNCH_PID=$! # Salva o PID do processo do launch do robô

echo "--- Iniciando o RPLIDAR real ---"
# Usamos ros2 run aqui para ter mais controle sobre os parâmetros,
# mas você pode usar seu launch file do RPLIDAR se ele já tiver os parâmetros corretos.
ros2 run "$RPLIDAR_PACKAGE" rplidar_node \
  --ros-args \
  -p serial_port:="$RPLIDAR_SERIAL_PORT" \
  -p serial_baudrate:="$RPLIDAR_BAUDRATE" \
  -p frame_id:="$RPLIDAR_FRAME_ID" &
RPLIDAR_NODE_PID=$! # Salva o PID do processo do nó do RPLIDAR

echo "Sistema completo iniciado. Pressione Ctrl+C para encerrar ambos os processos."

# Espera pelos processos em segundo plano para que o script não termine imediatamente
wait $ROBOT_LAUNCH_PID
wait $RPLIDAR_NODE_PID

echo "Todos os processos encerrados."
