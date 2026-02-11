#!/bin/bash

# --- Configurações ---
# Caminho para a instalação base do ROS 2 Jazzy
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"

# Caminho para o setup do seu workspace ROS 2
WORKSPACE_SETUP="$HOME/new_combined_ws/install/setup.bash"

# Nome do pacote do RPLIDAR
RPLIDAR_PACKAGE="rplidar_ros"

# Nome do launch file que inicia o RPLIDAR e o RViz2
# (usaremos o que criamos anteriormente: rplidar_with_rviz_launch.py)
RPLIDAR_LAUNCH_FILE="rplidar_with_rviz_launch.py"

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
    # Não saímos aqui, pois o ROS 2 base pode ser suficiente para alguns comandos,
    # mas para o launch file do seu workspace, ele é crucial.
fi

echo "--- Iniciando RPLIDAR e RViz2 via launch file ---"
echo "Comando: ros2 launch $RPLIDAR_PACKAGE $RPLIDAR_LAUNCH_FILE"
ros2 launch "$RPLIDAR_PACKAGE" "$RPLIDAR_LAUNCH_FILE"

# Verifica o código de saída do comando launch
if [ $? -eq 0 ]; then
    echo "RPLIDAR e RViz2 iniciados com sucesso."
else
    echo "ERRO: Falha ao iniciar RPLIDAR e RViz2."
fi
