#!/bin/bash

# --- Configurações Dinâmicas ---
# Obtém o diretório onde o script está localizado
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# --- Tenta localizar a raiz do workspace dinamicamente ---
# Primeiro, assume que o script está na raiz do workspace
WORKSPACE_ROOT="$SCRIPT_DIR"
WORKSPACE_SETUP="$WORKSPACE_ROOT/install/setup.bash"

# Se o setup.bash não for encontrado aqui, tenta subir um nível (caso o script esteja em um subdiretório como 'scripts/')
if [ ! -f "$WORKSPACE_SETUP" ]; then
    POTENTIAL_WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
    POTENTIAL_WORKSPACE_SETUP="$POTENTIAL_WORKSPACE_ROOT/install/setup.bash"
    if [ -f "$POTENTIAL_WORKSPACE_SETUP" ]; then
        WORKSPACE_ROOT="$POTENTIAL_WORKSPACE_ROOT"
        WORKSPACE_SETUP="$POTENTIAL_WORKSPACE_SETUP"
    else
        # Se ainda não encontrou, pode tentar subir mais níveis ou emitir um erro.
        # Por enquanto, vamos manter a lógica de erro mais abaixo.
        echo "AVISO: Não foi possível encontrar 'install/setup.bash' em '$SCRIPT_DIR' nem em '$POTENTIAL_WORKSPACE_ROOT'."
        echo "Por favor, verifique a estrutura do seu workspace ou a localização do script."
    fi
fi

# Caminho para a instalação base do ROS 2 Jazzy
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"

# Nome do pacote principal do seu robô
ROBOT_PACKAGE="robot_description_pkg"

# Nome do launch file principal do seu robô
ROBOT_LAUNCH_FILE="display_robot.launch.py"

# Nome do pacote do RPLIDAR
RPLIDAR_PACKAGE="rplidar_ros"

# Parâmetros para o RPLIDAR (ajuste conforme necessário)
RPLIDAR_SERIAL_PORT="/dev/ttyUSB0"
RPLIDAR_BAUDRATE="115200"
RPLIDAR_FRAME_ID="lidar_link"

# --- Argumentos de Linha de Comando para o Script ---
# Estes serão os valores padrão se não forem passados ao script
USE_FAKE_LIDAR="true" # Caso true, o lidar real não vai rodar
USE_FAKE_CAMERA="true"
USE_REAL_CAMERA="true"
USE_RVIZ="true" # Adicionado para controle do RViz
USE_JSP_GUI="true" # Reintroduzido para controle do joint_state_publisher_gui
ROBOT_MODEL="default" # Adicionado para selecionar o modelo do robô

# Processa os argumentos passados para este script
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --fake-lidar) USE_FAKE_LIDAR="true" ;;
        --no-fake-lidar) USE_FAKE_LIDAR="false" ;;
        --fake-camera) USE_FAKE_CAMERA="true" ;;
        --no-fake-camera) USE_FAKE_CAMERA="false" ;;
        --real-camera) USE_REAL_CAMERA="true" ;;
        --no-real-camera) USE_REAL_CAMERA="false" ;;
        --rviz) USE_RVIZ="true" ;;
        --no-rviz) USE_RVIZ="false" ;;
        --jsp-gui) USE_JSP_GUI="true" ;;
        --no-jsp-gui) USE_JSP_GUI="false" ;;
        --robot-model=*) ROBOT_MODEL="${1#*=}" ;;
        *) echo "Uso: $0 [--fake-lidar|--no-fake-lidar] [--fake-camera|--no-fake-camera] [--real-camera|--no-real-camera] [--rviz|--no-rviz] [--jsp-gui|--no-jsp-gui] [--robot-model=default|4_wheels]"; exit 1 ;;
    esac
    shift
done

# --- Seleciona o launch file com base no modelo do robô ---
if [ "$ROBOT_MODEL" = "4_wheels" ]; then
    ROBOT_LAUNCH_FILE="display_robot_4_wheels.launch.py"
    echo "--- Usando o modelo de robô de 4 rodas ---"
else
    ROBOT_LAUNCH_FILE="display_robot.launch.py"
    echo "--- Usando o modelo de robô padrão (2 rodas + castor) ---"
fi

# --- Execução ---

echo "--- Sourciando ambiente ROS 2 Jazzy ---"
if [ -f "$ROS_DISTRO_SETUP" ]; then
    source "$ROS_DISTRO_SETUP"
    echo "Ambiente ROS 2 Jazzy sourciado com sucesso."
else
    echo "ERRO: Arquivo de setup do ROS 2 Jazzy não encontrado em $ROS_DISTRO_SETUP"
    exit 1
fi

echo "--- Sourciando ambiente do workspace ($WORKSPACE_ROOT) ---"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
    echo "Ambiente do workspace sourciado com sucesso."
else
    echo "ERRO: Arquivo de setup do workspace não encontrado em $WORKSPACE_SETUP. Certifique-se de que o workspace foi compilado."
    exit 1
fi

echo "--- Iniciando o sistema do robô (odometria, câmera, RViz, etc.) ---"
echo "Configurações de sensores: Fake Lidar=$USE_FAKE_LIDAR, Fake Camera=$USE_FAKE_CAMERA, Real Camera=$USE_REAL_CAMERA, RViz=$USE_RVIZ, JSP GUI=$USE_JSP_GUI, Modelo Robô=$ROBOT_MODEL"

# Constrói a string de argumentos para o launch file
LAUNCH_ARGS="use_fake_lidar:=$USE_FAKE_LIDAR \
             use_fake_camera:=$USE_FAKE_CAMERA \
             use_real_camera:=$USE_REAL_CAMERA \
             use_rviz:=$USE_RVIZ \
             use_jsp_gui:=$USE_JSP_GUI"

# O comando ros2 launch é executado em segundo plano (&) para que o script possa continuar
ros2 launch "$ROBOT_PACKAGE" "$ROBOT_LAUNCH_FILE" $LAUNCH_ARGS &
ROBOT_LAUNCH_PID=$! # Salva o PID do processo do launch do robô

RPLIDAR_NODE_PID="" # Inicializa o PID do RPLIDAR como vazio

if [ "$USE_FAKE_LIDAR" = "false" ]; then
    echo "--- Iniciando o RPLIDAR real ---"
    ros2 run "$RPLIDAR_PACKAGE" rplidar_node \
      --ros-args \
      -p serial_port:="$RPLIDAR_SERIAL_PORT" \
      -p serial_baudrate:="$RPLIDAR_BAUDRATE" \
      -p frame_id:="$RPLIDAR_FRAME_ID" &
    RPLIDAR_NODE_PID=$! # Salva o PID do processo do nó do RPLIDAR
else
    echo "--- RPLIDAR real NÃO iniciado (use_fake_lidar está como true) ---"
fi

echo "Sistema completo iniciado. Pressione Ctrl+C para encerrar todos os processos."

# Função para encerrar os processos em segundo plano
cleanup() {
    echo -e "\n--- Encerrando processos ---"
    if [ -n "$ROBOT_LAUNCH_PID" ]; then
        kill "$ROBOT_LAUNCH_PID" 2>/dev/null
        wait "$ROBOT_LAUNCH_PID" 2>/dev/null
    fi
    if [ -n "$RPLIDAR_NODE_PID" ]; then
        kill "$RPLIDAR_NODE_PID" 2>/dev/null
        wait "$RPLIDAR_NODE_PID" 2>/dev/null
    fi
    echo "Todos os processos encerrados."
    exit 0
}

# Captura Ctrl+C para chamar a função cleanup
trap cleanup SIGINT

# Espera pelos processos em segundo plano para que o script não termine imediatamente
wait $ROBOT_LAUNCH_PID
if [ -n "$RPLIDAR_NODE_PID" ]; then
    wait $RPLIDAR_NODE_PID
fi

echo "Todos os processos encerrados."
