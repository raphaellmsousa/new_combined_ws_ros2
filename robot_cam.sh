#!/bin/bash

# Define o diretório do workspace
# Este script assume que está sendo executado da raiz do workspace (robot_ws)
# ou de um subdiretório, e encontra a raiz automaticamente.
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Se o script estiver em ~/robot_ws/scripts/, o WORKSPACE_DIR deve ser o pai
if [[ "$WORKSPACE_DIR" == *"scripts"* ]]; then
    WORKSPACE_DIR="$( cd "$WORKSPACE_DIR/.." &> /dev/null && pwd )"
fi

# --- 1. Source o ambiente ROS 2 principal (Jazzy) ---
echo "Sourciando o ambiente ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash
if [ $? -ne 0 ]; then
    echo "Erro: Não foi possível sourciar /opt/ros/jazzy/setup.bash. Verifique sua instalação do ROS 2 Jazzy."
    exit 1
fi
echo "ROS 2 Jazzy sourciado."

# --- 2. Compila o workspace ---
echo "Construindo o workspace ROS 2..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install
if [ $? -eq 0 ]; then
    echo "Build concluído com sucesso."
else
    echo "Erro no build. Verifique os logs."
    exit 1
fi

# --- 3. Source o ambiente do workspace ---
echo "Sourciando o ambiente do workspace..."
source "$WORKSPACE_DIR/install/setup.bash"
if [ $? -ne 0 ]; then
    echo "Erro ao sourciar o ambiente do workspace. O build pode ter falhado ou o diretório 'install' não existe."
    exit 1
fi
echo "Ambiente do workspace sourciado."

# --- 4. Lança a aplicação ROS 2 ---
# O nome do launch file que acabamos de criar
LAUNCH_FILE="display_robot.launch.py"
PACKAGE_NAME="robot_description_pkg" # Nome do seu pacote

echo "Lançando a aplicação ROS 2: $LAUNCH_FILE do pacote $PACKAGE_NAME..."
ros2 launch "$PACKAGE_NAME" "$LAUNCH_FILE"

exit 0

