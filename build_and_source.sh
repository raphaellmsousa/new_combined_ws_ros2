#!/bin/bash

# --- Configurações ---
# Nome do seu workspace ROS 2
WORKSPACE_NAME="new_combined_ws"
# Nome do seu pacote (se quiser compilar apenas ele)
PACKAGE_NAME="robot_description_pkg"

# --- Navega para o diretório do workspace ---
# Obtém o diretório onde o script está sendo executado
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Assume que o script está na raiz do workspace
# Se o script estiver em outro lugar, ajuste esta linha
cd "$SCRIPT_DIR"

echo "Navegando para o workspace: $(pwd)"

# --- Source do ambiente ROS 2 base ---
# Certifique-se de que o ROS 2 Jazzy esteja instalado e configurado
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Sourcing ROS 2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
else
    echo "ERRO: setup.bash do ROS 2 Jazzy não encontrado em /opt/ros/jazzy/. Verifique sua instalação ROS."
    exit 1
fi

# --- Compilação do pacote ---
echo "Compilando o pacote '$PACKAGE_NAME'..."
# Você pode usar --packages-select $PACKAGE_NAME para compilar apenas o seu pacote
# Ou remover para compilar todos os pacotes no workspace
colcon build --packages-select "$PACKAGE_NAME" --symlink-install

# Verifica se a compilação foi bem-sucedida
if [ $? -eq 0 ]; then
    echo "Compilação concluída com sucesso!"
else
    echo "ERRO: A compilação falhou."
    exit 1
fi

# --- Source do ambiente do workspace ---
if [ -f "install/setup.bash" ]; then
    echo "Sourcing o ambiente do workspace..."
    source install/setup.bash
    echo "Ambiente ROS 2 e workspace configurados."
else
    echo "ERRO: setup.bash do workspace não encontrado. A compilação pode ter falhado ou o diretório 'install' não existe."
    exit 1
fi

echo "Pronto para lançar os robôs!"
echo "Para o robô circular: ros2 launch robot_description_pkg display_robot.launch.py"
echo "Para o robô retangular: ros2 launch robot_description_pkg display_robot_rect.launch.py"