# new_combined_ws – ROS 2 Mobile Robot + Lidar + Webcam

Este repositório contém um workspace ROS 2 (`colcon`) com um robô móvel simples, odometria simulada em movimento circular, um Lidar (simulado ou futuro real) e uma pipeline de câmera com processamento de imagem em tempo real (OpenCV).

Testado em:
- **Ubuntu:** 24.04 (Noble)
- **ROS 2:** Jazzy

---

## Estrutura do Workspace

new_combined_ws/
├── src/
│   ├── robot_description_pkg/
│   │   ├── launch/
│   │   │   └── display_robot.launch.py
│   │   ├── urdf/
│   │   │   └── … (arquivos .urdf/.xacro do robô)
│   │   ├── rviz/
│   │   │   └── display_robot.rviz
│   │   ├── robot_description_pkg/
│   │   │   ├── simple_odom_publisher_node.py
│   │   │   └── …
│   │   └── setup.py
│   ├── webcam_demo/
│   │   ├── camera_node.py
│   │   └── processor_node.py
│   └── setup.py
└── README.md

---

## Pacotes

### `robot_description_pkg`

Pacote que descreve o robô e integra:

- **URDF/Xacro** do robô (estrutura + sensores).
- **Odometria simulada** em movimento circular:
  - Nó: `simple_odom_publisher_node`
  - Executável: `simple_odom`
  - Publica em:
    - `/odom` (`nav_msgs/Odometry`)
    - TF `odom -> base_link`
- **Launch principal**:
  - `display_robot.launch.py`:
    - `robot_state_publisher`
    - `joint_state_publisher_gui` (opcional, via argumento `use_jsp_gui`)
    - `rviz2` com configuração em `rviz/display_robot.rviz`
    - `simple_odom` (odometria)
    - nós de sensores (ex.: Lidar simulado e nós de câmera/processamento)

`entry_points` relevantes (em `robot_description_pkg/setup.py`), exemplo:

### `webcam_demo`

Pipeline de câmera com OpenCV e ROS 2.

- **`camera_node.py`**
  - Nó: `camera_node`
  - Executável: `camera`
  - Captura da webcam (`/dev/video0`) via OpenCV.
  - Publica em:
    - `camera/image_raw` (`sensor_msgs/Image`)
  - Pode exibir a imagem original em uma janela OpenCV.

- **`processor_node.py`**
  - Nó: `processor_node`
  - Executável: `processor`
  - Assina:
    - `camera/image_raw`
  - Processa o frame (ex.: conversão para tons de cinza, detecção simples, etc.) e publica em:
    - `camera/image_processed`
  - Pode exibir a imagem processada em uma janela OpenCV.

`entry_points` do pacote (`webcam_demo/setup.py`), exemplo:

<div class="widget code-container remove-before-copy"><div class="code-header non-draggable"><span class="iaf s13 w700 code-language-placeholder">python</span><div class="code-copy-button"><span class="iaf s13 w500 code-copy-placeholder">Copiar</span><img class="code-copy-icon" src="data:image/svg+xml;utf8,%0A%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2216%22%20height%3D%2216%22%20viewBox%3D%220%200%2016%2016%22%20fill%3D%22none%22%3E%0A%20%20%3Cpath%20d%3D%22M10.8%208.63V11.57C10.8%2014.02%209.82%2015%207.37%2015H4.43C1.98%2015%201%2014.02%201%2011.57V8.63C1%206.18%201.98%205.2%204.43%205.2H7.37C9.82%205.2%2010.8%206.18%2010.8%208.63Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%20%20%3Cpath%20d%3D%22M15%204.42999V7.36999C15%209.81999%2014.02%2010.8%2011.57%2010.8H10.8V8.62999C10.8%206.17999%209.81995%205.19999%207.36995%205.19999H5.19995V4.42999C5.19995%201.97999%206.17995%200.999992%208.62995%200.999992H11.57C14.02%200.999992%2015%201.97999%2015%204.42999Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%3C%2Fsvg%3E%0A" /></div></div><pre id="code-v705e5mmd" style="color:white;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;white-space:pre;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none;padding:8px;margin:8px;overflow:auto;background:#011627;width:calc(100% - 8px);border-radius:8px;box-shadow:0px 8px 18px 0px rgba(120, 120, 143, 0.10), 2px 2px 10px 0px rgba(255, 255, 255, 0.30) inset"><code class="language-python" style="white-space:pre;color:#d6deeb;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none"><span>entry_points</span><span class="token" style="color:rgb(127, 219, 202)">=</span><span class="token" style="color:rgb(199, 146, 234)">{</span><span>
</span><span>    </span><span class="token" style="color:rgb(173, 219, 103)">&#x27;console_scripts&#x27;</span><span class="token" style="color:rgb(199, 146, 234)">:</span><span> </span><span class="token" style="color:rgb(199, 146, 234)">[</span><span>
</span><span>        </span><span class="token" style="color:rgb(173, 219, 103)">&#x27;camera = webcam_demo.camera_node:main&#x27;</span><span class="token" style="color:rgb(199, 146, 234)">,</span><span>
</span><span>        </span><span class="token" style="color:rgb(173, 219, 103)">&#x27;processor = webcam_demo.processor_node:main&#x27;</span><span class="token" style="color:rgb(199, 146, 234)">,</span><span>
</span><span>    </span><span class="token" style="color:rgb(199, 146, 234)">]</span><span class="token" style="color:rgb(199, 146, 234)">,</span><span>
</span><span></span><span class="token" style="color:rgb(199, 146, 234)">}</span><span>
</span></code></pre></div>

---

## Dependências

Instale o ROS 2 Jazzy e ferramentas básicas conforme a documentação oficial.

Dependências Python principais (já declaradas nos `setup.py`):

- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `tf2_ros`
- `tf_transformations`
- `opencv-python`
- `cv_bridge`

Dependências ROS 2/Jazzy úteis para este workspace (Ubuntu 24.04):

<div class="widget code-container remove-before-copy"><div class="code-header non-draggable"><span class="iaf s13 w700 code-language-placeholder">bash</span><div class="code-copy-button"><span class="iaf s13 w500 code-copy-placeholder">Copiar</span><img class="code-copy-icon" src="data:image/svg+xml;utf8,%0A%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2216%22%20height%3D%2216%22%20viewBox%3D%220%200%2016%2016%22%20fill%3D%22none%22%3E%0A%20%20%3Cpath%20d%3D%22M10.8%208.63V11.57C10.8%2014.02%209.82%2015%207.37%2015H4.43C1.98%2015%201%2014.02%201%2011.57V8.63C1%206.18%201.98%205.2%204.43%205.2H7.37C9.82%205.2%2010.8%206.18%2010.8%208.63Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%20%20%3Cpath%20d%3D%22M15%204.42999V7.36999C15%209.81999%2014.02%2010.8%2011.57%2010.8H10.8V8.62999C10.8%206.17999%209.81995%205.19999%207.36995%205.19999H5.19995V4.42999C5.19995%201.97999%206.17995%200.999992%208.62995%200.999992H11.57C14.02%200.999992%2015%201.97999%2015%204.42999Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%3C%2Fsvg%3E%0A" /></div></div><pre id="code-undr3dbap" style="color:white;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;white-space:pre;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none;padding:8px;margin:8px;overflow:auto;background:#011627;width:calc(100% - 8px);border-radius:8px;box-shadow:0px 8px 18px 0px rgba(120, 120, 143, 0.10), 2px 2px 10px 0px rgba(255, 255, 255, 0.30) inset"><code class="language-bash" style="white-space:pre;color:#d6deeb;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none"><span class="token" style="color:rgb(130, 170, 255)">sudo</span><span> </span><span class="token" style="color:rgb(130, 170, 255)">apt</span><span> update
</span><span></span><span class="token" style="color:rgb(130, 170, 255)">sudo</span><span> </span><span class="token" style="color:rgb(130, 170, 255)">apt</span><span> </span><span class="token" style="color:rgb(130, 170, 255)">install</span><span> ros-jazzy-cv-bridge </span><span class="token" style="color:rgb(199, 146, 234)">\</span><span>
</span><span>                 ros-jazzy-image-transport </span><span class="token" style="color:rgb(199, 146, 234)">\</span><span>
</span><span>                 python3-opencv </span><span class="token" style="color:rgb(199, 146, 234)">\</span><span>
</span><span>                 ros-jazzy-tf-transformations </span><span class="token" style="color:rgb(199, 146, 234)">\</span><span>
</span>                 ros-jazzy-xacro
</code></pre></div>

> Observação:
> - `ros-jazzy-tf-transformations` é necessário para nós que usam `tf_transformations` em Python.
> - `ros-jazzy-xacro` é necessário para processar arquivos `.xacro` dos modelos do robô.

---

## Como Buildar o Workspace

Na raiz do workspace (`new_combined_ws`):

<div class="widget code-container remove-before-copy"><div class="code-header non-draggable"><span class="iaf s13 w700 code-language-placeholder">bash</span><div class="code-copy-button"><span class="iaf s13 w500 code-copy-placeholder">Copiar</span><img class="code-copy-icon" src="data:image/svg+xml;utf8,%0A%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2216%22%20height%3D%2216%22%20viewBox%3D%220%200%2016%2016%22%20fill%3D%22none%22%3E%0A%20%20%3Cpath%20d%3D%22M10.8%208.63V11.57C10.8%2014.02%209.82%2015%207.37%2015H4.43C1.98%2015%201%2014.02%201%2011.57V8.63C1%206.18%201.98%205.2%204.43%205.2H7.37C9.82%205.2%2010.8%206.18%2010.8%208.63Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%20%20%3Cpath%20d%3D%22M15%204.42999V7.36999C15%209.81999%2014.02%2010.8%2011.57%2010.8H10.8V8.62999C10.8%206.17999%209.81995%205.19999%207.36995%205.19999H5.19995V4.42999C5.19995%201.97999%206.17995%200.999992%208.62995%200.999992H11.57C14.02%200.999992%2015%201.97999%2015%204.42999Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%3C%2Fsvg%3E%0A" /></div></div><pre id="code-bl2buwpgf" style="color:white;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;white-space:pre;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none;padding:8px;margin:8px;overflow:auto;background:#011627;width:calc(100% - 8px);border-radius:8px;box-shadow:0px 8px 18px 0px rgba(120, 120, 143, 0.10), 2px 2px 10px 0px rgba(255, 255, 255, 0.30) inset"><code class="language-bash" style="white-space:pre;color:#d6deeb;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none"><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># 1. Fonte do ambiente do ROS 2</span><span>
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> /opt/ros/jazzy/setup.bash
</span>
<span></span><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># 2. Build dos pacotes</span><span>
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">cd</span><span> ~/new_combined_ws
</span>colcon build --symlink-install
<span></span><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># (opcional) --packages-select robot_description_pkg webcam_demo</span><span>
</span>
<span></span><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># 3. Fonte do workspace buildado</span><span>
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> install/setup.bash
</span></code></pre></div>

---

## Como Rodar

### 1. Launch completo do robô + RViz (+ sensores conforme configurado)

<div class="widget code-container remove-before-copy"><div class="code-header non-draggable"><span class="iaf s13 w700 code-language-placeholder">bash</span><div class="code-copy-button"><span class="iaf s13 w500 code-copy-placeholder">Copiar</span><img class="code-copy-icon" src="data:image/svg+xml;utf8,%0A%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2216%22%20height%3D%2216%22%20viewBox%3D%220%200%2016%2016%22%20fill%3D%22none%22%3E%0A%20%20%3Cpath%20d%3D%22M10.8%208.63V11.57C10.8%2014.02%209.82%2015%207.37%2015H4.43C1.98%2015%201%2014.02%201%2011.57V8.63C1%206.18%201.98%205.2%204.43%205.2H7.37C9.82%205.2%2010.8%206.18%2010.8%208.63Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%20%20%3Cpath%20d%3D%22M15%204.42999V7.36999C15%209.81999%2014.02%2010.8%2011.57%2010.8H10.8V8.62999C10.8%206.17999%209.81995%205.19999%207.36995%205.19999H5.19995V4.42999C5.19995%201.97999%206.17995%200.999992%208.62995%200.999992H11.57C14.02%200.999992%2015%201.97999%2015%204.42999Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%3C%2Fsvg%3E%0A" /></div></div><pre id="code-sq76s0q2a" style="color:white;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;white-space:pre;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none;padding:8px;margin:8px;overflow:auto;background:#011627;width:calc(100% - 8px);border-radius:8px;box-shadow:0px 8px 18px 0px rgba(120, 120, 143, 0.10), 2px 2px 10px 0px rgba(255, 255, 255, 0.30) inset"><code class="language-bash" style="white-space:pre;color:#d6deeb;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none"><span class="token" style="color:rgb(255, 203, 139)">source</span><span> /opt/ros/jazzy/setup.bash
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">cd</span><span> ~/new_combined_ws
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> install/setup.bash
</span>
<!-- -->ros2 launch robot_description_pkg display_robot.launch.py
</code></pre></div>

Esse launch normalmente sobe:

- `robot_state_publisher`
- `joint_state_publisher_gui` (se `use_jsp_gui:=true`)
- `rviz2`
- `simple_odom`
- e, se configurados no launch, nós de Lidar simulado, câmera (`webcam_demo`/`camera`) e processador (`webcam_demo`/`processor`).

### 2. Rodar câmera e processador separadamente

Em terminais separados (todos com o workspace já “sourçado”):

<div class="widget code-container remove-before-copy"><div class="code-header non-draggable"><span class="iaf s13 w700 code-language-placeholder">bash</span><div class="code-copy-button"><span class="iaf s13 w500 code-copy-placeholder">Copiar</span><img class="code-copy-icon" src="data:image/svg+xml;utf8,%0A%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2216%22%20height%3D%2216%22%20viewBox%3D%220%200%2016%2016%22%20fill%3D%22none%22%3E%0A%20%20%3Cpath%20d%3D%22M10.8%208.63V11.57C10.8%2014.02%209.82%2015%207.37%2015H4.43C1.98%2015%201%2014.02%201%2011.57V8.63C1%206.18%201.98%205.2%204.43%205.2H7.37C9.82%205.2%2010.8%206.18%2010.8%208.63Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%20%20%3Cpath%20d%3D%22M15%204.42999V7.36999C15%209.81999%2014.02%2010.8%2011.57%2010.8H10.8V8.62999C10.8%206.17999%209.81995%205.19999%207.36995%205.19999H5.19995V4.42999C5.19995%201.97999%206.17995%200.999992%208.62995%200.999992H11.57C14.02%200.999992%2015%201.97999%2015%204.42999Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%3C%2Fsvg%3E%0A" /></div></div><pre id="code-hgo4xzm45" style="color:white;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;white-space:pre;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none;padding:8px;margin:8px;overflow:auto;background:#011627;width:calc(100% - 8px);border-radius:8px;box-shadow:0px 8px 18px 0px rgba(120, 120, 143, 0.10), 2px 2px 10px 0px rgba(255, 255, 255, 0.30) inset"><code class="language-bash" style="white-space:pre;color:#d6deeb;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none"><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># Terminal 1 – câmera</span><span>
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> /opt/ros/jazzy/setup.bash
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">cd</span><span> ~/new_combined_ws
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> install/setup.bash
</span>ros2 run webcam_demo camera
<!-- -->
<span></span><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># Terminal 2 – processador</span><span>
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> /opt/ros/jazzy/setup.bash
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">cd</span><span> ~/new_combined_ws
</span><span></span><span class="token" style="color:rgb(255, 203, 139)">source</span><span> install/setup.bash
</span>ros2 run webcam_demo processor
</code></pre></div>

Tópicos esperados:

- `camera/image_raw`
- `camera/image_processed`

---

## Visualização no RViz

Com o `rviz2` rodando (via launch ou manualmente):

- **Fixed Frame:** `odom`
- Adicione os seguintes Displays:
  - `RobotModel` usando `robot_description`
  - `TF` para ver `odom -> base_link -> ...`
  - `Image` (para `camera/image_raw`)
  - `Image` (para `camera/image_processed`)
  - (se houver Lidar) `LaserScan` no tópico `/scan`

Depois de configurar, salve a visualização como `rviz/display_robot.rviz` para reutilizar.

---

## Comandos Úteis

Alguns comandos úteis durante o desenvolvimento:

<div class="widget code-container remove-before-copy"><div class="code-header non-draggable"><span class="iaf s13 w700 code-language-placeholder">bash</span><div class="code-copy-button"><span class="iaf s13 w500 code-copy-placeholder">Copiar</span><img class="code-copy-icon" src="data:image/svg+xml;utf8,%0A%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2216%22%20height%3D%2216%22%20viewBox%3D%220%200%2016%2016%22%20fill%3D%22none%22%3E%0A%20%20%3Cpath%20d%3D%22M10.8%208.63V11.57C10.8%2014.02%209.82%2015%207.37%2015H4.43C1.98%2015%201%2014.02%201%2011.57V8.63C1%206.18%201.98%205.2%204.43%205.2H7.37C9.82%205.2%2010.8%206.18%2010.8%208.63Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%20%20%3Cpath%20d%3D%22M15%204.42999V7.36999C15%209.81999%2014.02%2010.8%2011.57%2010.8H10.8V8.62999C10.8%206.17999%209.81995%205.19999%207.36995%205.19999H5.19995V4.42999C5.19995%201.97999%206.17995%200.999992%208.62995%200.999992H11.57C14.02%200.999992%2015%201.97999%2015%204.42999Z%22%20stroke%3D%22%23717C92%22%20stroke-width%3D%221.05%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%2F%3E%0A%3C%2Fsvg%3E%0A" /></div></div><pre id="code-f8jph34wj" style="color:white;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;white-space:pre;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none;padding:8px;margin:8px;overflow:auto;background:#011627;width:calc(100% - 8px);border-radius:8px;box-shadow:0px 8px 18px 0px rgba(120, 120, 143, 0.10), 2px 2px 10px 0px rgba(255, 255, 255, 0.30) inset"><code class="language-bash" style="white-space:pre;color:#d6deeb;font-family:Consolas, Monaco, &quot;Andale Mono&quot;, &quot;Ubuntu Mono&quot;, monospace;text-align:left;word-spacing:normal;word-break:normal;word-wrap:normal;line-height:1.5;font-size:1em;-moz-tab-size:4;-o-tab-size:4;tab-size:4;-webkit-hyphens:none;-moz-hyphens:none;-ms-hyphens:none;hyphens:none"><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># Ver tópicos</span><span>
</span>ros2 topic list
<!-- -->
<span></span><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># Ver TFs e gerar diagrama de frames</span><span>
</span>ros2 run tf2_tools view_frames
<!-- -->
<span></span><span class="token" style="color:rgb(99, 119, 119);font-style:italic"># Ecoar mensagens de tópicos</span><span>
</span><span>ros2 topic </span><span class="token" style="color:rgb(255, 203, 139)">echo</span><span> /odom
</span><span>ros2 topic </span><span class="token" style="color:rgb(255, 203, 139)">echo</span><span> /camera/image_raw
</span><span>ros2 topic </span><span class="token" style="color:rgb(255, 203, 139)">echo</span><span> /camera/image_processed
</span></code></pre></div>

---

## TODO / Próximos Passos

- Substituir sensores simulados por sensores reais (Lidar real, câmera real já integrada).
- Integrar com navegação (nav2) no futuro.
- Melhorar documentação do URDF (inércia, sensores, frames).
- Adicionar scripts de inicialização (ex.: `./robot_nav.sh`) para subir tudo com um comando.
- Refinar pipeline de visão (detecção, segmentação, etc.).

---

## Licença

Definir e atualizar o campo `license` nos `setup.py` e `package.xml` conforme a licença escolhida (por exemplo, MIT, Apache-2.0, BSD-3-Clause).

