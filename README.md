# Instruções para executar o programa

# Estimador de estado (EKF)
O `robot_state_ekf` roda no Odroid (nível 2) e fica no repositório
`projeto-vida-controle-baixo-nivel-odroid` (submodule em `ros_ws/src/robot_state_ekf`).
O notebook recebe `/robot_state/odom` e `/robot_state/status` e chama o serviço
`/robot_state_ekf_node/reset` ao entrar no modo automático.

O `setup.sh` continua inicializando submodules, caso o projeto volte a ter algum.

# Navegação (`vida_navigation`)
Visão geral dos nós por máquina em `docs/arquitetura_navegacao.pdf`.

Caminhos ficam em CSV (latitude/longitude, cabeçalho `# vida_path v1`), em
`ros_ws/src/vida_navigation/paths/`. Duas origens:

    # desenhado no Google Earth (Salvar lugar como... KML)
    ros2 run vida_navigation path_tools from-kml pista.kml pista.csv --name pista_ufla
    # gravado dirigindo com o controle (EKF rodando, RTK FIXED)
    ros2 run vida_navigation path_recorder --ros-args -p output_file:=volta_bruta.csv
    ros2 run vida_navigation path_tools from-recording volta_bruta.csv pista.csv

Cada comando valida o caminho (raio mínimo 3,8 m com ±20°, reta inicial de 5 m
para a manobra de inicialização, lacunas sem FIXED na gravação) e salva um
gráfico `.png` ao lado. `path_tools validate` e `path_tools compare` (Google x
gravado) também estão disponíveis. Testes:

    cd ros_ws/src/vida_navigation && python3 -m pytest test

## Modo automático (Stanley)

    ros2 launch vida_navigation navigation.launch.py path_file:=/caminho/pista.csv

Sobe o gamepad (publicando em `/gamepad/*`), o `mode_manager` (único que
publica `/vida/control/*/reference`), o `stanley` e o `path_publisher`.
Parâmetros em `ros_ws/src/vida_navigation/config/navigation.yaml`.

| Gamepad (5 s) | Ação | Vibração |
|---|---|---|
| △ | entra no automático | 1 pulso: aceito; 2 longos: Stanley no comando |
| □ | sai do automático (para) | 4 curtos: de volta ao manual |
| L2 | aborta e freia (qualquer momento) | contínua: aborto |
| X / O | modo bancada (ESP32) | contínua / 3 pulsos |

Pedido recusado: 6 pulsos rápidos (motivo no log e em `/navigation/mode`).
Para entrar: parado, ESP32 liberando, GPS com correções RTK e antena a até
3 m do início do caminho. Sequência: alinha as rodas, reinicia o EKF (origem
e bias, parado), anda reto a 0,5 m/s até o heading pelo GPS (RTK FIXED),
confere o alinhamento (< 1 m, < 30°) e segue uma volta a 1 m/s. Ao voltar ao
manual, a tração só volta ao gamepad depois de soltar o R2.

## Simulação

    ros2 launch vida_navigation simulation.launch.py              # caminho de exemplo
    ros2 run vida_navigation sim_scenarios volta                  # operador simulado

O `sim_vehicle` substitui ESP32 + GPS + EKF com os mesmos tópicos (entre-eixos
efetivo de 1,38 m, erro de 7% dos encoders, GPS a 1 Hz). Cenários:
`volta`, `float`, `l2`, `quadrado`, `rtk_perdido`, `gps_perdido`,
`esp_bloqueio`, `longe` (com `start_back_m:=6.0`) e `desalinhado` (com
`start_yaw_deg:=40.0`). Com `gamepad:=true` dá para dirigir o simulador.

Pendências para a pista: ganho `gain_k` validado só no simulador; GPS a
1 Hz gera degraus de até 0,35 m na posição (subir para 5-10 Hz no ZED-F9R
e calibrar a escala dos encoders, ver TODO do firmware).

# Instalação CMake
1- sudo apt update
2- sudo apt upgrade
3- sudo apt install cmake

# Instalação da biblioteca WiringPi
1- sudo apt update
2- sudo apt upgrade
3- sudo apt-get install git build-essential
4- git clone https://github.com/WiringPi/WiringPi
5- cd WiringPi
6- ./build

# Compilação
1- cmake -S src -B build -----------------Identificar as pastas src e build
2- cd build ------------------------------Acessa diretori build
3- make ----------------------------------Compilar
4- ./vida --------------------------------Executar
*Caso não funcione é so excluir a pasta build e repetir o processo:


*Steering control and encoder*
Video: https://youtu.be/gFds-Ci4kwE
This repository contains the VIDA standalone platform codes.

*Example_Thread_Semaphore_SharedMemory:*
Video: https://youtu.be/gFds-Ci4kwE


teste 
