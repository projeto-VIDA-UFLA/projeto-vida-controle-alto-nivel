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
