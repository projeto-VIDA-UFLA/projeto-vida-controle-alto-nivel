# Instruções para executar o programa

# Estimador de estado (EKF)
O `robot_state_ekf` roda no Odroid (nível 2) e fica no repositório
`projeto-vida-controle-baixo-nivel-odroid` (submodule em `ros_ws/src/robot_state_ekf`).
O notebook recebe `/robot_state/odom` e `/robot_state/status` e chama o serviço
`/robot_state_ekf_node/reset` ao entrar no modo automático.

O `setup.sh` continua inicializando submodules, caso o projeto volte a ter algum.

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
