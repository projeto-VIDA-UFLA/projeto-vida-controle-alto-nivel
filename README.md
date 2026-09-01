# Instruções para executar o programa

# Clonando o projeto (submodules)
Este projeto usa git submodules (ex: `ros_ws/src/robot_state_ekf`). Depois de clonar, rode uma vez:

	./setup.sh

Isso inicializa os submodules e configura este clone para atualiza-los automaticamente em
`git pull`/`checkout` daqui pra frente.

Alternativa manual (equivalente, sem o `git config` automatico):

	git clone --recurse-submodules https://github.com/NLMT-auto/projeto-vida-controle-alto-nivel.git
	# ou, se ja clonou sem a flag acima:
	git submodule update --init --recursive

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
