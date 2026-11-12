# Instruções para executar o programa

**Passo a passo completo (atualizar, gravar o firmware, ligar, manual, Stanley e gravar rota): [docs/operacao.md](docs/operacao.md).**

**Operar de outro notebook** (Docker, repositórios, rede até a Odroid e o botão **Conectar à
Odroid** do painel): [docs/novo_computador.md](docs/novo_computador.md). Os scripts de rede
ficam em `scripts/rede/`; o teste deles numa máquina virtual, em
[docs/teste_rede_vm.md](docs/teste_rede_vm.md).

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

Cada comando valida o caminho (raio mínimo 4 m com ±20° e recomendado de 6 m a
2,5 m/s, reta inicial de 10 m para a manobra de inicialização, lacunas sem
FIXED na gravação) e salva um
gráfico `.png` ao lado. `path_tools validate` e `path_tools compare` (Google x
gravado) também estão disponíveis. Testes:

    cd ros_ws/src/vida_navigation && python3 -m pytest test

## Modo automático (Stanley)

    ros2 launch vida_navigation navigation.launch.py path_file:=/caminho/pista.csv

Pelo Docker (a pasta `../analises` do notebook aparece como `/caminhos`):

    docker compose stop ros_ws
    VIDA_PATH_FILE=/caminhos/<pasta>/<caminho>.csv docker compose --profile navegacao up -d navegacao
    # de volta ao manual:
    docker compose stop navegacao && docker compose up -d ros_ws

Sobe o gamepad (publicando em `/gamepad/*`), o `mode_manager` (único que
publica `/vida/control/*/reference`), o `stanley` e o `path_publisher`.
Parâmetros em `ros_ws/src/vida_navigation/config/navigation.yaml`.

| Gamepad (5 s) | Ação | Vibração |
|---|---|---|
| △ | entra no automático | 1 pulso: aceito; 2 longos: Stanley no comando |
| □ | sai do automático (para) | 4 curtos: de volta ao manual |
| L2 | aborta e freia (qualquer momento) | contínua: aborto |
| X / O | modo bancada (ESP32) | contínua / 3 pulsos |
| R1 (3 s) | modo manobra: freio solto para empurrar (ESP32) | 2 curtos / 1 longo |

Pedido recusado: 6 pulsos rápidos (motivo no log e em `/navigation/mode`).
Para entrar: parado, ESP32 liberando, GPS com correções RTK e antena a até
3 m do início do caminho. Sequência: alinha as rodas, reinicia o EKF (origem
e bias, parado), anda reto a 2 m/s até o heading pelo GPS (RTK FIXED; ~5-8 m),
confere o alinhamento (< 0,5 m, < 15°) e segue uma volta a 2,5 m/s (abaixo de
~2 m/s a controladora de tração pulsa). No fim, freia com tudo na distância de
frenagem (v²/2a + v·atraso, a = 1,5 m/s² medido). Ao voltar ao manual, a tração
só volta ao gamepad depois de soltar o R2.

## Simulação

    ros2 launch vida_navigation simulation.launch.py              # caminho de exemplo
    ros2 run vida_navigation sim_scenarios volta                  # operador simulado

O `sim_vehicle` substitui ESP32 + GPS + EKF com os mesmos tópicos, com os
números medidos no chão em 30/10 (entre-eixos efetivo de 1,42 m, freio total
~1,7 m/s², roda livre 0,5 m/s², rampa de 40°/s no esterçamento, GPS a 1 Hz).
A 2,5 m/s, numa pista com curvas de 6 m: erro lateral médio de ~0,14 m. Cenários:
`volta`, `float`, `l2`, `quadrado`, `rtk_perdido`, `gps_perdido`,
`esp_bloqueio`, `longe` (com `start_back_m:=6.0`) e `desalinhado` (com
`start_yaw_deg:=40.0`). Com `gamepad:=true` dá para dirigir o simulador.

Ajuste de 30/10 (1º Stanley no chão): o esterçamento real tem ~0,3 s de atraso
e sobressinal, e o simulador foi calibrado com esse teste. Para 1,5 m/s:
`gain_k` 1,5 e `yaw_damping_s` 0,2 (amortecimento do giro em excesso).
Pendências para a pista: GPS a 1 Hz gera degraus de até ~0,2 m nas curvas (tentar 2 Hz no
ZED-F9R) e a escala dos encoders está ~3,5% acima do GPS.

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
