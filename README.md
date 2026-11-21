# Projeto VIDA: nível de navegação (notebook)

Software executado no **notebook de operação** do **VIDA (Veículo Inteligente de Desenvolvimento
Aplicado)**, protótipo de veículo autônomo do Laboratório de Mobilidade Terrestre (LMT) da UFLA.

Reúne a operação manual pelo gamepad, o controlador de seguimento de trajetória **Stanley**, o
**gerenciador de modos** (manual e automático), as ferramentas de gravação e tratamento de rotas,
um simulador do veículo e os scripts que preparam a rede de um notebook para falar com a Odroid.
Tudo roda em ROS 2 Humble, dentro de containers Docker: não é preciso instalar o ROS 2 no notebook.

- **Visão geral do projeto e como operar o veículo:** [projeto-VIDA-UFLA](https://github.com/projeto-VIDA-UFLA)
- **Nós por máquina:** [`docs/arquitetura_navegacao.pdf`](docs/arquitetura_navegacao.pdf)

## Lugar na arquitetura

| Nível | Hardware | Repositório |
|---|---|---|
| 1 | ESP32 | [projeto-vida-firmware-esp32](https://github.com/projeto-VIDA-UFLA/projeto-vida-firmware-esp32): sensores, controle de baixo nível e segurança |
| 2 | Odroid N2 | [projeto-vida-controle-baixo-nivel-odroid](https://github.com/projeto-VIDA-UFLA/projeto-vida-controle-baixo-nivel-odroid): agente micro-ROS, GNSS RTK, EKF e enable lógico |
| 3 | **Notebook** | **Este repositório:** gamepad, Stanley e gerenciador de modos; [projeto-vida-painel](https://github.com/projeto-VIDA-UFLA/projeto-vida-painel): painel de operação |

## Estrutura do repositório

```text
docker-compose.yml           Serviços ros_ws (manual) e navegacao (manual + Stanley)
ros_ws/
  Dockerfile                 Imagem vida-ros-ws:humble (também é a base do painel)
  src/
    control_pkg/             Leitura do gamepad (gamepad_reader) e referências do modo manual
    vida_navigation/         Stanley, gerenciador de modos, rotas e simulador
      config/navigation.yaml Parâmetros do Stanley e do gerenciador de modos
      paths/                 Rotas de exemplo
      test/                  Testes (pytest)
    vida_interfaces/         Mensagens ROS 2 do VIDA
scripts/rede/                Conexão de um notebook com a Odroid (serviço vida-rede)
docs/                        Diagrama da arquitetura de navegação
```

## Nós

| Nó | Pacote | Função |
|---|---|---|
| `gamepad_reader` | `control_pkg` | Lê o controle (DS4) e gera as referências de esterçamento e velocidade e o sinal `/gamepad/enabled` |
| `mode_manager` | `vida_navigation` | Único que publica as referências para o ESP32: repassa o gamepad no manual e o Stanley no automático; conduz a entrada (alinhamento, calibração do EKF e manobra inicial) e a saída do automático |
| `stanley` | `vida_navigation` | Controlador Stanley: erro lateral, erro de rumo, antecipação pela curvatura e amortecimento do giro |
| `path_publisher` | `vida_navigation` | Publica a rota no referencial do EKF, para visualização (o Stanley lê o arquivo da rota diretamente) |
| `path_recorder` / `path_tools` | `vida_navigation` | Gravação de rotas pelo EKF e tratamento, validação e comparação de rotas |
| `sim_vehicle` / `sim_scenarios` | `vida_navigation` | Simulador do veículo (substitui ESP32, GNSS e EKF) e cenários de teste |

## Serviços Docker

| Serviço | Container | O que sobe |
|---|---|---|
| `ros_ws` | `ros_ws` | Só o gamepad (modo manual) |
| `navegacao` (perfil `navegacao`) | `ros_ws_navegacao` | Gamepad, `mode_manager`, `stanley` e `path_publisher`, com a rota em `VIDA_PATH_FILE` |

A pasta `~/projeto-vida/analises` do notebook aparece nos containers como `/caminhos`: é onde
ficam as rotas. No dia a dia, quem inicia e para esses serviços é o painel de operação.

```bash
docker compose build                                         # imagem vida-ros-ws:humble
docker compose up -d                                         # modo manual
docker compose stop ros_ws                                   # navegação com uma rota:
VIDA_PATH_FILE=/caminhos/<pasta>/<rota>.csv docker compose --profile navegacao up -d navegacao
```

## Modos e comandos do gamepad

| Botão | Ação |
|---|---|
| R2 / L2 | Acelera (até 2,5 m/s) / freia |
| Analógico esquerdo | Esterça (até 20°) |
| △ por 5 s | Entra no automático (serviço `navegacao`) |
| □ por 5 s | Sai do automático |
| L2 | Aborta o automático e freia, a qualquer momento |
| R1 por 3 s | Modo manobra: freio solto para empurrar o veículo |
| X / O por 5 s | Entra / sai do modo bancada (veículo suspenso) |

Para entrar no automático, o veículo precisa estar parado, com o ESP32 liberando o movimento,
GNSS com correções RTK e a antena a até 3 m do início da rota. O `mode_manager` então alinha as
rodas, reinicia o EKF (origem e viés do giroscópio), anda em linha reta até obter o rumo pelo
GNSS, confere o alinhamento com a rota e passa o comando ao Stanley. No fim da rota, o veículo
freia e para.

## Rotas

As rotas são arquivos CSV em latitude e longitude (cabeçalho `# vida_path v1`), reamostrados a
cada 0,25 m. No dia a dia, são gravadas pelo painel de operação. Pelo terminal:

```bash
# desenhada no Google Earth (KML)
ros2 run vida_navigation path_tools from-kml pista.kml pista.csv --name pista
# gravada dirigindo, a partir do EKF
ros2 run vida_navigation path_recorder --ros-args -p output_file:=volta_bruta.csv
ros2 run vida_navigation path_tools from-recording volta_bruta.csv pista.csv
# validar uma rota (raio mínimo, reta inicial) e gerar o gráfico
ros2 run vida_navigation path_tools validate pista.csv
```

## Simulação e testes

```bash
ros2 launch vida_navigation simulation.launch.py       # simulador com a rota de exemplo
ros2 run vida_navigation sim_scenarios volta           # operador simulado
cd ros_ws/src/vida_navigation && python3 -m pytest test
```

Cenários disponíveis: `volta`, `float`, `l2`, `quadrado`, `rtk_perdido`, `gps_perdido`,
`esp_bloqueio`, `longe` e `desalinhado`. Com `gamepad:=true`, dá para dirigir o simulador.

## Rede até a Odroid em outro notebook

Os scripts de `scripts/rede/` preparam um notebook com Ubuntu para se conectar à Odroid pelo
cabo: instalam o compartilhamento de internet (NAT), o comando `vida-rede` e o serviço usado pelo
botão **Conectar à Odroid** do painel. O perfil de rede da Odroid só existe enquanto a conexão
está ativa, e a porta volta ao modo automático (DHCP) ao desconectar ou reiniciar.

```bash
sudo scripts/rede/preparar_computador.sh              # instala
scripts/rede/preparar_computador.sh --verificar       # só confere
sudo scripts/rede/preparar_computador.sh --remover    # desfaz
```

O passo a passo completo está na página do projeto.

## Licença

MIT. Veja [`LICENSE`](LICENSE).
