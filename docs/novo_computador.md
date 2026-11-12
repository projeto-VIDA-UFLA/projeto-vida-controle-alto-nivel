# Operar o VIDA a partir de outro computador

Roteiro para preparar um notebook com Ubuntu para operar o VIDA. As etapas 1 a 6 são
feitas **uma vez** por computador, no laboratório, com internet. Depois disso, a operação
é só a etapa 7: ligar o cabo e clicar em **Conectar à Odroid** no painel.

O notebook precisa de uma porta de rede cabeada (ou um adaptador USB-Ethernet), Wi-Fi para
a internet e Bluetooth para o controle. Não é preciso instalar o ROS 2: tudo roda em
containers Docker.

## 1. Git

```bash
sudo apt update
sudo apt install -y git
git config --global user.name "Seu Nome"
git config --global user.email "seu-email@exemplo.com"
```

## 2. Acesso ao GitHub por SSH

Os repositórios ficam na organização `NLMT-auto`; sua conta do GitHub precisa ter acesso a ela.

```bash
ssh-keygen -t ed25519 -C "seu-email@exemplo.com"   # Enter em tudo (ou defina uma senha)
cat ~/.ssh/id_ed25519.pub                          # copie a linha inteira
```

No GitHub: **Settings → SSH and GPG keys → New SSH key**, cole a chave e salve. Para testar:

```bash
ssh -T git@github.com     # "Hi <usuário>! You've successfully authenticated..."
```

## 3. Docker

Instalação oficial ([docs.docker.com](https://docs.docker.com/engine/install/ubuntu/)):

```bash
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo usermod -aG docker $USER     # usar o Docker sem sudo
```

**Saia da sessão e entre de novo** (ou reinicie) para o grupo valer. Para testar:
`docker run --rm hello-world`.

## 4. Repositórios e imagens

No notebook são dois repositórios: o **alto nível** (gamepad, Stanley, gerenciador de modos)
e o **painel**. Os outros dois (firmware e Odroid) ficam na Odroid.

```bash
mkdir -p ~/projeto-vida/analises ~/projeto-vida/rosbag
cd ~/projeto-vida
git clone git@github.com:NLMT-auto/projeto-vida-controle-alto-nivel.git
git clone git@github.com:NLMT-auto/projeto-vida-painel.git

cd ~/projeto-vida/projeto-vida-controle-alto-nivel
docker compose build                 # imagem vida-ros-ws:humble (alguns minutos)

cd ~/projeto-vida/projeto-vida-painel
docker compose up -d --build         # o painel usa a imagem do alto nível como base
```

O painel sobe sozinho a cada vez que o computador liga (`restart: unless-stopped`).

**Rotas:** a pasta `~/projeto-vida/analises` não está no Git. Copie para ela, do notebook
principal, as rotas que for usar (por exemplo, a `Pista_PV1`), mantendo as subpastas.

## 5. Rede para a Odroid

```bash
cd ~/projeto-vida/projeto-vida-controle-alto-nivel/scripts/rede
sudo ./preparar_computador.sh
```

O script:
- **não grava nenhum perfil de rede**: ao conectar (botão do painel ou comando `vida-rede`),
  o perfil **"VIDA Odroid"** (`192.168.10.2`) é criado só na memória, preso à porta do cabo,
  e é apagado ao desconectar ou ao reiniciar;
- instala o compartilhamento de internet com a Odroid (NAT), que só fica ativo junto com o perfil;
- instala o comando `vida-rede` e o serviço `vida-rede`, que executa o botão do painel (o
  painel roda em container e não mexe na rede do Ubuntu: ele só pede ao serviço);
- no fim, confere o resto da preparação (Docker, repositórios, imagens, controle).

**Ele não muda a rede normal do computador.** Fora do perfil do VIDA, a porta cabeada continua
no modo automático (DHCP), e reiniciar o computador sempre volta a esse modo.

Para só conferir, sem mudar nada: `./preparar_computador.sh --verificar`.
Para desfazer: `sudo ./preparar_computador.sh --remover`.

## 6. Controle (DS4) por Bluetooth

**Configurações → Bluetooth**. No controle, segure **PS + Share** até a luz piscar rápido e
escolha "Wireless Controller". O pareamento fica salvo; nas próximas vezes basta apertar PS.

## 7. Operar

1. Ligue o VIDA (a Odroid sobe sozinha) e passe o cabo de rede da Odroid até o notebook.
2. Abra o painel: **http://localhost:8080** (de preferência no Chrome).
3. Clique em **Conectar à Odroid** (no topo do painel). O painel configura a porta, compartilha
   a internet e confere com ping. Deu certo quando o bloco **ESP32 / Odroid** fica verde.
   - Se a Odroid não responder em 10 s, o painel avisa: confira se ela está ligada e se o cabo é o dela.
4. Opere pelo painel (rotas, iniciar o carro, gravar bag) e pelo controle, como no
   [operacao.md](operacao.md).
5. Ao terminar: pare o carro e clique em **Desconectar da Odroid**. A porta volta ao modo
   automático. Reiniciar o computador tem o mesmo efeito.

Pelo terminal, o mesmo funciona com:

```bash
vida-rede conectar      # configura a porta e confere o ping (pede a senha: sudo)
vida-rede status
vida-rede desconectar   # porta de volta ao modo automático
```

**Atalho na área de trabalho (opcional):** um arquivo `~/.local/share/applications/painel-vida.desktop`:

```ini
[Desktop Entry]
Type=Application
Name=Painel VIDA
Exec=google-chrome --app=http://localhost:8080
Icon=applications-science
```

## Atualizar depois

```bash
cd ~/projeto-vida/projeto-vida-controle-alto-nivel && git pull && docker compose build
cd ~/projeto-vida/projeto-vida-painel && git pull && docker compose up -d --build
```

Se o `scripts/rede` mudou, rode `sudo ./preparar_computador.sh` de novo (ele atualiza o que já existe).
