#!/bin/bash
# Prepara um notebook Ubuntu para operar o VIDA pelo cabo de rede ate a Odroid.
#
#   ./preparar_computador.sh --verificar   so confere (nao muda nada, sem sudo)
#   sudo ./preparar_computador.sh          instala
#   sudo ./preparar_computador.sh --remover desfaz a instalacao
#
# O que a instalacao faz:
#   1. Nenhum perfil de rede gravado. O vida-rede cria o perfil "VIDA Odroid"
#      (192.168.10.2/24) so na memoria, preso a porta, ao conectar, e o apaga ao
#      desconectar; o reinicio tambem o apaga. Um perfil gravado e sem porta fixa
#      faria o NetworkManager parar de criar os perfis automaticos (DHCP) das
#      portas cabeadas. O IP tem que ser .2: e o gateway configurado na Odroid.
#   2. Dispatcher 90-vida-odroid: NAT e firewall so enquanto o perfil esta ativo.
#   3. Comando vida-rede (conectar / desconectar / status).
#   4. Servico vida-rede: ponte com o botao "Conectar a Odroid" do painel, que
#      roda em container e so troca arquivos em /run/vida-rede.
#
# Nao toca na conexao cabeada normal do computador: fora do perfil do VIDA, a
# porta continua em DHCP. Se encontrar uma configuracao antiga para a Odroid
# (outro perfil com 192.168.10.2 ou o script 90-nat-odroid), para sem mudar nada.

set -euo pipefail
export LC_ALL=C.UTF-8   # nmcli em ingles: "unmanaged" (as mensagens deste script sao fixas)

PERFIL="VIDA Odroid"
IP_NOTEBOOK=192.168.10.2
REDE_PREFIXO=24
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DISPATCHER=/etc/NetworkManager/dispatcher.d/90-vida-odroid
DISPATCHER_ANTIGO=/etc/NetworkManager/dispatcher.d/90-nat-odroid
CLI=/usr/local/bin/vida-rede
SERVICO=/etc/systemd/system/vida-rede.service

USUARIO=${SUDO_USER:-$USER}
CASA=$(getent passwd "$USUARIO" | cut -d: -f6)
PROJETO=${VIDA_PROJETO_DIR:-$CASA/projeto-vida}

FALHAS=0
ok()    { printf '  \033[32m✓\033[0m %s\n' "$*"; }
aviso() { printf '  \033[33m!\033[0m %s\n' "$*"; }
falha() { printf '  \033[31m✗\033[0m %s\n' "$*"; FALHAS=$((FALHAS + 1)); }
titulo(){ printf '\n\033[1m%s\033[0m\n' "$*"; }

exigir_root() {
  [ "$(id -u)" -eq 0 ] || { echo "Rode com sudo: sudo $0 ${1:-}" >&2; exit 1; }
}

perfil_existe() { nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "$PERFIL"; }

# Perfis (fora o do VIDA) com o IP do notebook: configuracao manual antiga.
perfis_em_conflito() {
  local uuid nome ends
  for uuid in $(nmcli -t -f UUID connection show 2>/dev/null); do
    nome=$(nmcli -g connection.id connection show "$uuid" 2>/dev/null) || continue
    [ "$nome" = "$PERFIL" ] && continue
    ends=$(nmcli -g ipv4.addresses connection show "$uuid" 2>/dev/null) || continue
    case ",$ends," in
      *"$IP_NOTEBOOK/"*) echo "$nome" ;;
    esac
  done
}

checar_conflitos() {  # 0 = sem conflito
  local conflito=0 lista
  lista=$(perfis_em_conflito)
  if [ -n "$lista" ]; then
    while IFS= read -r p; do
      falha "O perfil \"$p\" ja usa $IP_NOTEBOOK: este computador ja tem uma configuracao manual para a Odroid"
    done <<< "$lista"
    conflito=1
  fi
  if [ -e "$DISPATCHER_ANTIGO" ]; then
    falha "Existe o NAT antigo ($DISPATCHER_ANTIGO)"
    conflito=1
  fi
  return $conflito
}

verificar() {
  titulo "Sistema"
  if grep -qi '^ID=ubuntu' /etc/os-release 2>/dev/null; then
    ok "$(. /etc/os-release; echo "$PRETTY_NAME")"
  else
    aviso "Nao e Ubuntu: o script foi feito para Ubuntu (pode funcionar em derivados)"
  fi
  if command -v nmcli >/dev/null && systemctl is-active --quiet NetworkManager; then
    ok "NetworkManager ativo"
  else
    falha "NetworkManager nao esta ativo (necessario para o perfil e o botao)"
  fi
  command -v iptables >/dev/null && ok "iptables presente" || falha "iptables ausente (sudo apt install iptables)"

  titulo "Rede"
  local portas
  portas=$(nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null |
           awk -F: '$2 == "ethernet" && $3 != "unmanaged" { print $1 }' | xargs)
  if [ -n "$portas" ]; then ok "Porta(s) cabeada(s): $portas"
  else aviso "Nenhuma porta cabeada agora: sera preciso um adaptador USB-Ethernet"; fi

  checar_conflitos && ok "Sem configuracao antiga para a Odroid"

  local ativo
  ativo=$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null |
          awk -F: -v p="$PERFIL" '$1 == p && $2 != "" { print $2; exit }')
  if [ -n "$ativo" ]; then
    ok "Conectado a Odroid em $ativo"
  elif perfil_existe; then
    falha "Perfil \"$PERFIL\" parado (versao anterior, gravado em disco): rode sudo $0 para remover"
  fi
  [ -x "$DISPATCHER" ] && ok "NAT do VIDA instalado ($DISPATCHER)" || aviso "NAT do VIDA nao instalado"
  [ -x "$CLI" ] && ok "Comando vida-rede instalado" || aviso "Comando vida-rede nao instalado"
  if systemctl is-active --quiet vida-rede 2>/dev/null; then
    ok "Servico vida-rede ativo (botao do painel)"
  else
    aviso "Servico vida-rede parado: o botao Conectar a Odroid do painel nao aparece"
  fi

  if [ -r /etc/ufw/ufw.conf ] && grep -q '^ENABLED=yes' /etc/ufw/ufw.conf; then
    aviso "Firewall ufw ligado: o NAT do VIDA libera a porta do cabo enquanto o perfil estiver ativo"
  else
    ok "Firewall ufw desligado"
  fi

  titulo "Docker"
  if command -v docker >/dev/null; then
    ok "Docker instalado"
    docker compose version >/dev/null 2>&1 && ok "docker compose presente" \
      || falha "Falta o plugin docker compose (sudo apt install docker-compose-plugin)"
    if id -nG "$USUARIO" | tr ' ' '\n' | grep -qx docker; then
      ok "Usuario $USUARIO no grupo docker"
    else
      falha "Usuario $USUARIO fora do grupo docker (sudo usermod -aG docker $USUARIO; depois sair e entrar na sessao)"
    fi
    if docker info >/dev/null 2>&1; then
      local img
      for img in vida-ros-ws:humble vida-painel:humble; do
        docker image inspect "$img" >/dev/null 2>&1 && ok "Imagem $img construida" \
          || aviso "Imagem $img ainda nao construida (docker compose build)"
      done
    else
      aviso "Sem acesso ao Docker agora: imagens nao conferidas"
    fi
  else
    falha "Docker nao instalado (https://docs.docker.com/engine/install/ubuntu/)"
  fi

  titulo "Projeto"
  local repo
  for repo in projeto-vida-controle-alto-nivel projeto-vida-painel; do
    [ -d "$PROJETO/$repo" ] && ok "$PROJETO/$repo" || aviso "Falta $PROJETO/$repo"
  done
  for repo in analises rosbag; do
    [ -d "$PROJETO/$repo" ] && ok "$PROJETO/$repo" || aviso "Falta a pasta $PROJETO/$repo (mkdir -p $PROJETO/$repo)"
  done

  titulo "Controle (gamepad)"
  if command -v bluetoothctl >/dev/null; then
    if bluetoothctl devices 2>/dev/null | grep -qi 'wireless controller'; then
      ok "Controle DS4 pareado"
    else
      aviso "Controle DS4 nao pareado: Configuracoes > Bluetooth, segurar PS + Share ate piscar"
    fi
  else
    aviso "Bluetooth indisponivel: o controle tambem funciona por cabo USB"
  fi

  echo
  if [ "$FALHAS" -eq 0 ]; then echo "Nenhuma falha."; else echo "$FALHAS falha(s): ver acima."; fi
}

instalar() {
  exigir_root
  titulo "Conferindo antes de instalar"
  command -v nmcli >/dev/null && systemctl is-active --quiet NetworkManager \
    || { echo "NetworkManager nao esta ativo: nada foi alterado." >&2; exit 1; }
  if ! checar_conflitos; then
    echo
    echo "Este computador ja tem outra configuracao para a Odroid (por exemplo, o notebook"
    echo "principal do VIDA). Nada foi alterado. Para migrar, remova a configuracao antiga"
    echo "antes, conscientemente, e rode este script de novo."
    exit 1
  fi
  ok "Sem configuracao antiga"

  titulo "Instalando"
  install -m 0755 -o root -g root "$DIR/90-vida-odroid" "$DISPATCHER"
  ok "NAT do VIDA em $DISPATCHER"
  install -m 0755 -o root -g root "$DIR/vida-rede" "$CLI"
  ok "Comando $CLI"
  install -m 0644 -o root -g root "$DIR/vida-rede.service" "$SERVICO"
  systemctl daemon-reload
  systemctl enable vida-rede.service >/dev/null 2>&1
  systemctl restart vida-rede.service
  ok "Servico vida-rede ativo (botao do painel)"

  # Versao anterior deste script gravava o perfil sem porta fixa: apagar e
  # devolver as portas cabeadas aos perfis automaticos (DHCP).
  if perfil_existe; then
    VIDA_REDE_SEM_SUDO=1 "$CLI" desconectar >/dev/null 2>&1 || true
    ok "Perfil \"$PERFIL\" da versao anterior removido"
  fi
  reativar_portas

  verificar
  cat <<EOF

Pronto. Se o painel ja estava rodando, recrie-o uma vez para ele enxergar o servico:
  cd $PROJETO/projeto-vida-painel && docker compose up -d --force-recreate

Com o cabo ligado na Odroid:
  botao "Conectar a Odroid" no painel, ou: vida-rede conectar
  vida-rede desconectar   # a porta volta ao normal (DHCP); pede a senha (sudo)
EOF
}

# Portas cabeadas paradas (ex.: depois de apagar o perfil antigo) voltam ao
# perfil automatico (DHCP), como ao ligar um cabo. -w 0: nao espera o DHCP.
reativar_portas() {
  local dev
  for dev in $(nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null |
               awk -F: '$2 == "ethernet" && $3 == "disconnected" { print $1 }'); do
    nmcli -w 0 device connect "$dev" >/dev/null 2>&1 || true
  done
}

remover() {
  exigir_root --remover
  titulo "Removendo"
  if perfil_existe; then
    # "down" antes de apagar: o dispatcher tira as regras no "down".
    if [ -x "$CLI" ]; then
      VIDA_REDE_SEM_SUDO=1 "$CLI" desconectar >/dev/null 2>&1 || true
    fi
    for u in $(nmcli -t -f UUID,NAME connection show | awk -F: -v p="$PERFIL" '$2 == p { print $1 }'); do
      nmcli connection delete uuid "$u" >/dev/null 2>&1 || true
    done
    reativar_portas
    ok "Perfil \"$PERFIL\" removido"
  fi
  if [ -e "$SERVICO" ]; then
    systemctl disable --now vida-rede.service >/dev/null 2>&1 || true
    rm -f "$SERVICO"
    systemctl daemon-reload
    rm -rf /run/vida-rede
    ok "Servico vida-rede removido"
  fi
  rm -f "$DISPATCHER" "$CLI"
  ok "NAT do VIDA e comando vida-rede removidos"
}

case "${1:-}" in
  --verificar) verificar; [ "$FALHAS" -eq 0 ] ;;
  --remover)   remover ;;
  "")          instalar ;;
  *) echo "Uso: $0 [--verificar|--remover]" >&2; exit 64 ;;
esac
