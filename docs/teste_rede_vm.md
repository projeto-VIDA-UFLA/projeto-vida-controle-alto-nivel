# Testar a conexão com a Odroid numa máquina virtual

Teste do [novo_computador.md](novo_computador.md) sem o VIDA: uma VM com Ubuntu faz o papel
do computador novo e um servidor com a mesma configuração de rede da Odroid
(`192.168.10.1`, gateway `192.168.10.2`) faz o papel da Odroid.

Valida a parte de rede: o perfil "VIDA Odroid", o NAT, o botão do painel e a volta ao
normal. A comunicação ROS 2 com a Odroid de verdade só dá para ver no carro.

## 1. Configurar o VMware (no Windows)

Use o **VMware Workstation** (o Player não tem o editor de redes).

**VM:** Ubuntu **Desktop** 22.04, não o Server: o script depende do NetworkManager, que só
vem no Desktop. Pelo menos 4 GB de RAM, 4 núcleos e 40 GB de disco (a imagem do ROS é pesada).

**Rede 1, a internet** (faz o papel do Wi-Fi): o adaptador que já vem na VM, em **NAT**.

**Rede 2, o cabo até o servidor:**
1. **Edit → Virtual Network Editor → Change Settings** (pede administrador).
2. **Add Network → VMnet2**, marcar **Bridged** e, em **"Bridged to:"**, escolher a placa
   **cabeada física** do notebook (Realtek, Intel Ethernet ou similar). Não usar "Automatic":
   ele pode pegar o Wi-Fi.
3. Na VM: **VM → Settings → Add → Network Adapter → Custom: VMnet2**, com
   "Connected at power on" marcado.

**Para o Windows não usar o cabo durante o teste:** Painel de Controle → Conexões de Rede →
propriedades da placa cabeada → desmarcar **IPv4** e **IPv6**, deixando marcado o
**"VMware Bridge Protocol"**. Assim só a VM fala pelo cabo. **Depois do teste, marcar os
dois de novo.**

Com isso a VM se comporta como um notebook com Wi-Fi e uma porta de rede. O script ignora
a placa NAT, porque é por ela que a VM sai para a internet (rota padrão).

## 2. Preparar a VM

Seguir o [novo_computador.md](novo_computador.md), etapas 1 a 5 (Git, SSH, Docker,
repositórios e rede). A etapa 6 (controle) não é necessária para este teste.

Antes de ligar o cabo, conferir:

```bash
cd ~/projeto-vida/projeto-vida-controle-alto-nivel/scripts/rede
./preparar_computador.sh --verificar     # nenhuma falha (os avisos de controle podem ficar)
systemctl status vida-rede               # active (running)
```

## 3. Testar

| # | Passo | Esperado |
|---|---|---|
| 1 | Cabo solto. Abrir `http://localhost:8080` na VM | Botão **Conectar à Odroid** cinza: "Ligue o cabo de rede até a Odroid" |
| 2 | Ligar o cabo do notebook até o servidor | Botão fica clicável (em até 2 s) |
| 3 | Clicar em **Conectar à Odroid** | Em poucos segundos o botão vira **Desconectar da Odroid** e o bloco ESP32 / Odroid mostra o tempo do ping (ex.: "Odroid 0,4 ms"). O bloco fica amarelo ou vermelho por falta dos nós ROS: normal, o servidor não roda o VIDA |
| 4 | No terminal da VM: `ip addr` | `192.168.10.2` na porta do cabo |
| 5 | `sudo iptables -S \| grep vida` e `sudo iptables -t nat -S \| grep vida` | 3 regras (INPUT e 2 FORWARD) e 1 MASQUERADE |
| 6 | No servidor: internet pelo cabo (ver abaixo) | Responde |
| 7 | Clicar em **Desconectar da Odroid** e confirmar | Botão volta a **Conectar**; `ip addr` sem `192.168.10.2`; os dois `iptables` acima sem regras |
| 8 | Conectar de novo e **reiniciar a VM** | Volta desconectada, **com internet** pela placa NAT; `nmcli connection show` lista os "Wired connection" e nenhum "VIDA Odroid"; sem regras `vida` |
| 9 | Depois do reinício, ligar a porta do cabo num roteador | Pega IP por DHCP normalmente |
| 10 | Com o servidor desligado (ou o cabo em outro aparelho), clicar em Conectar | Depois de ~10 s: aviso "Odroid sem resposta" |

**Internet do servidor pelo cabo (passo 6).** No servidor, `ip route get 8.8.8.8`:
- mostrou `via 192.168.10.2`: está usando o cabo. Testar com `ping 8.8.8.8`;
- mostrou a outra porta: forçar com `ping -I <porta_do_cabo> 8.8.8.8` (só funciona se houver
  rota padrão pelo `192.168.10.2` nessa porta, como na Odroid). Se não houver, desconectar
  a outra porta durante o teste.

**Pelo terminal** o mesmo teste funciona com `vida-rede conectar`, `vida-rede status` e
`vida-rede desconectar`.

## VM preparada com a versão anterior (perfil gravado)

A primeira versão do script gravava o perfil "VIDA Odroid" sem porta fixa. Com isso, depois
de reiniciar, nenhuma porta cabeada pegava IP (nem a placa NAT da VM). Para recuperar:

```bash
sudo nmcli connection delete "VIDA Odroid"   # devolve a internet à placa NAT
sudo nmcli device connect ens33              # se ela não voltar sozinha (ver o nome em: nmcli device)
cd ~/projeto-vida/projeto-vida-controle-alto-nivel && git pull
cd scripts/rede && sudo ./preparar_computador.sh   # instala a versão nova
```

## Se algo falhar

| Sintoma | Onde olhar |
|---|---|
| Botão não aparece | `systemctl status vida-rede`; `ls /run/vida-rede` deve ter `estado.json`. Se o painel subiu antes da instalação: `cd ~/projeto-vida/projeto-vida-painel && docker compose up -d --force-recreate` |
| "Mais de uma porta com cabo" | A placa NAT não está como rota padrão: `ip route show default` |
| Conecta, mas sem ping | Cabo na placa certa? No VMware, VMnet2 em bridge com a placa cabeada? IPv4 do Windows desmarcado na placa? |
| Servidor sem internet pelo cabo | `sudo iptables -t nat -S \| grep vida` na VM; `cat /proc/sys/net/ipv4/ip_forward` deve ser 1 |
| Histórico das ações | `journalctl -t vida-rede -t vida-odroid` |
