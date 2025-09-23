# Criar ponto de acesso wifi para conexão com controle

1- Instalar o NetworkManager (se ainda não estiver instalado):
    sudo apt update
    sudo apt install network-manager

2- Criar uma nova conexão Wi-Fi:    
    sudo nmcli connection add type wifi ifname wlan0 con-name VIDA autoconnect yes ssid VIDA
    *substiur wlan0 pela interface correta (verifique usando ifconfig)*

3- Configurar a segurança Wi-Fi com a senha:
    sudo nmcli connection modify VIDA 802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 1
    sudo nmcli connection modify VIDA 802-11-wireless-security.key-mgmt wpa-psk
    sudo nmcli connection modify VIDA 802-11-wireless-security.psk "***"

4- Configurar o IP fixo para a interface:
    sudo nmcli connection modify VIDA ipv4.method manual ipv4.addresses 192.168.1.10/24
    sudo nmcli connection modify VIDA ipv4.gateway 192.168.1.1
    sudo nmcli connection modify VIDA ipv4.dns "8.8.8.8 8.8.4.4"
    sudo nmcli connection modify VIDA ipv6.method ignore
    *informar gateway correto (ip route | grep default)*
    *informar gdns correto (resolvectl status | grep 'DNS Servers' -A 2)*

5- Iniciar o ponto de acesso:
    sudo nmcli connection up VIDA

6- Iniciar o ponto de acesso automáticamente na inicialização (opcional)
    sudo nmcli connection modify VIDA connection.autoconnect yes


    ligar ponto de acesso 

whilker@G15:~$ sudo nano /etc/dnsmasq.d/hostapd.conf
whilker@G15:~$ cd /etc/dnsmasq.d/
whilker@G15:/etc/dnsmasq.d$ ls
ap.conf  README
whilker@G15:/etc/dnsmasq.d$ sudo nano ap.conf 
whilker@G15:/etc/dnsmasq.d$ 
sudo ip addr flush dev wlp0s20f3
whilker@G15:/etc/dnsmasq.d$ sudo ip addr add 10.42.0.1/24 dev wlp0s20f3
whilker@G15:/etc/dnsmasq.d$ sudo ip link set wlp0s20f3 up
whilker@G15:/etc/dnsmasq.d$ sudo systemctl restart dnsmasq
whilker@G15:/etc/dnsmasq.d$ sudo journalctl -u dnsmasq


    sudo hostapd /etc/hostapd/hostapd.conf



nm-connection-editor