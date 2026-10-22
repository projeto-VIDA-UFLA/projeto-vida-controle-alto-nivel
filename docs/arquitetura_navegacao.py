"""Diagrama da arquitetura de navegacao do VIDA (nos ROS 2 por maquina).

Uso: python3 arquitetura_navegacao.py  ->  arquitetura_navegacao.{pdf,png,svg}
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

plt.rcParams.update({"font.family": "DejaVu Sans"})
BLUE, GREEN, ORANGE, GRAY = "#2a6fb0", "#2e8b57", "#d35400", "#888888"
NEW, OLD = "#dce9f7", "#eeeeee"

fig, ax = plt.subplots(figsize=(18, 10))
ax.set_xlim(0, 18); ax.set_ylim(0, 10); ax.axis("off")

def machine(x0, x1, y0, y1, title, sub, color, fill):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0.02,rounding_size=0.25",
                                fc=fill, ec=color, lw=3))
    ax.text((x0 + x1) / 2, y1 - 0.35, title, ha="center", va="center", fontsize=17, weight="bold", color=color)
    ax.text((x0 + x1) / 2, y1 - 0.72, sub, ha="center", va="center", fontsize=11, color=color)

W, H = 2.25, 0.85
def node(x, y, title, sub=None, new=False, dashed=False, color=None, w=W):
    ec = color or (BLUE if new else GRAY)
    ax.add_patch(FancyBboxPatch((x - w / 2, y - H / 2), w, H, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc="white" if dashed else (NEW if new else OLD), ec=ec, lw=1.8,
                                ls="--" if dashed else "-"))
    if sub:
        ax.text(x, y + 0.13, title, ha="center", va="center", fontsize=12, weight="bold", color=ec if dashed else "black")
        ax.text(x, y - 0.20, sub, ha="center", va="center", fontsize=9, color="#555555")
    else:
        ax.text(x, y, title, ha="center", va="center", fontsize=12, weight="bold", color=ec if dashed else "black")

def arrow(p0, p1, label=None, color="#555555", lw=1.6, ls="-", both=False, lpos=0.5, loff=(0, 0.17), lcolor=None, bold=False, rad=0.0):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="<|-|>" if both else "-|>", color=color, lw=lw, ls=ls,
                                shrinkA=2, shrinkB=2, mutation_scale=14, connectionstyle=f"arc3,rad={rad}"))
    if label:
        x = p0[0] + (p1[0] - p0[0]) * lpos + loff[0]
        y = p0[1] + (p1[1] - p0[1]) * lpos + loff[1]
        ax.text(x, y, label, ha="center", va="center", fontsize=9.5, color=lcolor or color,
                weight="bold" if bold else "normal",
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.9))

ax.text(9.0, 9.75, "VIDA — nós ROS 2 e onde cada um roda", ha="center", fontsize=19, weight="bold")

# Maquinas
machine(0.3, 6.1, 0.9, 8.4, "NOTEBOOK", "nível 3 · trajetória", BLUE, "#f6f9fd")
machine(7.4, 13.6, 0.9, 8.4, "ODROID", "nível 2 · estimação e sensores", GREEN, "#f5faf7")
machine(14.4, 17.7, 0.9, 8.4, "ESP32", "nível 1 · controle e segurança", ORANGE, "#fdf8f3")

R1, R2, R3 = 6.45, 4.4, 2.1   # linhas

# Notebook
node(1.75, R1, "path_publisher", "caminho.csv → Path", new=True)
node(4.65, R1, "stanley", "lei de Stanley", new=True)
node(4.65, R2, "mode_manager", "manual ↔ automático", new=True)
node(4.65, R3, "gamepad_reader", "manual · △ / □")
node(1.75, R3, "joy_node", "controle PS4")

# Odroid
node(8.85, R1, "robot_state_ekf", "migrado do notebook", new=True)
node(8.85, R2, "micro_ros_agent", "ponte serial")
node(8.85, R3, "security", "enable lógico")
node(12.05, R1, "zed_f9r_driver", "GPS RTK + IMU")
node(12.05, 5.15, "ntrip_client", "correções RTK")

# ESP32
node(16.05, R2, "esp32_controller", "esterçamento · velocidade", w=2.8)
ax.text(16.05, R2 - 0.68, "segurança · supervisor", ha="center", fontsize=9, color="#555555")

# Hardware (fora das caixas)
node(12.05, 9.05, "antena GNSS", dashed=True, color=GRAY, w=1.9)
node(16.05, R1, "emergência\npor rádio", dashed=True, color=ORANGE, w=2.6)
node(16.05, R3, "motores · freio\nencoders · LVDT", dashed=True, color=GRAY, w=2.6)

# --- Dentro do notebook
arrow((2.9, R1), (3.5, R1), "caminho", loff=(0, 0.2))
arrow((4.45, R1 - 0.45), (4.45, R2 + 0.45), "ref.\nautomática", lpos=0.5, loff=(-0.75, 0))
arrow((1.75 + W / 2, R3), (4.65 - W / 2, R3), "joy", loff=(0, 0.2))
arrow((4.45, R3 + 0.45), (4.45, R2 - 0.45), "ref. manual\n△ / □", loff=(-0.8, 0))
arrow((4.95, R2 - 0.45), (4.95, R3 + 0.45), "modo\n(vibração)", ls=":", loff=(0.62, 0))

# --- Dentro do Odroid
arrow((12.05, 8.62), (12.05, R1 + 0.45))
arrow((12.05, 5.15 + 0.45), (12.05, R1 - 0.45), "RTCM", loff=(0.42, 0))
arrow((12.05 - W / 2, R1), (8.85 + W / 2, R1), "fix · IMU\nstatus RTK", loff=(0, 0.42))
arrow((8.85, R2 + 0.45), (8.85, R1 - 0.45), "encoders · LVDT", loff=(0, 0))
arrow((8.85, R3 + 0.45), (8.85, R2 - 0.45), "enable lógico", loff=(0, 0))

# --- Notebook <-> Odroid (Wi-Fi)
arrow((8.85 - W / 2, R1 + 0.15), (4.65 + W / 2, R1 + 0.15), "odom 50 Hz", loff=(0, 0.2))
arrow((8.85 - W / 2, R1 + 0.32), (1.75, R1 + 0.45), "origem", rad=0.13, lpos=0.5, loff=(0, 0.5))
arrow((8.85 - W / 2, R1 - 0.25), (4.65 + W / 2, R2 + 0.2), "odom · status", lpos=0.45, loff=(0.25, 0.2))
arrow((4.65 + W / 2, R2 - 0.05), (8.85 - W / 2, R1 - 0.42), "reset", ls="--", color=BLUE, lpos=0.62, loff=(0.25, -0.2))
arrow((4.65 + W / 2, R2 - 0.25), (8.85 - W / 2, R2 - 0.25), "referências\nesterçamento · velocidade",
      color=BLUE, lw=2.6, bold=True, loff=(0, -0.42))
arrow((4.65 + W / 2, R3), (8.85 - W / 2, R3), "gamepad ativo", loff=(0, 0.2))

# --- Odroid <-> ESP32 (serial) e hardware
arrow((8.85 + W / 2, R2), (16.05 - 1.4, R2), "USB serial · micro-ROS", color=ORANGE, lw=2.6, both=True, lpos=0.5, loff=(0, -0.3))
arrow((16.05, R1 - 0.45), (16.05, R2 + 0.45), "enable físico", color=ORANGE, loff=(0, 0))
arrow((16.05, R2 - 0.95), (16.05, R3 + 0.45), color=GRAY, both=True)

ax.text(3.2, 1.25, "cabo de rede ponto a ponto (ROS 2 / DDS)", ha="center", fontsize=10, color=BLUE, style="italic")
ax.text(12.1, 1.25, "USB serial até o ESP32", ha="center", fontsize=10, color=ORANGE, style="italic")

# Legenda
ly = 0.35
items = [(NEW, BLUE, "nó novo"), (OLD, GRAY, "nó existente")]
x = 3.4
for fc, ec, lab in items:
    ax.add_patch(FancyBboxPatch((x, ly - 0.15), 0.5, 0.3, boxstyle="round,pad=0.01,rounding_size=0.06", fc=fc, ec=ec, lw=1.5))
    ax.text(x + 0.65, ly, lab, va="center", fontsize=10); x += 2.4
for ls, col, lab in (("-", "#555555", "tópico"), ("--", BLUE, "serviço"), ("-", ORANGE, "serial / sinal elétrico")):
    ax.plot([x, x + 0.6], [ly, ly], ls=ls, color=col, lw=2); ax.text(x + 0.75, ly, lab, va="center", fontsize=10); x += 2.3 if lab != "serial / sinal elétrico" else 3

for ext in ("pdf", "png", "svg"):
    fig.savefig(f"arquitetura_navegacao.{ext}", dpi=150 if ext == "png" else None, bbox_inches="tight")
