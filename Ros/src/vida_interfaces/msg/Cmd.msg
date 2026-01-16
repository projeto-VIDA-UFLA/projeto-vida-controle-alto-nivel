# ============================================
# VIDA - Comando do veículo (ROS2 -> ESP2 -> ESP1)
# ============================================
#
# Convenções:
# - steering em graus (posição) e rad/s (velocidade)
# - longitudinal em m/s e m/s²


std_msgs/Header header


# ----------------------------
# Modo de comando
# ----------------------------
#
# mode:
# 0 = STOP (força setpoints zero e pode acionar freio/neutral)
# 1 = Ativo

uint8 mode

# ----------------------------
# Setpoints
# ----------------------------
float32 steer_pos_sp_deg
float32 steer_vel_sp_rad_s
float32 drive_vel_sp_mps
float32 drive_acc_sp_mps2