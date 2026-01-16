# ============================================
# VIDA - Telemetria do veículo (ESP1 -> ESP2 -> ROS2)
# ============================================


# - stamp: tempo ROS
std_msgs/Header header

# Timestamp da amostragem no ESP1 (micros desde boot).
uint64 t_sample_us

# Delta T entre esta amostra e a anterior (ESP1), em microssegundos.
uint32 dt_us

# ----------------------------
# Esterçamento
# ----------------------------
float32 steer_angle_deg
float32 steer_rate_rad_s

# ----------------------------
# Longitudinal (médias de Velocidade e Aceleração)
# ----------------------------
float32 v_long_mean_mps
float32 a_long_mean_mps2

# ----------------------------
# Velocidade por encoder
# ----------------------------
float32 v_enc_r_mps
float32 v_enc_l_mps
float32 v_enc_rear_mps

# ----------------------------
# Aceleração por encoder
# ----------------------------
float32 a_enc_r_mps2
float32 a_enc_l_mps2
float32 a_enc_rear_mps2

# ----------------------------
# Contadores de ticks (odometria)
# ----------------------------
int32 ticks_total_r
int32 ticks_total_l
int32 ticks_total_rear