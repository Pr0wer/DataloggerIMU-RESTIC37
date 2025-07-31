import pandas as pd
import matplotlib.pyplot as plt

# Lê o arquivo CSV
df = pd.read_csv("ArquivosDados/mpu_data.csv")

 # Converte o tempo de milissegundos para segundos para melhor visualização
df["time_s"] = df["time_ms"] / 1000.0

# Gráfico de Aceleração
plt.figure(figsize=(12, 6)) # Define o tamanho da figura
plt.plot(df["time_s"], df["accel_x"], label="Accel X", linewidth=1.5)
plt.plot(df["time_s"], df["accel_y"], label="Accel Y", linewidth=1.5)
plt.plot(df["time_s"], df["accel_z"], label="Accel Z", linewidth=1.5)
plt.title("Dados de Aceleração do MPU6050 ao longo do Tempo", fontsize=16)
plt.xlabel("Tempo (s)", fontsize=12)
plt.ylabel("Aceleração (g)", fontsize=12)
plt.legend(loc="upper right", fontsize=10) # Posição da legenda
plt.grid(True, linestyle='--', alpha=0.7) # Adiciona grade com estilo
plt.tight_layout() # Ajusta o layout para evitar sobreposição

# Gráfico de Giroscópio
plt.figure(figsize=(12, 6)) # Define o tamanho da figura
plt.plot(df["time_s"], df["giro_x"], label="Giro X", linewidth=1.5)
plt.plot(df["time_s"], df["giro_y"], label="Giro Y", linewidth=1.5)
plt.plot(df["time_s"], df["giro_z"], label="Giro Z", linewidth=1.5)
plt.title("Dados de Giroscópio do MPU6050 ao longo do Tempo", fontsize=16)
plt.xlabel("Tempo (s)", fontsize=12)
plt.ylabel("Velocidade Angular (°/s)", fontsize=12)
plt.legend(loc="upper right", fontsize=10)
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()

# Mostrar todos os gráficos
plt.show()