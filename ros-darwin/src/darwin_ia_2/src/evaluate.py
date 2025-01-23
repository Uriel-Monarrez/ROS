#!/usr/bin/env python3
from gazebo_env import GazeboEnv
import numpy as np

# Cargar la tabla Q entrenada
q_table = np.load("q_table.npy")

# Configurar el entorno
env = GazeboEnv()

for episode in range(10):
    state = env.reset()
    total_reward = 0
    done = False

    while not done:
        action = np.argmax(q_table[state])
        next_state, reward, done, _ = env.step(action)
        state = next_state
        total_reward += reward

    print(f"Episodio {episode + 1}, Recompensa Total: {total_reward}")