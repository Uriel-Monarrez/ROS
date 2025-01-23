#!/usr/bin/env python3
from gazebo_env import GazeboEnv
from q_learning_agent import QLearningAgent
import numpy as np
import rospy
 
# Configurar el entorno y el agente
Alpha = rospy.get_param("/alpha")
Epsilon = rospy.get_param("/epsilon")
Gamma = rospy.get_param("/gamma")

env = GazeboEnv()
agent = QLearningAgent(actions=env.action_space.nvec,epsilon=Epsilon,alpha=Alpha,gamma=Gamma)

episodes = 500
for episode in range(episodes):
    state = env.reset()
    total_reward = 0
    done = False

    while not done:
        action = agent.chooseAction(state)
        next_state, reward, done, _ = env.step(action)
        agent.learn(state, action, reward, next_state)
        state = next_state
        total_reward += reward

    print(f"Episodio {episode + 1}/{episodes}, Recompensa Total: {total_reward}")

# Guardar la tabla Q
np.save("q_table.npy", agent.q_table)
print("Entrenamiento completado y tabla Q guardada.")