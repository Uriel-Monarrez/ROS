#!/usr/bin/env python3

import gym
import time
from gym import wrappers
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg
import neural_netwok
import numpy as np
import tensorflow as tf
import gym
# import our training environment
import monoped_env


if __name__ == '__main__':
    
    rospy.init_node('monoped_gym', anonymous=True, log_level=rospy.INFO)

    learning_rate = rospy.get_param("/learning_rate")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")
    discount_factor= rospy.get_param("/discount_factor")
    # Initial exploration probability
    exploration_prob= rospy.get_param("/exploration_prob")
    # Decay rate of exploration probability
    exploration_decay= rospy.get_param("/exploration_decay")
    # Minimum exploration probability
    min_exploration_prob= rospy.get_param("/min_exploration_prob")

    num_actions = rospy.get_param("/num_actions")
   

    # Create the Gym environment
    env = gym.make('Monoped-v0')
    rospy.logdebug ( "Gym environment done")
    
    reward_pub = rospy.Publisher('/darwin/reward', Float64, queue_size=1)
    episode_reward_pub = rospy.Publisher('/darwin/episode_reward', Float64, queue_size=1)

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('darwin_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")
    
    last_time_steps = np.ndarray(0)

    dqn_agent = neural_netwok.DQN(num_actions)
    
    loss_fn = tf.keras.losses.MeanSquaredError()
    optimizer = tf.keras.optimizers.Adam(learning_rate=learning_rate)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    """
    Alpha = rospy.get_param("/alpha")
    Epsilon = rospy.get_param("/epsilon")
    Gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    """
    # Initialises the algorithm that we are going to use for learning

    dqn_agent.cargar_modelo()
 
    start_time = time.time()
    highest_reward = 0
    
    # Starts the main training loop: the one about the episodes to do
    
    
    for episode in range(nepisodes):
        rospy.loginfo ("STARTING Episode #"+str(episode))
        
        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        
        
        
        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        # Now We return directly the stringuified observations called state
        state = env.reset()
        episode_reward = 0
        #rospy.logdebug("env.get_state...==>"+str(state))
        
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):

            # Pick an action based on the current state
            if np.random.rand() < exploration_prob:
                action = env.action_space.sample()# Explore randomly
                rospy.loginfo("Random: "+str(action))
            else:
                action = dqn_agent(state[np.newaxis, :]).numpy().flatten()
                rospy.loginfo("DQN: "+str(action))

            
            next_state, reward, done, _ = env.step(action)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logdebug("env.get_state...[distance_from_desired_point,base_roll,base_pitch,base_yaw,contact_force,joint_states_haa,joint_states_hfe,joint_states_kfe]==>" + str(next_state))

            # Update the Q-values using Bellman equation
            
                
            with tf.GradientTape() as tape:

                #rospy.loginfo("Tipo de state: "+str(type(state))+", Valor de state: "+str(state))

                # Predicción de Q-values actuales y futuros
                current_q_values = dqn_agent(state[np.newaxis, :])
                next_q_values = dqn_agent(next_state[np.newaxis, :])
                
                # Calcula el valor máximo de Q del siguiente estado
                max_next_q = tf.reduce_max(next_q_values, axis=-1).numpy()
                
                # Calcula el objetivo: recompensa + factor de descuento * max_next_q
                target_q_values = current_q_values.numpy()
                target_q_values[0] = reward + discount_factor * max_next_q * (1 - done)

                # Calcula la pérdida
                loss = loss_fn(current_q_values, target_q_values)
    
            # Aplica los gradientes para optimizar el modelo
            gradients = tape.gradient(loss, dqn_agent.trainable_variables)
            optimizer.apply_gradients(zip(gradients, dqn_agent.trainable_variables))
    
            state = next_state
            episode_reward += reward

            # We publish the cumulated reward
            

            dqn_agent.guardar_pesos()
            if done:
                break
 
        # Decay exploration probability
        exploration_prob = max(min_exploration_prob, exploration_prob * exploration_decay)
        print(f"Episode {episode + 1}: Reward = {episode_reward}")