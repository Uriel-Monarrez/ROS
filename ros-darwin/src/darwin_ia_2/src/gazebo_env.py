#!/usr/bin/env python3
import gym
from gym import spaces
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import numpy as np

class GazeboEnv(gym.Env):
    def __init__(self):
        super(GazeboEnv, self).__init__()
        
        # Espacio de observación: ángulos de 4 articulaciones principales (por ejemplo)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(6,), dtype=np.float32)
        
        # Espacio de acción: cambios en los ángulos de las articulaciones [-0.1, 0, 0.1]
        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3 , 3 ,3])  # Una acción para cada articulación
        
        # Inicializa ROS y suscriptores/publicadores
        rospy.init_node('gym_gazebo_env', anonymous=True)
        self.pub_ankle1_r = rospy.Publisher('/darwin/j_ankle1_r_position_controller/command', Float64, queue_size=10)
        self.pub_ankle2_r = rospy.Publisher('/darwin/j_ankle2_r_position_controller/command', Float64, queue_size=10)
        self.pub_pelvis_r = rospy.Publisher('/darwin/j_pelvis_r_position_controller/command', Float64, queue_size=10)
        self.pub_tibia_r = rospy.Publisher('/darwin/j_tibia_r_position_controller/command', Float64, queue_size=10)
        self.pub_thigh2_r = rospy.Publisher('/darwin/j_thigh2_r_position_controller/command', Float64, queue_size=10)
        self.pub_thigh1_r = rospy.Publisher('/darwin/j_thigh1_r_position_controller/command', Float64, queue_size=10)
        self.sub_joint_states = rospy.Subscriber('/darwin/joint_states', JointState, self._joint_state_callback)
        self.sub_imu = rospy.Subscriber("/darwin/imu", Imu)
        self.joint_positions = np.zeros(6)  # Posiciones de las articulaciones relevantes

    def _joint_state_callback(self, data):
        # Actualiza las posiciones de las articulaciones según los datos recibidos
        joint_names = ['j_ankle1_r_position_controller', 'j_ankle2_r_position_controller', 'j_pelvis_r_position_controller', 'j_tibia_r_position_controller','j_thigh2_r_position_controller','j_thigh1_r_position_controller']
        for i, name in enumerate(joint_names):
            idx = data.name.index(name)
            self.joint_positions[i] = data.position[idx]

    def reset(self):
        # Reinicia la simulación y las posiciones iniciales de las articulaciones
        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
        rospy.sleep(1.0)  # Esperar a que se reinicie Gazebo
        return self.joint_positions

    def step(self, action):
        # Aplica cambios incrementales a las articulaciones
        deltas = [-0.1, 0, 0.1]  # Cambios posibles
        #commands = self.joint_positions + np.array([deltas[a] for a in action])
        
        delta_array = np.zeros_like(self.joint_positions)

        # Aplica un cambio solo a la articulación seleccionada
        delta_array[action] = deltas[1]  # Por ejemplo, seleccionamos el delta "0" para esa articulación

        commands = self.joint_positions + delta_array

        self.pub_ankle1_r.publish(commands[0])
        self.pub_ankle2_r.publish(commands[1])
        self.pub_pelvis_r.publish(commands[2])
        self.pub_tibia_r.publish(commands[3])
        self.pub_thigh2_r.publish(commands[4])
        self.pub_thigh1_r.publish(commands[5])
        
        rospy.sleep(0.1)  # Dejar tiempo para que el robot reaccione
        
        # Calcula la recompensa (por ejemplo, distancia recorrida hacia adelante)
        reward = self._calculate_reward()
        done = self._check_done()
        
        return self.joint_positions, reward, done, {}

    def _calculate_reward(self):
        # Implementa una recompensa basada en la distancia hacia adelante o estabilidad
        # Por simplicidad, se puede usar un placeholder como recompensa
        
        return 1.0  # Recompensa constante por dar un paso

    def _check_done(self):
        # Termina el episodio si el robot cae o tiene un problema de estabilidad
        return False  # Placeholder, se debe implementar según el criterio de caída

    def render(self, mode='human'):
        pass

    def close(self):
        rospy.signal_shutdown("Cerrando entorno")