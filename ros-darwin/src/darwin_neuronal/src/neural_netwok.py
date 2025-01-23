import tensorflow as tf
import rospy
import rospkg
import keras
import os
learning_rate = rospy.get_param("/learning_rate")
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('darwin_neuronal')
outdir = pkg_path + '/training_results'
class DQN(tf.keras.Model):
    def __init__(self, num_actions):
        super(DQN, self).__init__()
        self.dense1 = tf.keras.layers.Dense(64, activation='tanh')

        self.dense2 = tf.keras.layers.Dense(64, activation='tanh')
        self.dense3 = tf.keras.layers.Dense(64, activation='tanh')

        self.output_layer = tf.keras.layers.Dense(
            num_actions, activation='tanh')
        rospy.loginfo("Whats going on heeeere")
 
    def call(self, inputs):
        x = self.dense1(inputs)
        x = self.dense2(x)
        x = self.dense3(x)
        rospy.loginfo("Se calio: "+str(self.output_layer(x)))
        return self.output_layer(x)
    
    def cargar_modelo(self):
        # Crear directorio si no existe
        if not os.path.exists(outdir):
            rospy.loginfo("No se encontraron pesos previos. Entrenamiento comenzará desde cero.")
            return

        # Inicializar variables del modelo
        entrada_ficticia = tf.zeros((1,17))  # Ajusta según tu entrada
        self(entrada_ficticia)

        # Cargar pesos
        pesos_path = os.path.join(outdir, 'my_weights.h5')
        if os.path.exists(pesos_path):
            self.load_weights(pesos_path)
            rospy.loginfo(f"Pesos cargados desde: {pesos_path}")
        else:
            rospy.loginfo("Archivo de pesos no encontrado. Entrenamiento comenzará desde cero.")

    def guardar_pesos(self):
        # Crear directorio si no existe
        if not os.path.exists(outdir):
            os.makedirs(outdir)
        
        # Guardar pesos en el disco
        #Despues aguardar los mejores pesos
        pesos_path = os.path.join(outdir, 'my_weights.h5')
        self.save_weights(pesos_path)
        rospy.loginfo(f"Pesos guardados en: {pesos_path}")
    

