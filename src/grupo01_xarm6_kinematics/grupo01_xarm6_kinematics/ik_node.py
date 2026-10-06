#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        # Suscriptor al tópico /target (geometry_msgs/msg/Point)
        self.sub = self.create_subscription(Point, '/target', self.target_cb, 10)
        # Publicador de la solución al tópico /joint_states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Parámetros del algoritmo iterativo
        self.alpha = 0.5            # Factor de actualización
        self.epsilon = 0.001        # Tolerancia de error (1 mm)
        self.max_iter = 100         # Número máximo de iteraciones
        
        # Configuración inicial (semilla q0)
        self.q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.get_logger().info("Nodo IK iniciado. Esperando objetivo (x, y, z) en /target...")

    def dh_matrix(self, theta, d, a, alpha):
        c_t, s_t = np.cos(theta), np.sin(theta)
        c_a, s_a = np.cos(alpha), np.sin(alpha)
        return np.array([
            [c_t, -s_t*c_a,  s_t*s_a, a*c_t],
            [s_t,  c_t*c_a, -c_t*s_a, a*s_t],
            [0,    s_a,      c_a,     d],
            [0,    0,        0,       1]
        ])

    def forward_kinematics(self, q):
        # Desfases especiales del xArm6
        th2 = q[1] - 1.3849
        th3 = q[2] + 1.3849
        A1 = self.dh_matrix(q[0], 0.267, 0.0, -np.pi/2)
        A2 = self.dh_matrix(th2, 0.0, 0.28949, 0.0)
        A3 = self.dh_matrix(th3, 0.0, 0.0775, -np.pi/2)
        A4 = self.dh_matrix(q[3], 0.3425, 0.0, np.pi/2)
        A5 = self.dh_matrix(q[4], 0.0, 0.076, -np.pi/2)
        A6 = self.dh_matrix(q[5], 0.097, 0.0, 0.0)
        T06 = A1 @ A2 @ A3 @ A4 @ A5 @ A6
        return T06[0:3, 3] # Solo retorna Px, Py, Pz

    def positional_jacobian(self, q):
        # Jacobiano posicional calculado numéricamente
        delta = 1e-5
        J = np.zeros((3, 6))
        p0 = self.forward_kinematics(q)
        for i in range(6):
            q_temp = q.copy()
            q_temp[i] += delta
            p_temp = self.forward_kinematics(q_temp)
            J[:, i] = (p_temp - p0) / delta
        return J

    def target_cb(self, msg):
        pd = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"--- Objetivo recibido: X={pd[0]:.3f}, Y={pd[1]:.3f}, Z={pd[2]:.3f} ---")
        
        qk = self.q0.copy()
        converged = False
        
        for k in range(self.max_iter):
            pk = self.forward_kinematics(qk)
            e = pd - pk # Error euclidiano
            error_norm = np.linalg.norm(e)
            
            if error_norm < self.epsilon:
                converged = True
                self.get_logger().info(f"Convergencia en iteración {k}. Error final: {error_norm:.6f} m")
                break
                
            J = self.positional_jacobian(qk)
            J_pinv = np.linalg.pinv(J) # Pseudoinversa
            qk = qk + self.alpha * (J_pinv @ e) # Actualización
            
        if converged:
            self.publish_joint_states(qk)
            self.q0 = qk # Actualiza la semilla para el próximo movimiento
        else:
            self.get_logger().warn(f"No convergió tras {self.max_iter} iteraciones. Error: {error_norm:.6f} m")

    def publish_joint_states(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = q.tolist()
        self.pub.publish(msg)
        self.get_logger().info("Solución articular publicada en /joint_states.")

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()