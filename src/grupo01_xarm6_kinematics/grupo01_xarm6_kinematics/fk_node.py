#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        # Suscripción obligatoria a /joint_states según la plantilla
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        
        # Timer para mostrar los resultados en consola cada 1 segundo
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.latest_q = None

    def listener_callback(self, msg):
        # Leer y ordenar correctamente las articulaciones del xArm6
        try:
            q = [0.0] * 6
            names = msg.name
            for i in range(1, 7):
                idx = names.index(f'joint{i}')
                q[i-1] = msg.position[idx]
            self.latest_q = q
        except ValueError:
            pass

    def dh_matrix(self, theta, d, a, alpha):
        # Evaluación de la matriz DH estándar
        c_t = np.cos(theta)
        s_t = np.sin(theta)
        c_a = np.cos(alpha)
        s_a = np.sin(alpha)
        
        return np.array([
            [c_t, -s_t*c_a,  s_t*s_a, a*c_t],
            [s_t,  c_t*c_a, -c_t*s_a, a*s_t],
            [0,    s_a,      c_a,     d],
            [0,    0,        0,       1]
        ])

    def timer_callback(self):
        if self.latest_q is None:
            return

        q1, q2, q3, q4, q5, q6 = self.latest_q

        # Casos especiales de desfase angular para el xArm6
        th2 = q2 - 1.3849
        th3 = q3 + 1.3849

        # Obtención de las matrices individuales A_i-1^i
        A1 = self.dh_matrix(q1, 0.267, 0.0, -np.pi/2)
        A2 = self.dh_matrix(th2, 0.0, 0.28949, 0.0)
        A3 = self.dh_matrix(th3, 0.0, 0.0775, -np.pi/2)
        A4 = self.dh_matrix(q4, 0.3425, 0.0, np.pi/2)
        A5 = self.dh_matrix(q5, 0.0, 0.076, -np.pi/2)
        A6 = self.dh_matrix(q6, 0.097, 0.0, 0.0)

        # Cálculo de T_0^n
        T06 = A1 @ A2 @ A3 @ A4 @ A5 @ A6

        # Extracción de la posición (x, y, z)
        px, py, pz = T06[0,3], T06[1,3], T06[2,3]

        self.get_logger().info('--- Cinemática Directa xArm6 ---')
        self.get_logger().info(f'q (rad): [{q1:.3f}, {q2:.3f}, {q3:.3f}, {q4:.3f}, {q5:.3f}, {q6:.3f}]')
        self.get_logger().info(f'Posición del efector: X={px:.4f}, Y={py:.4f}, Z={pz:.4f}')
        self.get_logger().info('--------------------------------')

def main(args=None):
    rclpy.init(args=args)
    fk_node = FKNode()
    rclpy.spin(fk_node)
    fk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()