#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class DistFinderAlpha(Node):
    def __init__(self):
        super().__init__('dist_finder_alpha')

        # Suscribirse al topic del LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Cambiar este topic si es necesario
            self.getRange,
            10
        )

        # Publicar el mensaje con los resultados (distancia y ángulo)
        self.publisher = self.create_publisher(Float32, 'error', 10)

        # Parámetros de control
        self.setpoint = 1.0  # Setpoint deseado para la distancia al muro, ajustable
        # Parámetro del desplazamiento del vehículo
        self.desplazamiento = 0.3  # En metros

        self.hay_muro = False
        self.contador = 0

        self.ciclo_transicion = 1

        self.get_logger().info("Nodo dist_finder_alphapy iniciado correctamente.")

    def getRange(self, msg: LaserScan):
        # Asumimos que msg.angle_min y msg.angle_increment están en radianes
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Calcular los índices de los ángulos de interés (85 y 90 grados)
        theta = 15  # Diferencia angular entre 85 y 90 grados
        angle_65_deg = math.radians(-75)
        angle_90_deg = math.radians(-90)
        angle_652_deg = math.radians(75)
        angle_902_deg = math.radians(90)
        angle_0_deg = math.radians(0)


        index_65 = int((angle_65_deg - angle_min) / angle_increment)
        index_90 = int((angle_90_deg - angle_min) / angle_increment)
        index_652 = int((angle_652_deg - angle_min) / angle_increment)
        index_902 = int((angle_902_deg - angle_min) / angle_increment)
        index_0 = int((angle_0_deg - angle_min) / angle_increment)

        # Verificar que los índices estén dentro del rango de las mediciones
        if 0 <= index_65 < len(ranges) and 0 <= index_90 < len(ranges) and 0 <= index_652 < len(ranges) and 0 <= index_902 < len(ranges):

            # Obtener las mediciones, usando 12m si hay valores infinitos
            frente = ranges[index_0] if not math.isinf(ranges[index_0]) else 12.0
            a = ranges[index_65] if not math.isinf(ranges[index_65]) else 12.0
            b = ranges[index_90] if not math.isinf(ranges[index_90]) else 12.0
            a2 = ranges[index_652] if not math.isinf(ranges[index_652]) else 12.0
            b2 = ranges[index_902] if not math.isinf(ranges[index_902]) else 12.0

            # Crear el mensaje para publicar (usamos Float32MultiArray)
            msg_out = Float32()

            # Calcular alpha (ángulo de orientación)
            alpha = math.atan2((a * math.cos(math.radians(theta)) - b), (a * math.sin(math.radians(theta))))
            alpha2 = math.atan2((a2 * math.cos(math.radians(theta)) - b2), (a2 * math.sin(math.radians(theta))))
            # Calcular distancia al muro
            distancia_muro = b * math.cos(alpha)
            distancia_muro2 = b2 * math.cos(alpha2)

            # Calcular distancia al muro futura
            distancia_muro_2 = distancia_muro + self.desplazamiento * math.sin(alpha)
            distancia_muro_22 = distancia_muro2 + self.desplazamiento * math.sin(alpha2)

            self.get_logger().info(
                f"a: {a:.2f}, b: {b:.2f}, alpha: {math.degrees(alpha):.2f} grados, "
                f"a2: {a2:.2f}, b2: {b2:.2f}, alpha2: {math.degrees(alpha2):.2f} grados, "
            )

            self.get_logger().info(
                    f"distancia al muro derecho: {distancia_muro:.2f} , distancia al muro derecho futura: {distancia_muro_2:.2f} "
                    f"distancia al muro izquierdo: {distancia_muro2:.2f} , distancia al muro izquierdo futura: {distancia_muro_22:.2f} "
                )

            if ((distancia_muro2 > 3.0 and distancia_muro2 < 4.0 and self.contador >=180) and (not(self.hay_muro))):
               self.hay_muro = True
               self.setpoint_transicion = distancia_muro2

            if (not(self.hay_muro)):

                # Calcular el error
                error = (self.setpoint - distancia_muro_2)

                self.contador = self.contador+1

                if ((frente < 2.0) & (distancia_muro < 2.0)):
                    msg_out.data = 10.0  # Publicamos distancia y ángulo en grados

                    # Publicar el mensaje
                    self.publisher.publish(msg_out)
                else:
                    msg_out.data = error  # Publicamos distancia y ángulo en grados
                    # Publicar el mensaje
                    self.publisher.publish(msg_out)

            elif ((self.hay_muro)):

                self.setpoint_transicion = ((self.ciclo_transicion/600) * (self.setpoint - self.setpoint_transicion))+self.setpoint_transicion

                self.ciclo_transicion += 1  # Sigue aumentando hasta 480 ciclos

                error2 = -(self.setpoint_transicion - distancia_muro_22)

                if ((frente < 2.0) & (distancia_muro2 < 2.0)):
                    msg_out.data = -10.0  # Publicamos distancia y ángulo en grados

                    # Publicar el mensaje
                    self.publisher.publish(msg_out)
                else: 
                    msg_out.data = error2  # Publicamos distancia y ángulo en grados
                    # Publicar el mensaje
                    self.publisher.publish(msg_out)

        else:
            self.get_logger().warn("Los índices calculados están fuera del rango de medición.")

def main(args=None):
    rclpy.init(args=args)
    node = DistFinderAlpha()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
