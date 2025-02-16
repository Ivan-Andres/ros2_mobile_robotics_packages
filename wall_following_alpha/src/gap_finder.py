#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np

class GapFinder(Node):
    def __init__(self):
        super().__init__('gap_finder')

        # Suscripción al topic del LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.process_scan,
            10
        )

        # Publicar el mensaje con los resultados (distancia y ángulo)
        self.publisher = self.create_publisher(Float32, 'error', 10)

        # Parámetros de configuración
        # Inicializar variables
        self.gap_setpoint = 0.0  # Setpoint basado en el ángulo medio del gap
        self.angle_min = -65.0  # Grados
        self.angle_max = 65.0   # Grados
        self.threshold = 3.0    # Umbral para considerar valores como obstáculos
        self.safety_radius = 0.3

        self.get_logger().info("Nodo GapFinder iniciado correctamente.")

    def process_scan(self, msg: LaserScan):
        self.get_logger().info("Recibidos datos del LIDAR.")

        # Conversión de ángulos de grados a radianes
        angle_min_rad = math.radians(self.angle_min)
        angle_max_rad = math.radians(self.angle_max)

        # Calcular los índices correspondientes a los ángulos deseados
        index_min = int((angle_min_rad - msg.angle_min) / msg.angle_increment)
        index_max = int((angle_max_rad - msg.angle_min) / msg.angle_increment)

        # Limitar el rango de índices dentro de los datos disponibles
        index_min = max(0, index_min)
        index_max = min(len(msg.ranges) - 1, index_max)

        # Filtrar los datos del LIDAR
        filtered_ranges = []
        for i, distance in enumerate(msg.ranges[index_min:index_max + 1]):
            if math.isinf(distance):
                filtered_ranges.append(12.0)  # Conservar los valores infinitos
            elif distance < self.threshold:
                filtered_ranges.append(0.0)  # Valores por debajo del umbral se ponen en 0.0
            else:
                filtered_ranges.append(distance)  # Conservar valores válidos

        # Depuración: Imprimir el rango filtrado
        self.get_logger().info(f"Rangos filtrados: {filtered_ranges}")

        # Paso adicional: Implementación de la burbuja de seguridad
        # Encontrar el índice del valor más cercano, ignorando los valores 0
        # Paso adicional: Implementación de la burbuja de seguridad
        valid_distances = [(i, d) for i, d in enumerate(filtered_ranges) if d > 0]

        if not valid_distances:
            self.get_logger().warn("No se encontraron puntos válidos para la burbuja de seguridad.")
            return

        bubble_radius = self.safety_radius
        angle_increment = msg.angle_increment

        # Identificar los gaps (regiones contiguas de valores > 0)
        gaps = []
        current_gap = []

        for index, distance in valid_distances:
            if not current_gap or index == current_gap[-1][0] + 1:
                current_gap.append((index, distance))
            else:
                gaps.append(current_gap)
                current_gap = [(index, distance)]

        # Agregar el último gap si no está vacío
        if current_gap:
            gaps.append(current_gap)

        # Procesar cada gap y aplicar la burbuja en el valor mínimo de cada uno
        for gap in gaps:
            # Encontrar el valor mínimo en el gap
            nearest_index, nearest_distance = min(gap, key=lambda x: x[1])

            # Calcular el rango angular afectado
            affected_angle = math.atan(bubble_radius / nearest_distance) if nearest_distance > 0 else math.pi
            affected_indices = int(affected_angle / angle_increment)

            # Aplicar la burbuja de seguridad: Establecer en 0 los puntos dentro del rango
            for i in range(max(0, nearest_index - affected_indices), min(len(filtered_ranges), nearest_index + affected_indices + 1)):
                filtered_ranges[i] = 0.0

        # Depuración: Mostrar rangos después de aplicar la burbuja
        self.get_logger().info(f"Rangos filtrados (minimo): {filtered_ranges}")

            # Identificar los gaps (regiones contiguas de valores > 0)
        valid_distances = [(i, d) for i, d in enumerate(filtered_ranges) if d > 0]

        if not valid_distances:
            self.get_logger().warn("No se encontraron puntos válidos para la burbuja de seguridad.")
            return

        bubble_radius = self.safety_radius
        angle_increment = msg.angle_increment

        gaps = []
        current_gap = []

        for index, distance in valid_distances:
            if not current_gap or index == current_gap[-1][0] + 1:
                current_gap.append((index, distance))
            else:
                gaps.append(current_gap)
                current_gap = [(index, distance)]

        # Agregar el último gap si no está vacío
        if current_gap:
            gaps.append(current_gap)

        # Procesar cada gap y aplicar la burbuja en el valor mínimo de cada uno
        for gap in gaps:

            # Aplicar burbujas en los extremos del gap
            # Extremo inferior del gap
            start_index, start_distance = gap[0]
            affected_angle = math.atan(bubble_radius / 2.0) if start_distance > 0 else math.pi
            affected_indices = int(affected_angle / angle_increment)

            for i in range(max(0, start_index - affected_indices), min(len(filtered_ranges), start_index + affected_indices + 1)):
                if filtered_ranges[i] > 0:
                    filtered_ranges[i] = 0.0

            # Extremo superior del gap
            end_index, end_distance = gap[-1]
            affected_angle = math.atan(bubble_radius / 2.0) if end_distance > 0 else math.pi
            affected_indices = int(affected_angle / angle_increment)

            for i in range(max(0, end_index - affected_indices), min(len(filtered_ranges), end_index + affected_indices + 1)):
                if filtered_ranges[i] > 0:
                    filtered_ranges[i] = 0.0

            # Depuración: Mostrar rangos después de aplicar la burbuja
        self.get_logger().info(f"Rangos filtrados (extremos): {filtered_ranges}")

            # Identificar los gaps (regiones contiguas de valores > 0)

        valid_distances = [(i, d) for i, d in enumerate(filtered_ranges) if d > 0]

        if not valid_distances:
            self.get_logger().warn("No se encontraron puntos válidos para la burbuja de seguridad.")
            return

        bubble_radius = self.safety_radius
        angle_increment = msg.angle_increment

        gaps = []
        current_gap = []

        for index, distance in valid_distances:
            if not current_gap or index == current_gap[-1][0] + 1:
                current_gap.append((index, distance))
            else:
                gaps.append(current_gap)
                current_gap = [(index, distance)]

        # Agregar el último gap si no está vacío
        if current_gap:
            gaps.append(current_gap)

        # Procesar cada gap y aplicar la burbuja en el valor mínimo de cada uno
        for gap in gaps:

            # Encontrar el valor más lejano en el gap
            farthest_index, farthest_distance = max(gap, key=lambda x: x[1])

            # Depuración: Mostrar rangos después de aplicar la burbuja
            self.get_logger().info(f"valor mas lejano: {farthest_distance}, indice: {farthest_index}")

            # Calcular el rango angular afectado por la burbuja grande
            # Puedes aumentar el factor de multiplicación para hacer la burbuja más grande
            affected_angle = math.atan(1.5 / farthest_distance) if farthest_distance > 0 else math.pi
            affected_indices = int(affected_angle / angle_increment)

            # Aplicar la burbuja más grande: Establecer en 0 los puntos dentro del rango
            for i in range(max(0, farthest_index - affected_indices), min(len(filtered_ranges), farthest_index + affected_indices + 1)):
                filtered_ranges[i] = 4.0
            

        # Depuración: Mostrar rangos después de aplicar la burbuja
        self.get_logger().info(f"Rangos filtrados (mas lejano): {filtered_ranges}")

        valid_distances = [(i, d) for i, d in enumerate(filtered_ranges) if d > 0]

        if not valid_distances:
            self.get_logger().warn("No se encontraron puntos válidos para la burbuja de seguridad.")
            return

        bubble_radius = self.safety_radius
        angle_increment = msg.angle_increment

        # Encontrar gaps (brechas)
        gaps = []
        current_gap = []
        
        for i, distance in enumerate(filtered_ranges):
            angle = math.degrees(msg.angle_min + msg.angle_increment * (index_min + i))

            if distance == 0.0:
                if current_gap:  # Si estamos en un gap y encontramos un 0, cerramos el gap
                    gaps.append(current_gap)
                    current_gap = None  # Reiniciamos el gap actual
            else:
                if not current_gap:  # Si no estamos en un gap, inicializamos uno nuevo
                    current_gap = {
                        "angles": [angle, angle],  # Guardamos el rango angular inicial y final
                        "distances": [distance],  # Guardamos la primera distancia del gap
                    }
                else:
                    current_gap["angles"][1] = angle  # Extendemos el rango angular
                    current_gap["distances"].append(distance)  # Añadimos la distancia al gap actual

        # Añadir el último gap si quedó uno abierto
        if current_gap:
            gaps.append(current_gap)

        # Depuración: Mostrar los gaps encontrados
        self.get_logger().info(f"Gaps encontrados: {gaps}")

        # Encontrar el gap más ancho
        max_gap = None
        max_gap_width = 0
        for gap in gaps:
            # Calcular el ancho del gap en grados
            gap_width = gap["angles"][1] - gap["angles"][0]
            
            # Verificar si este gap es el más ancho
            if gap_width > max_gap_width:
                max_gap = gap
                max_gap_width = gap_width


        # Depuración: Mostrar el gap más ancho encontrado
        if max_gap is not None:
            self.get_logger().info(f"Gap más ancho: {max_gap['angles']} con ancho {max_gap_width:.2f} grados.")
            # Mostrar la distancia máxima dentro del gap
            max_distance = max(max_gap["distances"])  # Distancia máxima dentro del gap
            self.get_logger().info(f"Distancia máxima dentro del gap más ancho: {max_distance:.2f} unidades.")
        else:
            self.get_logger().info("No se encontraron gaps.")
    
        if max_gap is not None:
            # Convertir los valores de max_gap de grados a radianes
            gap_start_rad = math.radians(max_gap["angles"][0])  # Convertir ángulo inicial del gap a radianes

            # Calcular todos los ángulos en grados según el tamaño del vector de distancias y el incremento
            angles = [math.degrees(gap_start_rad + msg.angle_increment * i) for i in range(len(max_gap["distances"]))]

            # Depuración: Mostrar los ángulos
            self.get_logger().info(f"Ángulos calculados: {angles}")

            # Encontrar el índice de la máxima distancia en el vector de distancias
            max_distance_index = max_gap["distances"].index(max(max_gap["distances"]))

            # Obtener el ángulo correspondiente al índice de la máxima distancia
            max_distance_angle = angles[max_distance_index]  # Ángulo correspondiente a la distancia máxima

            # Depuración: Mostrar el ángulo correspondiente
            self.get_logger().info(f"Ángulo correspondiente a la distancia máxima: {max_distance_angle:.2f} grados")

            self.gap_setpoint = math.radians(max_distance_angle)


        self.get_logger().info(f"Setpoint calculado: {self.gap_setpoint:.2f}°")

        # Crear el mensaje para publicar (usamos Float32)
        msg_out = Float32()

        msg_out.data = self.gap_setpoint  # Publicamos distancia y ángulo en grados

        # Publicar el mensaje
        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = GapFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
