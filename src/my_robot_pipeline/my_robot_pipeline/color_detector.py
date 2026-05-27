#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time

# Rangos HSV para cada color
COLOR_RANGES = {
    "rojo":  [(0,   120, 70),  (10,  255, 255)],
    "azul":  [(100, 100, 50),  (130, 255, 255)],
    "verde": [(40,  50,  50),  (80,  255, 255)],
}

class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.bridge = CvBridge()

        # ========================================================
        # CÁLCULO DINÁMICO DE LA HOMOGRAFÍA
        # ========================================================
        # 1. Los píxeles que extrajiste de la cámara (script original)
        puntos_imagen = np.array([
            [1149, 1157],    # Arriba Izquierda
            [910, 766],   # Arriba Derecha
            [1252, 685],  # Abajo Derecha
            [1581, 1008]   # Abajo Izquierda
        ], dtype="float32")

        # 2. Las coordenadas reales en metros (Gazebo)
        puntos_mundo = np.array([
            [0.35,  0.15], # Arriba Izquierda
            [0.35, -0.15], # Arriba Derecha
            [0.15, -0.15], # Abajo Derecha
            [0.15,  0.15]  # Abajo Izquierda
        ], dtype="float32")

        # 3. Generar la matriz H
        self.H, _ = cv2.findHomography(puntos_imagen, puntos_mundo)
        self.get_logger().info(" Matriz de Homografía calculada e integrada")
        # ========================================================

        # Suscribirse a la cámara
        self.sub = self.create_subscription(
            Image,
            "/rgb_camera/image_raw",
            self.imagen_callback,
            10
        )

        # Publicar colores detectados y sus coordenadas (X, Y)
        self.pub = self.create_publisher(String, "/detected_colors", 10)

        self.get_logger().info(" ColorDetector listo")

    def detectar_color(self, imagen, color):
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        bajo, alto = COLOR_RANGES[color]
        mask = cv2.inRange(hsv, np.array(bajo), np.array(alto))

        # Filtrar ruido
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contornos, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for c in contornos:
            if cv2.contourArea(c) > 100:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # ------------------------------------------------
                    # TRANSFORMACIÓN DE PÍXEL A METROS
                    # ------------------------------------------------
                    punto_pixel = np.array([[[cx, cy]]], dtype="float32")
                    punto_metros = cv2.perspectiveTransform(punto_pixel, self.H)
                    
                    x_real = round(float(punto_metros[0][0][0]), 4)
                    y_real = round(float(punto_metros[0][0][1]), 4)
                    
                    # Devolvemos un diccionario con las coordenadas
                    return {"x": x_real, "y": y_real}
        return None

    def imagen_callback(self, msg):
        inicio_vision = time.time()

        imagen = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        diccionario_colores = {}
        for color in COLOR_RANGES:
            coords = self.detectar_color(imagen, color)
            if coords:
                diccionario_colores[color] = coords

        fin_vision = time.time()
        latencia_vision_ms = (fin_vision - inicio_vision) * 1000

        if diccionario_colores:
            # Inyectamos la latencia del cuadro dentro del mismo JSON
            diccionario_colores["latencia_ms"] = latencia_vision_ms
            
            resultado = json.dumps(diccionario_colores)
            self.pub.publish(String(data=resultado))

def main():
    rclpy.init()
    node = ColorDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()