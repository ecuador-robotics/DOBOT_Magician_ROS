#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
import json
import time

SYSTEM_PROMPT = (
    "Eres un asistente cognitivo para un brazo robótico DOBOT Magician en Gazebo. "
    "Tu tarea es analizar comandos de voz en lenguaje natural, entender la intención real "
    "(incluyendo correcciones o secuencias) y generar un JSON estricto.\n\n"
    "Formato requerido:\n"
    "- 'colores': Debe ser una lista (array) con los colores objetivo en orden de ejecución. "
    "Los valores válidos son 'rojo', 'azul', 'verde'. Si no hay color, devuelve [].\n\n"
    "REGLAS CRUCIALES DE RAZONAMIENTO:\n"
    "1. SECUENCIAS: Si el usuario pide múltiples acciones (ej. 'roja luego azul'), agrega ambos a la lista en orden -> {\"colores\": [\"rojo\", \"azul\"]}.\n"
    "2. AUTOCORRECCIONES: Si el usuario se equivoca y cambia de opinión (ej. 've por la roja... no, mejor la verde'), ignora el color cancelado y devuelve solo la intención final -> {\"colores\": [\"verde\"]}.\n\n"
    "Ejemplos:\n"
    "- 'Mueve a la esfera azul y después a la verde' -> {\"colores\": [\"azul\", \"verde\"]}\n"
    "- 'Agarra el rojo, ah no espera, el azul' -> {\"colores\": [\"azul\"]}\n"
    "- 'brazo, esfera, roja' -> {\"colores\": [\"rojo\"]}\n\n"
    "   - Ejemplo: 'Trae la esfera azul... no, mejor la roja' -> {\"colores\": [\"rojo\"]}\n"
    "   - Ejemplo: 'Mueve el brazo al rojo, ah no, al azul... bueno la verdad prefiero el verde' -> {\"colores\": [\"verde\"]}\n"
    "   - Ejemplo: 'Lleva la verde, pero primero recoge la roja' -> {\"colores\": [\"rojo\", \"verde\"]}\n"
    "3. CONDICIONALES IMPLÍCITOS: Si el usuario expresa una preferencia condicional ('si puedes...', 'si está...'), coloca el objeto preferido primero en la lista.\n"
    "   - Ejemplo: 'Trae la azul o si no está la verde' -> {\"colores\": [\"azul\", \"verde\"]}\n\n"
    "   - Ejemplo: 'Lleva la verde, pero primero recoge la roja' -> {\"colores\": [\"rojo\", \"verde\"]}\n"

    "Responde ÚNICAMENTE con el objeto JSON estructurado, sin texto adicional."
)

class LLMNode(Node):
    def __init__(self):
        super().__init__("llm_node")

        self.sub = self.create_subscription(
            String, "/voice_text", self.texto_callback, 10
        )
        self.pub = self.create_publisher(
            String, "/robot_command", 10
        )
        self.get_logger().info(" LLMNode listo — esperando texto...")

    def texto_callback(self, msg):
        texto = msg.data
        self.get_logger().info(f"Recibido: '{texto}'")

        try:
            self.get_logger().info(f"Consultando a Qwen 2.5 con: '{texto}'...")
            
            inicio_llm = time.time()
            
            respuesta = ollama.chat(
                model="qwen2.5:3b",
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user",   "content": texto}
                ]
            )
            
            # ⏱FIN DE MEDICIÓN INMEDIATAMENTE DESPUÉS
            fin_llm = time.time()
            
            latencia_llm_ms = (fin_llm - inicio_llm) * 1000
            self.get_logger().info(f"MÉTRICA IEEE - Latencia LLM: {latencia_llm_ms:.2f} ms")

            contenido = respuesta["message"]["content"].strip()
            
            # =======================================================
            # FILTRO ANTI-MARKDOWN (Para Gemma y Llama 3)
            # Limpiamos los acentos graves y la palabra json si el modelo los añade
            contenido = contenido.replace("```json", "").replace("```", "").strip()
            # =======================================================

            self.get_logger().info(f"LLM responde (Limpio): {contenido}")

            comando = json.loads(contenido)

            # Inyectamos la latencia de Qwen en el JSON antes de enviarlo al pipeline
            comando["latencia_llm"] = round(latencia_llm_ms, 2)
            colores = comando.get("colores", [])

            if colores:
                self.pub.publish(String(data=json.dumps(comando)))

        except json.JSONDecodeError:
            self.get_logger().error(f"JSON inválido: {contenido}")
        except Exception as e:
            self.get_logger().error(f"Error LLM: {e}")

def main():
    rclpy.init()
    node = LLMNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()