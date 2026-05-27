#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
from scipy.io import wavfile
import os
import time  
import re

class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_node")
        
        self.pub = self.create_publisher(String, "/voice_text", 10)
        
        # Guardamos la ruta absoluta en la carpeta /tmp para evitar problemas de permisos
        self.ruta_archivo = "/tmp/dobot_voice_output.wav"
        
        # Parámetros de audio estándar para Whisper
        self.frecuencia = 16000  # 16kHz es el estándar nativo de Whisper
        self.duracion = 4.0      # 4 segundos fijos de grabación
        
        self.get_logger().info("Cargando modelo Whisper...")
        self.model = whisper.load_model("small")
        self.get_logger().info(" VoiceNode listo — habla cuando veas el prompt")

    def grabar_audio(self):
        try:
            # sd.rec graba de forma síncrona/bloqueante si usamos sd.wait()
            self.get_logger().info(f"  Grabando de verdad durante {self.duracion} segundos... ¡HABLA AHORA!")
            audio_data = sd.rec(
                int(self.duracion * self.frecuencia), 
                samplerate=self.frecuencia, 
                channels=1, 
                dtype='int16'
            )
            sd.wait()  # <--- Esto obliga a Python a esperar los 4 segundos reales
            self.get_logger().info(" Grabación de 4 segundos finalizada. Guardando...")

            # =======================================================
            # 1. SOLO ESCRIBIMOS EL ARCHIVO WAV EN EL DISCO
            wavfile.write(self.ruta_archivo, self.frecuencia, audio_data)
            # =======================================================
            
            return True
            
        except Exception as e:
            self.get_logger().error(f" Error físico en el hardware de grabación: {e}")
            return False

    def transcribir(self):
        if not os.path.exists(self.ruta_archivo):
            self.get_logger().error(f"El archivo {self.ruta_archivo} no se encuentra en el disco.")
            return ""
            
        # Opciones estrictas combinadas con tu genial initial_prompt
        opciones_whisper = {
            "language": "es",
            "initial_prompt": "robótica, brazo, esfera, roja, azul, verde, pinza, gripper, abre, cierra, home",
            "condition_on_previous_text": False,
            "no_speech_threshold": 0.6,
            "logprob_threshold": -1.0
        }
        
        # =======================================================
        #INICIO DE MEDICIÓN CIENTÍFICA (LATENCIA)
        inicio_whisper = time.time()
        
        resultado = self.model.transcribe(self.ruta_archivo, **opciones_whisper)
        
        fin_whisper = time.time()
        # FIN DE MEDICIÓN
        
        # Calcular latencia en milisegundos
        latencia_ms = (fin_whisper - inicio_whisper) * 1000
        self.get_logger().info(f"MÉTRICA IEEE - Latencia Whisper: {latencia_ms:.2f} ms")
        # =======================================================

        # Pre-procesamiento de texto (Todo a minúsculas y limpiamos errores acústicos raros)
        texto_crudo = resultado["text"].strip().lower()
        texto_limpio = re.sub(r'(poza|piza|pinsa|pisa|pinca)', 'pinza', texto_crudo)
        texto_limpio = re.sub(r'(habre|abra)', 'abre', texto_limpio)
        
        return texto_limpio

    def escuchar_y_publicar(self):
        while rclpy.ok():
            input("\n⏎  Presiona ENTER para comenzar a grabar...")
            
            # 1. Grabamos bloqueando el hilo de ROS por 4 segundos
            if self.grabar_audio():
                
                # 2. Transcribimos con Whisper y FFmpeg sobre la ruta segura
                try:
                    texto = self.transcribir()
                    self.get_logger().info(f" Transcripción exitosa: '{texto}'")
                    
                    if texto:
                        msg = String()
                        msg.data = texto
                        self.pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f" Error fatal en Whisper / FFmpeg: {e}")

def main():
    rclpy.init()
    node = VoiceNode()
    try:
        node.escuchar_y_publicar()
    except KeyboardInterrupt:
        pass
    finally:
        # Limpieza del archivo temporal al cerrar
        if os.path.exists("/tmp/dobot_voice_output.wav"):
            os.remove("/tmp/dobot_voice_output.wav")
        rclpy.shutdown()

if __name__ == "__main__":
    main()