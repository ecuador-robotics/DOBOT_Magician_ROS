#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import json
import threading
import time
import math

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0]
# Altura fija a la que bajará el end-effector (ajústala según la altura de tu TCP en Gazebo)
Z_FIJO = 0.10

class PipelineNode(Node):
    def __init__(self):
        super().__init__("pipeline_node")

        self.robot = MoveItPy(node_name="pipeline_node")
        self.arm   = self.robot.get_planning_component("arm")

        # 1. Escuchar los comandos del LLM (Voz)
        self.sub_comando = self.create_subscription(
            String, "/robot_command", self.comando_callback, 10
        )

        # 2. Escuchar las coordenadas de la Visión Artificial
        self.sub_vision = self.create_subscription(
            String, "/detected_colors", self.vision_callback, 10
        )

        self.coordenadas_vision = {} # Aquí guardaremos el último JSON de la cámara
        self.ejecutando = False
        
        self.get_logger().info(" PipelineNode listo — esperando voz y visión...")

        # Inicialización forzada por hardware al arrancar:
        self.get_logger().info(" Inicializando controladores. Sincronizando posición Home...")
        self.ir_a_home()

    def vision_callback(self, msg):
        # Actualizamos constantemente dónde están las esferas
        try:
            self.coordenadas_vision = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def esperar(self, segundos=0.5):
        inicio = self.get_clock().now()
        while (self.get_clock().now() - inicio).nanoseconds < segundos * 1e9:
            rclpy.spin_once(self, timeout_sec=0.05)

    def ir_a_home(self):
        self.get_logger().info(" Volviendo a home...")
        arm_state = RobotState(self.robot.get_robot_model())
        arm_state.set_joint_group_positions("arm", np.array(HOME_JOINTS))
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=arm_state)
        plan = self.arm.plan()
        if plan:
            self.robot.execute(plan.trajectory, controllers=[])
            self.esperar(1.0)

    def mover_a_coordenadas(self, x, y, z):
        self.get_logger().info(f" Corrigiendo proyección frontal hacia: X={x}, Y={y}, Z={z}")
        
        # 1. Establecer el estado inicial al actual
        self.arm.set_start_state_to_current_state()
        
        # 2. Calcular el ángulo dinámico hacia el objetivo (Azimut / Yaw)
        theta = math.atan2(float(y), float(x))
        
        # 3. COMPENSACIÓN VECTORIAL DE LA PINZA (6.5 cm del end_effector)
        # Desplazamos el punto de destino para que coincida exactamente con la geometría física
        desfase_pinza = 0.065
        x_corregido = float(x) + (desfase_pinza * math.cos(theta))
        y_corregido = float(y) + (desfase_pinza * math.sin(theta))
        
        self.get_logger().info(f"📍 Coordenadas corregidas por hardware: X={x_corregido:.4f}, Y={y_corregido:.4f}")
        
        # 4. Crear el mensaje PoseStamped nativo para MoveIt
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = x_corregido
        pose_goal.pose.position.y = y_corregido
        pose_goal.pose.position.z = float(z)
        
        # Cuaternión de rotación pura alineado en Z (Yaw dinámico)
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = float(math.sin(theta / 2.0))
        pose_goal.pose.orientation.w = float(math.cos(theta / 2.0))
        
        # 5. Enviar la meta al end_effector
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector")
        
        # 6. Planificar y ejecutar
        # =======================================================
        #  INICIO MÉTRICA MOVEIT (PLANIFICACIÓN CINEMÁTICA)
        inicio_moveit = time.time()
        
        plan = self.arm.plan()
        
        fin_moveit = time.time()
        latencia_moveit_ms = (fin_moveit - inicio_moveit) * 1000
        # =======================================================

        if plan:
            self.get_logger().info(f" MÉTRICA IEEE - Latencia MoveIt: {latencia_moveit_ms:.2f} ms")
            self.get_logger().info(" ¡Trayectoria perfecta encontrada! Ejecutando en Gazebo...")
            self.robot.execute(plan.trajectory, controllers=[])
            self.esperar(1.5)
            return True
        else:
            self.get_logger().error(" Error de IK: El punto corregido está fuera de los límites físicos o colisiona.")
            return False

    def comando_callback(self, msg):
        if self.ejecutando:
            self.get_logger().warn("  Robot ocupado")
            return
        thread = threading.Thread(target=self.ejecutar_comando, args=(msg,))
        thread.start()

    def ejecutar_comando(self, msg):
        try:
            comando = json.loads(msg.data)
            
            # Leemos la lista de colores (si no hay, devuelve lista vacía)
            colores_solicitados = comando.get("colores", [])

            # ========================================================
            # BLOQUEO DE SEGURIDAD NORMAL
            # ========================================================
            if getattr(self, 'ejecutando', False):
                self.get_logger().warn("  Robot ocupado ejecutando otra tarea.")
                return

            if not colores_solicitados:
                self.get_logger().warn(" No se detectaron colores en el comando de voz.")
                return

            self.ejecutando = True

            # ========================================================
            # BUCLE DE SECUENCIA AUTOMÁTICA MÚLTIPLE
            # ========================================================
            for color in colores_solicitados:
                color = color.lower()
                self.get_logger().info(f"🚀 INICIANDO SECUENCIA PARA: {color.upper()}")
                
                if color not in self.coordenadas_vision:
                    self.get_logger().warn(f" El color '{color}' se solicitó, pero la cámara no lo ve. Saltando a la siguiente...")
                    continue # Importante: Usar continue en lugar de return para no matar la lista

                # Extraer coordenadas X, Y de la cámara
                coords = self.coordenadas_vision[color]
                x_obj = coords["x"]
                y_obj = coords["y"]
                
                # --------------------------------------------------------
                # PASO 1: ABRIR LA PINZA PREVIAMENTE (Estando arriba)
                # --------------------------------------------------------
                self.get_logger().info(f" 1/6: Abriendo la pinza previamente al máximo para {color.upper()}...")
                gripper_group = self.robot.get_planning_component("gripper")
                gripper_group.set_start_state_to_current_state()
                gripper_group.set_goal_state(configuration_name="gripper_open")
                plan_open = gripper_group.plan()
                if plan_open:
                    self.robot.execute(plan_open.trajectory, controllers=[])
                self.esperar(0.3)

                # --------------------------------------------------------
                # PASO 2: MOVIMIENTO DIRECTO AL PUNTO DE AGARRE
                # --------------------------------------------------------
                self.get_logger().info(f"✈️  2/6: Moviendo brazo hacia la esfera {color.upper()}...")
                z_agarre = 0.105  # Altura calibrada
                ok_movimiento = self.mover_a_coordenadas(x_obj, y_obj, z_agarre)
                if not ok_movimiento:
                    self.get_logger().error(f" Abortando ciclo para {color.upper()}: IK falló.")
                    continue # Salta a la siguiente esfera de la lista

                # --------------------------------------------------------
                # PASO 3: SUJECIÓN (Cerrar la pinza)
                # --------------------------------------------------------
                self.get_logger().info(" 3/6: Cerrando pinza sobre el objetivo...")
                gripper_group.set_start_state_to_current_state()
                gripper_group.set_goal_state(configuration_name="gripper_closed")
                plan_close = gripper_group.plan()
                if plan_close:
                    self.robot.execute(plan_close.trajectory, controllers=[])
                self.esperar(0.5)

                # --------------------------------------------------------
                # PASO 4: ELEVACIÓN A UNA BUENA ALTURA DE SEGURIDAD
                # --------------------------------------------------------
                z_segura = 0.150  
                self.get_logger().info(f" 4/6: Elevando carga a Z = {z_segura}m...")
                self.mover_a_coordenadas(x_obj, y_obj, z_segura)
                self.esperar(0.2)

                # --------------------------------------------------------
                # PASO 5: TRASLADO Y DESCARGA 
                # --------------------------------------------------------
                x_descarga = 0.22
                y_descarga = -0.15
                z_descarga = 0.115
                
                self.get_logger().info(f" 5/6: Trasladando a zona de descarga: X={x_descarga}, Y={y_descarga}...")
                ok_descarga = self.mover_a_coordenadas(x_descarga, y_descarga, z_descarga)
                
                if ok_descarga:
                    self.get_logger().info(" Soltando la esfera...")
                    gripper_group.set_start_state_to_current_state()
                    gripper_group.set_goal_state(configuration_name="gripper_open")
                    
                    plan_release = gripper_group.plan()
                    if plan_release:
                        self.robot.execute(plan_release.trajectory, controllers=[])
                    
                    self.esperar(0.6)  
                    
                    # Micro-elevación rápida de escape
                    self.mover_a_coordenadas(x_descarga, y_descarga, z_segura)
                else:
                    self.get_logger().error(" Error de traslado a zona de descarga.")

                # --------------------------------------------------------
                # PASO 6: RETORNO SEGURO A HOME
                # --------------------------------------------------------
                self.get_logger().info(" 6/6: Retornando a Home...")
                arm_group = self.robot.get_planning_component("arm")
                arm_group.set_start_state_to_current_state()
                
                self.ir_a_home()
                self.esperar(0.4)
                
                self.get_logger().info(" Asegurando pinza en Home (Cerrando)...")
                gripper_group.set_start_state_to_current_state()
                gripper_group.set_goal_state(configuration_name="gripper_closed")
                plan_home_close = gripper_group.plan()
                if plan_home_close:
                    self.robot.execute(plan_home_close.trajectory, controllers=[])
                
                self.get_logger().info(f" ¡CICLO COMPLETO CON ÉXITO PARA {color.upper()}! ")
                
                # Pausa antes de procesar la siguiente esfera de la lista (si la hay)
                self.esperar(1.5)
            
        except Exception as e:
            self.get_logger().error(f" Error crítico en ejecución: {e}")
        finally:
            # Se libera el robot solo cuando TODA la lista se procesó
            self.ejecutando = False


def main():
    rclpy.init()
    node = PipelineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()