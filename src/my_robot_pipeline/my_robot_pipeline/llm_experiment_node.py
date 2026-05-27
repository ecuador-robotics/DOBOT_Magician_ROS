#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
import json
import time
import csv
import os

# ======================================================================
# DATASET DE VALIDACIÓN CIENTÍFICA (150 PRUEBAS)
# ======================================================================
DATASET = [
    # --- NIVEL BÁSICO (1-50): Órdenes directas y falsos positivos ---
    {"f": "Mueve la esfera roja", "j": ["rojo"]},
    {"f": "Agarra el cubo rojo", "j": ["rojo"]},
    {"f": "Lleva la pieza roja al centro", "j": ["rojo"]},
    {"f": "Robot, busca el color rojo", "j": ["rojo"]},
    {"f": "Recoge el objeto rojo", "j": ["rojo"]},
    {"f": "Por favor, mueve la roja", "j": ["rojo"]},
    {"f": "Quiero que agarres la esfera roja", "j": ["rojo"]},
    {"f": "Detecta y mueve el rojo", "j": ["rojo"]},
    {"f": "Pinza, toma la roja", "j": ["rojo"]},
    {"f": "Inicia secuencia con la roja", "j": ["rojo"]},
    
    {"f": "Mueve la esfera azul", "j": ["azul"]},
    {"f": "Agarra el cubo azul", "j": ["azul"]},
    {"f": "Lleva la pieza azul al centro", "j": ["azul"]},
    {"f": "Robot, busca el color azul", "j": ["azul"]},
    {"f": "Recoge el objeto azul", "j": ["azul"]},
    {"f": "Por favor, mueve la azul", "j": ["azul"]},
    {"f": "Quiero que agarres la esfera azul", "j": ["azul"]},
    {"f": "Detecta y mueve el azul", "j": ["azul"]},
    {"f": "Pinza, toma la azul", "j": ["azul"]},
    {"f": "Inicia secuencia con la azul", "j": ["azul"]},

    {"f": "Mueve la esfera verde", "j": ["verde"]},
    {"f": "Agarra el cubo verde", "j": ["verde"]},
    {"f": "Lleva la pieza verde al centro", "j": ["verde"]},
    {"f": "Robot, busca el color verde", "j": ["verde"]},
    {"f": "Recoge el objeto verde", "j": ["verde"]},
    {"f": "Por favor, mueve la verde", "j": ["verde"]},
    {"f": "Quiero que agarres la esfera verde", "j": ["verde"]},
    {"f": "Detecta y mueve el verde", "j": ["verde"]},
    {"f": "Pinza, toma la verde", "j": ["verde"]},
    {"f": "Inicia secuencia con la verde", "j": ["verde"]},

    # Pruebas de rechazo (El LLM debe devolver vacío [])
    {"f": "Abre la pinza", "j": []},
    {"f": "Cierra el gripper", "j": []},
    {"f": "Vuelve a la posición home", "j": []},
    {"f": "Detente inmediatamente", "j": []},
    {"f": "Apaga los motores", "j": []},
    {"f": "Mueve la esfera amarilla", "j": []}, # Amarillo no es válido
    {"f": "Recoge el cubo naranja", "j": []},
    {"f": "Busca la pieza morada", "j": []},
    {"f": "Mueve la esfera negra", "j": []},
    {"f": "Lleva la blanca al centro", "j": []},
    {"f": "Hola brazo robótico, ¿cómo estás?", "j": []},
    {"f": "Esto es una prueba de sonido", "j": []},
    {"f": "Uno dos tres probando", "j": []},
    {"f": "No hagas nada", "j": []},
    {"f": "Ignora este comando", "j": []},
    {"f": "Gira a la derecha", "j": []},
    {"f": "Baja en el eje z", "j": []},
    {"f": "Sube cinco centímetros", "j": []},
    {"f": "Activa la cámara", "j": []},
    {"f": "Calibra los sensores", "j": []},

    # --- NIVEL MEDIO (51-100): Secuencias y listas ---
    {"f": "Toma la roja y luego la azul", "j": ["rojo", "azul"]},
    {"f": "Primero la roja, después la azul", "j": ["rojo", "azul"]},
    {"f": "Mueve la roja y a continuación la azul", "j": ["rojo", "azul"]},
    {"f": "Quiero la roja y en segundo lugar la azul", "j": ["rojo", "azul"]},
    {"f": "Agarra la roja, seguida de la azul", "j": ["rojo", "azul"]},
    {"f": "Mueve la roja y la azul", "j": ["rojo", "azul"]},
    {"f": "Lleva la pieza roja y también la azul", "j": ["rojo", "azul"]},
    {"f": "Recolecta la roja, posteriormente la azul", "j": ["rojo", "azul"]},
    {"f": "Empieza por la roja, termina con la azul", "j": ["rojo", "azul"]},
    {"f": "Pásame la roja y la azul", "j": ["rojo", "azul"]},

    {"f": "Toma la azul y luego la verde", "j": ["azul", "verde"]},
    {"f": "Primero la azul, después la verde", "j": ["azul", "verde"]},
    {"f": "Mueve la azul y a continuación la verde", "j": ["azul", "verde"]},
    {"f": "Quiero la azul y en segundo lugar la verde", "j": ["azul", "verde"]},
    {"f": "Agarra la azul, seguida de la verde", "j": ["azul", "verde"]},
    {"f": "Mueve la azul y la verde", "j": ["azul", "verde"]},
    {"f": "Lleva la pieza azul y también la verde", "j": ["azul", "verde"]},
    {"f": "Recolecta la azul, posteriormente la verde", "j": ["azul", "verde"]},
    {"f": "Empieza por la azul, termina con la verde", "j": ["azul", "verde"]},
    {"f": "Pásame la azul y la verde", "j": ["azul", "verde"]},

    {"f": "Toma la verde y luego la roja", "j": ["verde", "rojo"]},
    {"f": "Primero la verde, después la roja", "j": ["verde", "rojo"]},
    {"f": "Mueve la verde y a continuación la roja", "j": ["verde", "rojo"]},
    {"f": "Quiero la verde y en segundo lugar la roja", "j": ["verde", "rojo"]},
    {"f": "Agarra la verde, seguida de la roja", "j": ["verde", "rojo"]},
    {"f": "Mueve la verde y la roja", "j": ["verde", "rojo"]},
    {"f": "Lleva la pieza verde y también la roja", "j": ["verde", "rojo"]},
    {"f": "Recolecta la verde, posteriormente la roja", "j": ["verde", "rojo"]},
    {"f": "Empieza por la verde, termina con la roja", "j": ["verde", "rojo"]},
    {"f": "Pásame la verde y la roja", "j": ["verde", "rojo"]},

    {"f": "Mueve las tres, roja, azul y verde", "j": ["rojo", "azul", "verde"]},
    {"f": "Ordena roja azul y verde", "j": ["rojo", "azul", "verde"]},
    {"f": "Recoge azul, roja y verde en ese orden", "j": ["azul", "rojo", "verde"]},
    {"f": "Tráeme la verde, la azul y al final la roja", "j": ["verde", "azul", "rojo"]},
    {"f": "Limpia la mesa empezando por la roja, luego verde y azul", "j": ["rojo", "verde", "azul"]},
    {"f": "Mueve la azul, la roja y la verde", "j": ["azul", "rojo", "verde"]},
    {"f": "Primero verde, segundo rojo, tercero azul", "j": ["verde", "rojo", "azul"]},
    {"f": "Agarra rojo, verde y por último azul", "j": ["rojo", "verde", "azul"]},
    {"f": "Azul verde y rojo", "j": ["azul", "verde", "rojo"]},
    {"f": "Toma todas: verde, azul y roja", "j": ["verde", "azul", "rojo"]},
    
    {"f": "Haz una torre con la roja y la roja", "j": ["rojo", "rojo"]},
    {"f": "Mueve dos azules", "j": ["azul", "azul"]},
    {"f": "Pásame la verde y otra verde", "j": ["verde", "verde"]},
    {"f": "Quiero la roja y la amarilla", "j": ["rojo"]}, # Filtra el inválido
    {"f": "Primero la morada y luego la azul", "j": ["azul"]},
    {"f": "Mueve la blanca, la negra y la roja", "j": ["rojo"]},
    {"f": "Verde y naranja", "j": ["verde"]},
    {"f": "Azul, amarillo, verde", "j": ["azul", "verde"]},
    {"f": "Rojo, turquesa, azul", "j": ["rojo", "azul"]},
    {"f": "Verde, rojo y fucsia", "j": ["verde", "rojo"]},

    # --- NIVEL DIFÍCIL (101-150): Autocorrecciones, dudas y condicionales ---
    {"f": "Mueve la roja... ah no, la azul", "j": ["azul"]},
    {"f": "Agarra la azul, espera, me equivoqué, la verde", "j": ["verde"]},
    {"f": "Lleva la verde... no, mejor la roja", "j": ["rojo"]},
    {"f": "Creo que quiero la roja, no, sí, la azul", "j": ["azul"]},
    {"f": "Mueve la azul, corrección, la verde", "j": ["verde"]},
    {"f": "Quiero la verde, olvídalo, toma la roja", "j": ["rojo"]},
    {"f": "Pásame la roja, mmm no, la azul está mejor", "j": ["azul"]},
    {"f": "Agarra la azul, cancela eso, ve por la verde", "j": ["verde"]},
    {"f": "La verde, no mentira, la roja", "j": ["rojo"]},
    {"f": "Ve a la roja, no, azul", "j": ["azul"]},

    {"f": "Quiero la roja y la verde... ah no, la azul en vez de la verde", "j": ["rojo", "azul"]},
    {"f": "Toma la azul y la roja, espera, solo la azul", "j": ["azul"]},
    {"f": "Primero la verde luego la azul, no, al revés", "j": ["azul", "verde"]},
    {"f": "Lleva la roja, azul y verde, no, deja la verde", "j": ["rojo", "azul"]},
    {"f": "Mueve la roja, mmm, y también la verde", "j": ["rojo", "verde"]},
    {"f": "Agarra la azul, y sabes qué, la roja también", "j": ["azul", "rojo"]},
    {"f": "La verde, y después... mmm, la azul", "j": ["verde", "azul"]},
    {"f": "Roja, no, verde, y luego azul", "j": ["verde", "azul"]},
    {"f": "Azul y roja, ah, cambia la roja por la verde", "j": ["azul", "verde"]},
    {"f": "Llévate todas, no, solo la roja", "j": ["rojo"]},

    {"f": "Si está la roja agárrala, si no la verde", "j": ["rojo", "verde"]},
    {"f": "Intenta con la azul, y si fallas la roja", "j": ["azul", "rojo"]},
    {"f": "Tráeme la verde preferiblemente, o la azul", "j": ["verde", "azul"]},
    {"f": "Busca la roja, en su defecto la verde", "j": ["rojo", "verde"]},
    {"f": "Mueve la azul, a menos que esté la roja", "j": ["azul", "rojo"]},
    {"f": "Si ves la verde tómala, y también la azul", "j": ["verde", "azul"]},
    {"f": "Primero intenta con la roja, luego vemos la verde", "j": ["rojo", "verde"]},
    {"f": "Si tienes tiempo mueve la azul y la roja", "j": ["azul", "rojo"]},
    {"f": "Dame la verde si puedes, sino la roja", "j": ["verde", "rojo"]},
    {"f": "Como primera opción la azul, segunda opción verde", "j": ["azul", "verde"]},

    {"f": "Oye robot, este, quiero que muevas la, ah, la verde", "j": ["verde"]},
    {"f": "Mira, necesito que vayas por la esfera... azul", "j": ["azul"]},
    {"f": "Vamos a ver, agarra la roja por favor", "j": ["rojo"]},
    {"f": "Mmm, ¿puedes mover la verde al centro?", "j": ["verde"]},
    {"f": "Fíjate si puedes agarrar la azul, gracias", "j": ["azul"]},
    {"f": "No sé, creo que la roja es la que quiero", "j": ["rojo"]},
    {"f": "Este, brazo robótico, toma la, mmm, verde", "j": ["verde"]},
    {"f": "Hola, mueve la azul cuando estés listo", "j": ["azul"]},
    {"f": "Por favor, si no es molestia, la roja", "j": ["rojo"]},
    {"f": "A ver qué pasa si te pido la verde", "j": ["verde"]},

    {"f": "Esa esfera de color, ay cómo se dice, rojo, sí, esa", "j": ["rojo"]},
    {"f": "Quiero el color del cielo, digo, azul", "j": ["azul"]},
    {"f": "Agarra la que es color pasto, verde", "j": ["verde"]},
    {"f": "Lleva la roja, pero despacito por favor", "j": ["rojo"]},
    {"f": "Mueve la azul rápido, no, mentira, tomate tu tiempo", "j": ["azul"]},
    {"f": "Robot, verde, rojo, azul... nah, solo verde", "j": ["verde"]},
    {"f": "La azulita, esa misma, agárrala", "j": ["azul"]},
    {"f": "El rojito que está ahí", "j": ["rojo"]},
    {"f": "Ve por la verde, que es mi favorita", "j": ["verde"]},
    {"f": "Última prueba, mueve la roja y ya terminamos", "j": ["rojo"]}
]

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

class LLMExperimentNode(Node):
    def __init__(self):
        super().__init__("llm_experiment_node")
        
        self.sub = self.create_subscription(String, "/voice_text", self.texto_callback, 10)
        self.csv_file = os.path.expanduser("~/resultados_paper_llms_final.csv")
        
        self.iteracion = 0
        self.max_iteraciones = len(DATASET)
        
        # Memoria RAM temporal para guardar lo que hablas antes de ejecutar los LLMs
        self.datos_capturados = []

        self.get_logger().info(f"\n🚀 Nodo Experimental Listo. FASE 1: Recolección de Voz")
        self.mostrar_siguiente_frase()

    def mostrar_siguiente_frase(self):
        if self.iteracion < self.max_iteraciones:
            datos_actuales = DATASET[self.iteracion]
            frase_a_leer = datos_actuales["f"]
            nivel = "Básico" if self.iteracion < 50 else ("Medio" if self.iteracion < 100 else "Difícil")
                
            print("\n" + "="*70)
            print(f" PRUEBA {self.iteracion + 1} / {self.max_iteraciones} | Nivel: {nivel}")
            print("="*70)
            print(f"\n VE A LA OTRA TERMINAL, PRESIONA ENTER Y LEE ESTO EN VOZ ALTA:\n")
            print(f"    ▶▶  \"{frase_a_leer}\"  ◀◀\n")
            print("="*70)

    def texto_callback(self, msg):
        texto_transcrito = msg.data
        datos_actuales = DATASET[self.iteracion]
        
        print(f"\n Whisper capturó: '{texto_transcrito}'")
        lat_whisper_str = input("⏱ Ingresa la latencia de Whisper de la otra terminal (ms): ")
        try:
            lat_whisper = float(lat_whisper_str)
        except ValueError:
            lat_whisper = 0.0

        # Guardamos en la memoria RAM temporal, NO ejecutamos LLMs todavía
        self.datos_capturados.append({
            "iteracion": self.iteracion + 1,
            "nivel": "Básico" if self.iteracion < 50 else ("Medio" if self.iteracion < 100 else "Difícil"),
            "frase_original": datos_actuales["f"],
            "texto_transcrito": texto_transcrito,
            "json_esperado": json.dumps({"colores": datos_actuales["j"]}),
            "lat_whisper": lat_whisper
        })
        
        print(f"Voz registrada ({self.iteracion + 1}/150). Pasando a la siguiente...")
        self.iteracion += 1
        
        if self.iteracion >= self.max_iteraciones:
            self.ejecutar_fase_llms()
        else:
            self.mostrar_siguiente_frase()

    def consultar_llm(self, modelo, texto):
        inicio = time.time()
        try:
            respuesta = ollama.chat(
                model=modelo,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": texto}
                ]
            )
            fin = time.time()
            latencia_ms = (fin - inicio) * 1000
            
            contenido = respuesta["message"]["content"].strip()
            contenido = contenido.replace("```json", "").replace("```", "").strip()
            return contenido, latencia_ms
        except Exception as e:
            self.get_logger().error(f"Error consultando {modelo}: {e}")
            return "Error", 0.0

    def ejecutar_fase_llms(self):
        print("\n" + "*"*70)
        print("FASE 1 COMPLETADA. INICIANDO FASE 2 (INFERENCIA MASIVA) ")
        print("Esto tomará un par de minutos. Puedes ir a tomar un vaso de agua.")
        print("*"*70)

        latencias_qwen = []
        latencias_llama = []
        latencias_gemma = []

        # ==========================================
        # EJECUCIÓN EN BLOQUE PARA EVITAR VRAM SWAP
        # ==========================================
        
        print("\n 1/3: Evaluando Qwen 2.5 en las 150 pruebas...")
        self.consultar_llm("qwen2.5:3b", "calentamiento") # Disparo ciego para absorber el Cold Start
        resultados_qwen = []
        for dato in self.datos_capturados:
            json_res, lat = self.consultar_llm("qwen2.5:3b", dato["texto_transcrito"])
            resultados_qwen.append((json_res, lat))
            latencias_qwen.append(lat)
            print(f"   - Prueba {dato['iteracion']} Qwen: {lat:.2f} ms")

        print("\n 2/3: Evaluando Llama 3.2 en las 150 pruebas...")
        self.consultar_llm("llama3.2", "calentamiento") # Absorbe Cold Start
        resultados_llama = []
        for dato in self.datos_capturados:
            json_res, lat = self.consultar_llm("llama3.2", dato["texto_transcrito"])
            resultados_llama.append((json_res, lat))
            latencias_llama.append(lat)
            print(f"   - Prueba {dato['iteracion']} Llama: {lat:.2f} ms")

        print("\n 3/3: Evaluando Gemma 2 en las 150 pruebas...")
        self.consultar_llm("gemma2:2b", "calentamiento") # Absorbe Cold Start
        resultados_gemma = []
        for dato in self.datos_capturados:
            json_res, lat = self.consultar_llm("gemma2:2b", dato["texto_transcrito"])
            resultados_gemma.append((json_res, lat))
            latencias_gemma.append(lat)
            print(f"   - Prueba {dato['iteracion']} Gemma: {lat:.2f} ms")

        print("\n💾 Guardando resultados masivos en el CSV...")
        with open(self.csv_file, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                "Iteracion", "Nivel Dificultad", "Frase Solicitada", "Frase Transcrita (Whisper)", 
                "JSON Ground-Truth", 
                "JSON Qwen", "Latencia Qwen (ms)",
                "JSON Llama", "Latencia Llama (ms)",
                "JSON Gemma", "Latencia Gemma (ms)",
                "Latencia Whisper (ms)"
            ])
            for i in range(self.max_iteraciones):
                dato = self.datos_capturados[i]
                writer.writerow([
                    dato["iteracion"], dato["nivel"], dato["frase_original"], dato["texto_transcrito"], 
                    dato["json_esperado"],
                    resultados_qwen[i][0], round(resultados_qwen[i][1], 2),
                    resultados_llama[i][0], round(resultados_llama[i][1], 2),
                    resultados_gemma[i][0], round(resultados_gemma[i][1], 2),
                    round(dato["lat_whisper"], 2)
                ])

        print("\n" + "*"*70)
        print(" EXPERIMENTO 100% FINALIZADO ")
        print(f" PROMEDIOS REALES (Latencia Pura sin Cold Start):")
        
        prom_qwen = sum(latencias_qwen) / len(latencias_qwen)
        prom_llama = sum(latencias_llama) / len(latencias_llama)
        prom_gemma = sum(latencias_gemma) / len(latencias_gemma)
        prom_whisper = sum([d["lat_whisper"] for d in self.datos_capturados]) / len(self.datos_capturados)

        print(f" Whisper: {prom_whisper:.2f} ms")
        print(f" Qwen 2.5: {prom_qwen:.2f} ms")
        print(f" Llama 3.2: {prom_llama:.2f} ms")
        print(f" Gemma 2: {prom_gemma:.2f} ms")
        print(f"\n Archivo maestro guardado en: {self.csv_file}")
        print("Presiona Ctrl+C para salir.")

def main():
    rclpy.init()
    node = LLMExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nRecolección interrumpida manualmente.")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()