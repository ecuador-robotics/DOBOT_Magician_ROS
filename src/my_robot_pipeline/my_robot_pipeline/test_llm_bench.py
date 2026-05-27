import ollama
import json
import time

# El mismo prompt del sistema exacto que refinamos para tu nodo
SYSTEM_PROMPT = (
    "Eres un asistente de control para un brazo robótico DOBOT Magician en Gazebo. "
    "Tu única tarea es transformar el comando de voz del usuario en un formato JSON estricto.\n\n"
    "Campos válidos:\n"
    "- 'color': Puede ser 'rojo', 'azul', 'verde' o null si no se menciona un objetivo de color.\n"
    "- 'action': Debe ser EXCLUSIVAMENTE 'open' (para abrir), 'close' (para cerrar) o null. JAMAS uses otra palabra en este campo.\n\n"
    "REGLAS CRUCIALES DE CONTEXTO FONÉTICO:\n"
    "1. Si el texto contiene 'sierra', 'cierra', 'sierra de la paz', 'sierra de la pensa', 'blochero de pesa', o 'pensa', la intención es CERRAR LA PINZA -> {\"color\": null, \"action\": \"close\"}.\n"
    "2. Si el texto contiene 'abre la poza', 'ara pasa', 'adiós pizza', o 'poza', la intención es ABRIR LA PINZA -> {\"color\": null, \"action\": \"open\"}.\n\n"
    "Responde UNICAMENTE con el objeto JSON estructurado, sin texto adicional."
)

# Batería de pruebas: Frases reales extraídas de tus logs de Whisper
FRASES_TEST = [
    "Mueve el brazo a la esfera azul.",
    "Mueve el brazo, esfera roja.",
    "Mueve el brazo a la esfera verde.",
    "Abre la poza.",           # Error acústico de abrir
    "Sierra de la Pensa.",     # Error acústico de cerrar
    "Adiós, pizza!",          # Error acústico de abrir
    "Mueve el brazo.",         # Comando sin color ni acción
    "¡Habla la Polícia!"       # Ruido puro / Frase inválida
]

MODELOS_A_EVALUAR = ["llama3.2", "qwen2.5:3b", "gemma2:2b"]

def evaluar_modelo(nombre_modelo):
    print(f"\n" + "="*50)
    print(f"📊 EVALUANDO MODELO: {nombre_modelo}")
    print("="*50)
    
    tiempos = []
    json_validos = 0
    intenciones_correctas = 0
    
    # Calentamiento del modelo (para evitar sesgo de carga inicial en memoria)
    try:
        ollama.chat(model=nombre_modelo, messages=[{"role": "user", "content": "hola"}])
    except Exception:
        print(f"❌ ¡Error! Asegúrate de haber ejecutado: ollama run {nombre_modelo}")
        return

    for i, frase in enumerate(FRASES_TEST, 1):
        t_inicio = time.perf_counter()
        try:
            respuesta = ollama.chat(
                model=nombre_modelo,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user",   "content": frase}
                ]
            )
            t_fin = time.perf_counter()
            latencia = (t_fin - t_inicio) * 1000 # Convertir a milisegundos
            tiempos.append(latencia)
            
            contenido = respuesta["message"]["content"].strip()
            
            # Validar si es un JSON estructurado correcto
            is_json = False
            try:
                datos = json.loads(contenido)
                is_json = True
                json_validos += 1
            except json.JSONDecodeError:
                datos = {}

            print(f"Frase {i}: '{frase}'")
            print(f" 🧠 Rpta: {contenido.replace('\n', '')} | ⏱️  {latencia:.2f} ms | JSON: {'✅' if is_json else '❌'}")
            print("-" * 40)
            
        except Exception as e:
            print(f"❌ Error en la inferencia: {e}")
            
    # Estadísticas finales del modelo
    latencia_promedio = sum(tiempos) / len(tiempos) if tiempos else 0
    tasa_json = (json_validos / len(FRASES_TEST)) * 100
    
    print(f"\n📈 RESULTADOS PARA {nombre_modelo}:")
    print(f" - Latencia Promedio: {latencia_promedio:.2f} ms")
    print(f" - Tasa de Estructura JSON Válida: {tasa_json:.1f}%")

if __name__ == "__main__":
    for modelo in MODELOS_A_EVALUAR:
        evaluar_modelo(modelo)