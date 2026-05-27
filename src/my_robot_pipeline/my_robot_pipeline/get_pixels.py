import cv2

def hacer_clic(evento, x, y, flags, param):
    if evento == cv2.EVENT_LBUTTONDOWN:
        print(f"[{x}, {y}],")
        cv2.circle(imagen, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow("Captura", imagen)

imagen = cv2.imread("/home/r11/repositorios/DOBOT_Magician_ROS/src/my_robot_pipeline/my_robot_pipeline/img4.png")

# --- ESTAS DOS LÍNEAS SON LA MAGIA ---
cv2.namedWindow("Captura", cv2.WINDOW_NORMAL) # Permite que la ventana sea redimensionable
cv2.resizeWindow("Captura", 800, 600)         # Le da un tamaño inicial pequeño
# ------------------------------------

cv2.imshow("Captura", imagen)
cv2.setMouseCallback("Captura", hacer_clic)

print("Haz clic en los 4 marcadores en este orden: Arriba-Izq, Arriba-Der, Abajo-Der, Abajo-Izq")
print("Presiona la tecla 'ESC' o 'q' para salir.")

cv2.waitKey(0)
cv2.destroyAllWindows()