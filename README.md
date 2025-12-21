# Sistema de corte Automatizada de Banano 

![ROS](https://img.shields.io/badge/ROS-Melodic%2FNoetic-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3-3776AB?style=for-the-badge&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![DOBOT](https://img.shields.io/badge/Hardware-DOBOT%20Magician-ff69b4?style=for-the-badge&logo=robot&logoColor=white)

##  Descripción del Proyecto

Este repositorio contiene el desarrollo de un sistema robótico automatizado diseñado para el procesamiento agrícola de precisión. Utilizando un brazo robótico DOBOT Magician, el sistema es capaz de identificar, cortar y separar manos de banano de manera autónoma.

El objetivo principal es realizar la separación del racimo minimizando el daño al producto, utilizando una solución compacta y eficiente.

## Características Principales

* **Visión Artificial:** Implementación de algoritmos de procesamiento de imágenes para localizar espacialmente cada mano de banano y determinar el punto óptimo de corte en el tallo.
* **End-Effector** Desarrollo de una herramienta terminal personalizada que combina mecanismos de corte y sujeción simultáneos. Esto permite asegurar la fruta antes del corte para evitar caídas.
* **Clasificación Automática:** Planificación de trayectorias para depositar cuidadosamente cada mano cortada en una zona de recolección designada.

## Funcionamiento del Sistema

1.  **Detección:** La cámara captura el racimo y el algoritmo calcula las coordenadas de corte.
2.  **Aproximación:** El DOBOT se mueve a la posición objetivo.
3.  **Ejecución:** El end-effector sujeta la mano y activa el mecanismo de corte.
4.  **Recolección:** El robot transporta la mano separada a la bandeja de salida.

