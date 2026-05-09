# 🤖 CoBot Clasificador de Colores

**Robot babas** — 3er lugar Día de las Ingenierías IBERO 2026 🥉

Sistema de pick and place autónomo con un UR3 de 6 GDL que detecta cubos de colores con una cámara y los clasifica en zonas sin que nadie toque nada.

---

## ¿Qué hace?

Tienes cubos rojos, verdes y azules tirados en una mesa. El robot los detecta con visión artificial, calcula a dónde están, va, los agarra y los lleva a su zona de color. Solo. Sin botones, sin intervención.

Hay dos modos:
- **GO** → el operador elige un cubo específico y el robot lo mueve
- **SORT** → el robot detecta todo lo que está mal colocado y lo ordena solo
- **SORT-BUCLE** → ordena, espera a que alguien desordene de nuevo, y vuelve a ordenar. Indefinidamente.

---

## Stack técnico

- **Robot:** Universal Robots UR3, 6 GDL, comandado por socket TCP directo al puerto 30002 (URScript, sin RoboDK ni nada)
- **Gripper:** OnRobot Soft Gripper, controlado por HTTP REST
- **Cámara:** Logitech C920 en vista cenital (top-down)
- **Visión:** OpenCV — segmentación HSV, morfología, filtros geométricos
- **Calibración:** Marcadores ArUco para homografía cámara→robot (error < 3 mm)
- **Cinemática inversa:** Analítica pura derivada de los parámetros DH del UR3, < 1 ms por llamada, con cascada de 5 fallbacks si falla
- **Trayectorias:** Polinomio quíntico para los tránsitos, movej lento cerca del cubo

---

## Contexto

- **Materia:** Control Avanzado
- **Profesores:** Dr. Andrés Guillermo Molano Jiménez · Dr. Julio Antonio Caballero Mora
- **Carrera:** Ingeniería Mecatrónica, 8° semestre
- **Universidad:** IBERO Ciudad de México · Primavera 2026

---

## Documentación

👉 [Ver sitio de documentación completo](https://elias-papu.github.io/proyecto_robotica_primavera2026)

Incluye derivación de la cinemática inversa analítica, pipeline de visión, comparativa de métodos y código comentado.