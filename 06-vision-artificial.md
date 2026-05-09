---
layout: default
title: Visión Artificial
nav_order: 7
description: "Pipeline HSV, calibración ArUco, homografía y corrección de distorsión"
permalink: /06-vision-artificial/
---

# 6. Visión Artificial
{: .no_toc }

Pipeline completo de detección: segmentación HSV, filtros geométricos, homografía ArUco y corrección de distorsión.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 6.1 Pipeline de Detección

El pipeline transforma un frame de cámara en coordenadas cartesianas del robot en 8 etapas:

```
Frame (1280×720)
      │
      ▼ 1. Conversión de color
  HSV Image
      │
      ▼ 2. Segmentación por color
  Máscara binaria
      │
      ▼ 3. Morfología (open + close)
  Máscara limpia
      │
      ▼ 4. Detección de contornos
  Lista de contornos
      │
      ▼ 5. Filtros geométricos
  Contornos válidos (cubos)
      │
      ▼ 6. Cálculo de centroide
  (px, py) en píxeles
      │
      ▼ 7. Corrección de distorsión radial
  (px_corr, py_corr)
      │
      ▼ 8. Transformación de homografía
  (X, Y) en mm (coords robot)
```

---

## 6.2 Rangos HSV Calibrados

Los rangos HSV fueron calibrados experimentalmente bajo las condiciones de iluminación del laboratorio:

### Rojo

El rojo ocupa dos rangos en el espacio HSV (extremos del espectro):

| Rango | H_min | S_min | V_min | H_max | S_max | V_max |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| Rojo inferior | 0 | 130 | 70 | 10 | 255 | 255 |
| Rojo superior | 170 | 130 | 70 | 180 | 255 | 255 |

### Verde Lima

| H_min | S_min | V_min | H_max | S_max | V_max |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 45 | 100 | 60 | 80 | 255 | 220 |

### Azul Turquesa

| H_min | S_min | V_min | H_max | S_max | V_max |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 85 | 130 | 80 | 105 | 255 | 255 |

> **Nota:** Los rangos de saturación mínima ≥ 80-130 descartan sombras y superficies blancas. El valor mínimo ≥ 60-70 elimina regiones oscuras. Esto hace el pipeline robusto ante variaciones moderadas de iluminación ambiental.

---

## 6.3 Filtros Geométricos

Después de detectar contornos, se aplican 4 filtros geométricos para eliminar falsos positivos:

| Filtro | Condición | Justificación |
|---|---|---|
| **Área** | 1000 ≤ A ≤ 15000 px² | Elimina ruido (pequeño) y regiones grandes (fondo) |
| **Solidez** | S > 0.85 | $$S = A / A_{hull}$$, garantiza forma compacta (cubo, no L o C) |
| **Aspecto** | 0.75 ≤ W/H ≤ 1.30 | El cubo proyectado desde arriba es aproximadamente cuadrado |
| **Polígono** | 4 ≤ vértices ≤ 6 | `approxPolyDP` confirma forma poligonal cuadrada |

```python
def filtros_geometricos(contorno):
    """
    Aplica los 4 filtros geométricos al contorno.
    Retorna True si el contorno corresponde a un cubo válido.
    """
    area = cv2.contourArea(contorno)
    if not (1000 <= area <= 15000):
        return False

    # Solidez
    hull = cv2.convexHull(contorno)
    area_hull = cv2.contourArea(hull)
    solidez = area / area_hull if area_hull > 0 else 0
    if solidez < 0.85:
        return False

    # Aspecto (relación ancho/alto)
    _, _, w, h = cv2.boundingRect(contorno)
    aspecto = w / h if h > 0 else 0
    if not (0.75 <= aspecto <= 1.30):
        return False

    # Número de vértices
    epsilon = 0.04 * cv2.arcLength(contorno, True)
    approx = cv2.approxPolyDP(contorno, epsilon, True)
    if not (4 <= len(approx) <= 6):
        return False

    return True
```

---

## 6.4 Calibración de Homografía con ArUco

### Marcadores ArUco

Se colocan **4 marcadores ArUco** en las esquinas del área de trabajo. Sus coordenadas reales en el sistema del robot son:

| ID Marcador | X robot (mm) | Y robot (mm) | Posición |
|:---:|:---:|:---:|---|
| **ID 0** | 460 | -510 | Esquina inferior derecha |
| **ID 1** | -460 | -510 | Esquina inferior izquierda |
| **ID 2** | 460 | -90 | Esquina superior derecha |
| **ID 3** | -460 | -90 | Esquina superior izquierda |

### Cálculo de la Homografía

```python
import cv2
import numpy as np
import json

# Coordenadas reales de los marcadores (mm en sistema robot)
ARUCO_COORDS_ROBOT = {
    0: ( 460, -510),
    1: (-460, -510),
    2: ( 460,  -90),
    3: (-460,  -90),
}

def calibrar_homografia(frame, cache_path="homografia.json"):
    """
    Detecta los 4 marcadores ArUco y calcula la matriz de homografía.
    Guarda el resultado en caché JSON para reusar sin recalibrar.
    """
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None or len(ids) < 4:
        raise RuntimeError("No se detectaron los 4 marcadores ArUco.")

    puntos_pixel  = []
    puntos_robot  = []

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in ARUCO_COORDS_ROBOT:
            # Centroide del marcador en píxeles
            cx = corners[i][0][:, 0].mean()
            cy = corners[i][0][:, 1].mean()
            puntos_pixel.append([cx, cy])
            puntos_robot.append(ARUCO_COORDS_ROBOT[marker_id])

    if len(puntos_pixel) < 4:
        raise RuntimeError("No se encontraron suficientes marcadores conocidos.")

    src = np.array(puntos_pixel,  dtype=np.float32)
    dst = np.array(puntos_robot,  dtype=np.float32)

    # Calcular homografía con RANSAC para robustez ante outliers
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransacReprojThreshold=5.0)

    # Guardar en caché
    with open(cache_path, "w") as f:
        json.dump({"H": H.tolist()}, f)

    print(f"✓ Homografía calculada. Inliers: {mask.sum()}/4")
    return H
```

---

## 6.5 Corrección de Distorsión Radial (Barrel)

La Logitech C920 introduce una leve distorsión radial de tipo *barrel* (líneas rectas aparecen curvas hacia afuera). Se corrige con el modelo:

$$
r_{corr} = r \cdot (1 + K \cdot r^2)
$$

con $$K = -2 \times 10^{-5}$$ (determinado por calibración con tablero de ajedrez).

```python
def corregir_distorsion(px, py, cx=640, cy=360, K=-2e-5):
    """
    Corrige la distorsión radial barrel de la cámara.

    Args:
        px, py:   coordenadas del punto en píxeles
        cx, cy:   centro óptico de la cámara (principal point)
        K:        coeficiente de distorsión radial (K < 0: barrel)

    Returns:
        px_corr, py_corr: coordenadas corregidas
    """
    # Normalizar respecto al centro óptico
    dx = px - cx
    dy = py - cy
    r2 = dx**2 + dy**2

    # Factor de corrección
    factor = 1 + K * r2

    # Coordenadas corregidas
    px_corr = cx + dx * factor
    py_corr = cy + dy * factor

    return px_corr, py_corr
```

---

## 6.6 Función `_detectar_cubos_en_frame`

```python
def _detectar_cubos_en_frame(self, frame):
    """
    Detecta todos los cubos de colores en un frame.

    Args:
        frame: imagen BGR de OpenCV (1280×720)

    Returns:
        lista de dicts: [{color, px, py, area}, ...]
    """
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cubos = []

    for color, rangos in self.RANGOS_HSV.items():
        # Crear máscara (puede ser unión de dos rangos para rojo)
        mascara = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lo, hi) in rangos:
            mascara |= cv2.inRange(hsv, lo, hi)

        # Morfología: eliminar ruido y rellenar huecos
        kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN,  kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

        # Detectar contornos
        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contornos:
            if not filtros_geometricos(cnt):
                continue

            # Centroide
            M  = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            px = M["m10"] / M["m00"]
            py = M["m01"] / M["m00"]

            cubos.append({
                "color": color,
                "px":    px,
                "py":    py,
                "area":  cv2.contourArea(cnt),
            })

    return cubos
```

---

## 6.7 Función `_pixel_a_robot`

```python
def _pixel_a_robot(self, px, py):
    """
    Convierte coordenadas de píxel a milímetros en el sistema del robot.
    Aplica corrección de distorsión radial + homografía.

    Args:
        px, py: coordenadas en píxeles

    Returns:
        x_robot, y_robot: coordenadas en mm (sistema robot)
    """
    # 1. Corrección de distorsión barrel
    px_c, py_c = corregir_distorsion(px, py)

    # 2. Transformación de homografía
    punto_pixel = np.array([[[px_c, py_c]]], dtype=np.float32)
    punto_robot = cv2.perspectiveTransform(punto_pixel, self.H)

    x_robot = float(punto_robot[0, 0, 0])
    y_robot = float(punto_robot[0, 0, 1])

    return x_robot, y_robot
```

---

[← Implementación Industrial](../05-implementacion-industrial) &nbsp;&nbsp; [Resultados →](../07-resultados){: .btn .btn-outline }
