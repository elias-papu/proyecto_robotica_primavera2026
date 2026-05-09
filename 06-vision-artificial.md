---
layout: default
title: Visión Artificial
nav_order: 7
description: "Pipeline HSV real de camera_detector.py — rangos, filtros y homografía ArUco"
permalink: /06-vision-artificial/
---

# 6. Visión Artificial
{: .no_toc }

Pipeline real de `camera_detector.py` y `calibrar_homografia.py`.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 6.1 Pipeline Completo

```
Frame BGR (1280×720) ─→ HSV ─→ Máscara por color ─→ Morfología
  ─→ Contornos ─→ 4 filtros geométricos ─→ Centroide (px, py)
  ─→ Corrección distorsión (K_DIST) ─→ Homografía H
  ─→ (X_mm, Y_mm) en coordenadas del robot
```

---

## 6.2 Rangos HSV Calibrados

Del archivo `camera_detector.py` — calibrados con `calibrar_color.py`:

```python
# camera_detector.py — Rangos HSV calibrados en laboratorio IBERO
COLOR_RANGES = {
    "ROJO": [
        (np.array([0,   130,  70]), np.array([10,  255, 255])),
        (np.array([170, 130,  70]), np.array([180, 255, 255])),
    ],
    "AZUL": [
        (np.array([85,  130,  80]), np.array([105, 255, 255])),
    ],
    "VERDE": [
        # Rango ampliado: cubre verde claro, oscuro y verde-amarillento
        # S_min=60 y V_min=40 para condiciones de iluminación variable
        (np.array([35,  60,  40]), np.array([90,  255, 255])),
    ],
}
```

| Color | H_min | S_min | V_min | H_max | S_max | V_max | Notas |
|---|:---:|:---:|:---:|:---:|:---:|:---:|---|
| **Rojo inferior** | 0 | 130 | 70 | 10 | 255 | 255 | Rango bajo del espectro |
| **Rojo superior** | 170 | 130 | 70 | 180 | 255 | 255 | Rango alto (wrap-around) |
| **Azul turquesa** | 85 | 130 | 80 | 105 | 255 | 255 | Cubos de color turquesa |
| **Verde lima** | 35 | 60 | 40 | 90 | 255 | 255 | Rango ampliado para robustez |

---

## 6.3 Filtros Geométricos — Código Real

```python
# camera_detector.py — _detectar_cubos_en_frame() (extracto)

for c in cnts:
    # A. Área en píxeles
    area_px = cv2.contourArea(c)
    if area_px < 1000 or area_px > 15000:
        continue

    # B. Solidez: S = área / área_convexHull > 0.85
    hull      = cv2.convexHull(c)
    hull_area = cv2.contourArea(hull)
    solidity  = float(area_px) / hull_area if hull_area > 0 else 0
    if solidity < 0.85:
        continue

    # C. Relación de aspecto: 0.75 < W/H < 1.30
    x, y, w, h = cv2.boundingRect(c)
    ar = float(w) / h if h > 0 else 0
    if not (0.75 < ar < 1.30):
        continue

    # D. Número de vértices (forma poligonal): 4 ≤ n ≤ 6
    peri  = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    if not (4 <= len(approx) <= 6):
        continue

    # E. Posición vertical válida (ignorar parte baja = brazo/fondo)
    if v_px > 520:   # _ZONA_VALIDA_Y_MAX = 520 px
        continue

    # F. Workspace del robot (filtro final)
    if not (-500 <= x_mm <= 500 and -600 <= y_mm <= 100):
        continue
```

| Filtro | Condición | Justificación |
|---|---|---|
| **Área** | 1000 ≤ A ≤ 15000 px² | Elimina ruido pequeño y regiones grandes |
| **Solidez** | S > 0.85 | Cubo visto de arriba es muy compacto |
| **Aspecto** | 0.75 < W/H < 1.30 | Proyección cenital ≈ cuadrado |
| **Polígono** | 4 ≤ vértices ≤ 6 | `approxPolyDP` confirma forma cuadrada |
| **Y_máx** | v_px < 520 | Ignora brazo y fondo inferior |
| **Workspace** | X: ±500, Y: −600..+100 mm | Descarta falsas detecciones lejanas |

---

## 6.4 Calibración ArUco — `calibrar_homografia.py`

### Coordenadas Reales de los Marcadores

```python
# calibrar_homografia.py — Coordenadas medidas con TCP del robot
COORDS_ROBOT_MM = {
    0: (  435.0,  -490.0),   # ID0 — esquina superior izquierda (en imagen)
    1: ( -435.0,  -485.0),   # ID1 — esquina superior derecha
    2: (  435.0,  -120.0),   # ID2 — esquina inferior izquierda
    3: ( -435.0,  -115.0),   # ID3 — esquina inferior derecha
}
```

### Proceso de Calibración

```python
# calibrar_homografia.py — Cálculo de homografía con RANSAC

def _calcular_H(centros_px, coords_robot):
    ids_ok    = [i for i in range(4) if i in centros_px]
    pts_px    = np.array([centros_px[i]   for i in ids_ok], dtype="float32")
    pts_robot = np.array([coords_robot[i] for i in ids_ok], dtype="float32")

    H, _ = cv2.findHomography(pts_px, pts_robot, cv2.RANSAC, 5.0)
    return H
```

El proceso completo:
1. Mostrar feed en vivo hasta que el usuario presiona ENTER
2. Promediar **40 frames** para mayor estabilidad del centroide
3. Calcular H con `findHomography` + RANSAC (umbral 5 px)
4. Verificar error de reproyección (umbral de advertencia: 8 mm)
5. Guardar en caché JSON para siguientes sesiones

### Caché JSON

```python
# calibrar_homografia.py — Formato del caché
{
  "H": [[h00, h01, h02], [h10, h11, h12], [h20, h21, h22]],
  "coords": {
    "0": [435.0, -490.0],
    "1": [-435.0, -485.0],
    "2": [435.0, -120.0],
    "3": [-435.0, -115.0]
  }
}
```

---

## 6.5 Transformación Pixel → Robot

```python
# camera_detector.py — _pixel_a_robot()
def _pixel_a_robot(u, v):
    """
    Transforma pixel (u, v) → coordenadas del robot (mm).
    La distorsión radial está desactivada porque la H calibrada
    con ArUco ya absorbe la transformación perspectiva completa.
    """
    u_c, v_c = _corregir_distorsion(u, v)   # K_DIST=0.0 → sin efecto
    p = np.array([u_c, v_c, 1.0]).reshape(3, 1)
    q = _H @ p
    w = q[2, 0]
    if abs(w) < 1e-9:
        return 0.0, 0.0
    return float(q[0,0] / w), float(q[1,0] / w)
```

---

## 6.6 Herramienta de Calibración de Color

El proyecto incluye `calibrar_color.py` — una herramienta interactiva con trackbars para ajustar los rangos HSV en tiempo real:

```python
# calibrar_color.py — Uso
# python calibrar_color.py
# Ajusta los sliders hasta que solo el cubo objetivo sea blanco en la máscara
# Presiona 'q' para imprimir el rango final en la terminal
#
# Output:
#   RANGO FINAL:
#   (np.array([45, 100, 60]), np.array([80, 255, 220]))
```

---

[← Implementación Industrial](../05-implementacion-industrial) &nbsp;&nbsp; [Resultados →](../07-resultados){: .btn .btn-outline }
