---
layout: default
title: Introducción
nav_order: 2
description: "Contexto del problema, hardware real y objetivos del CoBot Clasificador"
permalink: /01-introduccion/
---

# 1. Introducción
{: .no_toc }

Contexto del problema, descripción del hardware real y objetivos del proyecto.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1.1 Contexto del Problema

En entornos industriales de manufactura y logística, la **clasificación manual de piezas** representa una tarea repetitiva, lenta y propensa a errores. En este proyecto se aborda el caso específico de cubos de colores (rojo, verde, azul) depositados de forma desordenada sobre una superficie compartida, donde un operador debe identificarlos visualmente y transportarlos a su zona de destino.

El **CoBot Clasificador de Colores** resuelve este problema integrando un brazo robótico colaborativo con visión artificial en tiempo real, eliminando la intervención humana durante la operación de clasificación y garantizando tiempos de ciclo constantes con precisión submilimétrica.

> **Problema formal:** Dado un conjunto de cubos de colores depositados aleatoriamente en un área de trabajo de 920 × 420 mm, detectar cada cubo mediante visión artificial, calcular su posición cartesiana, calcular la cinemática inversa y ejecutar el ciclo de pick and place autónomamente.

---

## 1.2 Hardware del Sistema

### 1.2.1 Brazo Robótico — Universal Robots UR3

![Robot UR3 con cubos de colores en el área de trabajo](../assets/img/robot-workspace.jpeg)
*UR3 de 6 GDL sobre el área de trabajo. Se pueden observar los cubos rojo, verde y azul, las zonas de destino marcadas con papel de color y la estructura de aluminio.*

El manipulador principal es el **UR3 de Universal Robots**, robot colaborativo de 6 grados de libertad diseñado para cargas útiles de hasta 3 kg.

| Parámetro | Valor |
|---|---|
| **Grados de libertad** | 6 (6 articulaciones rotacionales) |
| **Carga útil máxima** | 3 kg |
| **Alcance** | 500 mm |
| **Repetibilidad** | ±0.1 mm |
| **Interfaz de programación** | TCP/IP puerto 30002 (URScript directo) |
| **IP en el proyecto** | 192.168.1.74 |
| **Modo de operación** | REMOTE (sin teach pendant) |

El UR3 se controla mediante comandos **URScript** (`movej`) enviados directamente a través de un socket TCP al puerto 30002, sin necesidad de RoboDK ni de ningún software intermediario.

### 1.2.2 Gripper — OnRobot Soft Gripper

![Gripper OnRobot realizando un pick sobre la zona verde](../assets/img/robot-gripper.jpeg)
*OnRobot Soft Gripper durante la fase de APPROACH/PICK. Se observa el gripper bajando sobre la zona de destino verde.*

| Parámetro | Valor |
|---|---|
| **Tipo** | Neumático de dedos suaves (soft gripper) |
| **Interfaz** | HTTP REST (API OnRobot) |
| **IP en el proyecto** | 192.168.1.1 (brida OnRobot) |
| **Endpoint grip** | `POST /api/dc/weblogic/run/7742` |
| **Endpoint release** | `POST /api/dc/weblogic/run/335` |
| **Inicialización** | `GET /api/dc/sg/initialize/0/1` |
| **Tiempo de cierre** | ~2.0 s (incluye margen de seguridad) |

{% highlight python %}
# robot_controller.py — Control del gripper OnRobot vía HTTP REST
import requests

IP_BRIDA    = "192.168.1.1"
URL_INIT    = f"http://{IP_BRIDA}/api/dc/sg/initialize/0/1"
URL_GRIP    = f"http://{IP_BRIDA}/api/dc/weblogic/run/7742"
URL_RELEASE = f"http://{IP_BRIDA}/api/dc/weblogic/run/335"

def cerrar_brida(self):
    respuesta = requests.post(URL_GRIP, timeout=3)
    if respuesta.status_code == 200:
        print("  [Brida] GRIP ✓")
    time.sleep(2.0)  # Tiempo para que agarre bien

def abrir_brida(self):
    respuesta = requests.post(URL_RELEASE, timeout=3)
    if respuesta.status_code == 200:
        print("  [Brida] RELEASE ✓")
    time.sleep(2.0)  # Tiempo para soltar bien
{% endhighlight %}

### 1.2.3 Cámara — Logitech C920

La cámara de visión artificial es una **Logitech C920** montada en configuración **top-down** (vista cenital) sobre el área de trabajo, compartiendo la misma instancia `cv2.VideoCapture` entre el módulo de calibración ArUco y el detector de cubos.

| Parámetro | Valor |
|---|---|
| **Modelo** | Logitech C920 HD Pro |
| **Resolución de operación** | 1280 × 720 px |
| **FPS** | 30 fps |
| **Índice en el sistema** | 1 (cámara 0 = webcam integrada) |
| **Buffer** | 1 frame (para evitar latencia) |
| **Distorsión** | Radial barrel (K = −2×10⁻⁵, actualmente desactivada por la H) |

{% highlight python %}
# main.py — Apertura de cámara compartida entre calibración y detección
import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # índice 1, Windows DirectShow
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)       # ← buffer=1 para latencia mínima
time.sleep(1.2)  # calentar sensor antes de capturar ArUcos
{% endhighlight %}

---

## 1.3 Área de Trabajo y Zonas de Color

El área de trabajo mide **920 mm × 420 mm** y está dividida en tres zonas de destino marcadas con papel de color sobre la superficie. Las zonas están definidas en `zone_manager.py`:

{% highlight python %}
# zone_manager.py — Definición real de zonas
ZONAS = {
    "VERDE": {
        "centro_x":  275.0,   # mm en coordenadas del robot
        "centro_y": -294.0,
        "ancho":    250.0,
        "alto":     420.0,
        "slots_nx":   2,
        "slots_ny":   2,
    },
    "ROJO": {
        "centro_x":   0.0,
        "centro_y": -294.0,
        "ancho":    250.0,
        "alto":     420.0,
        "slots_nx":   2,
        "slots_ny":   2,
    },
    "AZUL": {
        "centro_x": -275.0,
        "centro_y": -294.0,
        "ancho":    250.0,
        "alto":     420.0,
        "slots_nx":   2,
        "slots_ny":   2,
    },
}
{% endhighlight %}

| Zona | Color | Centro X (mm) | Centro Y (mm) | Posición visual |
|---|---|:---:|:---:|---|
| **Verde** | 🟢 Verde | +275 | -294 | Derecha (desde la cámara) |
| **Rojo** | 🔴 Rojo | 0 | -294 | Centro |
| **Azul** | 🔵 Azul | -275 | -294 | Izquierda (desde la cámara) |

> **Nota:** La orientación de los ejes depende de dónde está montada la base del UR3. Los valores positivos de X apuntan hacia el lado donde está instalado el robot.

---

## 1.4 Alturas de Operación

El sistema trabaja con tres alturas Z fijas, configurables desde el menú de `main.py`:

| Fase | Z (mm) | Descripción |
|---|:---:|---|
| **PREMOVE** | 240 | Tránsito alto — evita colisiones con estructura y barras |
| **APPROACH** | 210 | Pre-agarre — se detiene y verifica alineación antes de bajar |
| **PICK / PLACE** | 160 | Contacto — altura de agarre y depósito del cubo |

---

## 1.5 Objetivos del Proyecto

### Objetivo General

Desarrollar un sistema autónomo de clasificación robótica que integre visión artificial con cinemática inversa analítica para clasificar cubos de colores mediante el UR3, operando en tiempo real sin intervención humana.

### Objetivos Específicos

1. Implementar la **cinemática directa** del UR3 con los parámetros DH reales.
2. Derivar y programar la **cinemática inversa analítica** con cascada de fallback (4 ramas + Levenberg-Marquardt).
3. Diseñar **trayectorias suaves** con polinomios quínticos para las 3 fases de pick.
4. Calibrar la **homografía cámara-robot** con ArUco (error < 8 mm).
5. Implementar el **pipeline de visión** con segmentación HSV robusta ante variaciones de iluminación.
6. Integrar el sistema en los modos **GO** (un cubo) y **SORT** (todos los mal colocados).
7. Obtener reconocimiento en el **Día de las Ingenierías IBERO 2026** — ✅ 3er Lugar.

---

[← Inicio](../) &nbsp;&nbsp; [Cinemática Directa →](../02-cinematica-directa){: .btn .btn-outline }