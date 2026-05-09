---
layout: default
title: Introducción
nav_order: 2
description: "Contexto, hardware y objetivos del CoBot Clasificador de Colores"
permalink: /01-introduccion/
---

# 1. Introducción
{: .no_toc }

Contexto del problema, descripción del hardware y objetivos del proyecto.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1.1 Contexto del Problema

En entornos industriales de manufactura y logística, la **clasificación manual de piezas** representa una tarea repetitiva, lenta y propensa a errores humanos. Cuando cubos de diferentes colores se depositan de forma desordenada sobre una superficie compartida, un operador debe identificar visualmente el color de cada pieza y transportarla a su zona de destino correspondiente, lo que implica:

- Fatiga cognitiva y física a lo largo de turnos prolongados
- Variabilidad en el tiempo de ciclo según el estado del operador
- Riesgo de clasificación errónea bajo condiciones de iluminación variable
- Baja escalabilidad ante incrementos en el volumen de piezas

El **CoBot Clasificador de Colores** resuelve este problema integrando un brazo robótico colaborativo con visión artificial en tiempo real, eliminando la intervención humana durante la operación de clasificación y garantizando tiempos de ciclo constantes y precisión submilimétrica.

> **Definición del problema:** Dado un conjunto de cubos de colores (rojo, verde, azul) depositados aleatoriamente en un área de trabajo compartida, el sistema debe identificar cada cubo, calcular su posición cartesiana precisa y transportarlo autónomamente a su zona de color asignada.

---

## 1.2 Hardware del Sistema

### 1.2.1 Brazo Robótico — Universal Robots UR3

El manipulador principal es el **UR3 de Universal Robots**, un robot colaborativo de 6 grados de libertad diseñado para cargas útiles de hasta 3 kg y alcance máximo de 500 mm.

| Parámetro | Valor |
|---|---|
| **Grados de libertad** | 6 (6 articulaciones rotacionales) |
| **Carga útil máxima** | 3 kg |
| **Alcance** | 500 mm |
| **Repetibilidad** | ±0.1 mm |
| **Peso del robot** | 11.2 kg |
| **Protección** | IP54 |
| **Interfaz de programación** | TCP/IP puerto 30002 (URScript) |
| **Velocidad máx. articulación** | 180 °/s |

El UR3 se controla mediante **comandos URScript** enviados directamente a través de un socket TCP al puerto 30002, sin necesidad de un teach pendant o interfaz gráfica durante la operación autónoma.

### 1.2.2 Gripper — OnRobot Soft Gripper

El efector final es un **gripper neumático OnRobot Soft Gripper** controlado mediante una API HTTP REST integrada al controlador del robot.

| Parámetro | Valor |
|---|---|
| **Tipo** | Neumático de dedos suaves |
| **Interfaz** | HTTP REST (JSON) |
| **Endpoint grip** | `POST /api/v1/gripper/grip` |
| **Endpoint release** | `POST /api/v1/gripper/release` |
| **Tiempo de respuesta** | ~200 ms |

```python
# Control del gripper OnRobot vía HTTP REST
import requests

GRIPPER_URL = "http://192.168.1.1"  # IP del controlador OnRobot

def grip():
    """Activa el gripper para agarrar un cubo."""
    response = requests.post(f"{GRIPPER_URL}/api/v1/gripper/grip")
    return response.status_code == 200

def release():
    """Libera el gripper para soltar un cubo."""
    response = requests.post(f"{GRIPPER_URL}/api/v1/gripper/release")
    return response.status_code == 200
```

### 1.2.3 Cámara — Logitech C920

La cámara de visión artificial es una **Logitech C920** montada en configuración **top-down** (vista cenital) sobre el área de trabajo.

| Parámetro | Valor |
|---|---|
| **Modelo** | Logitech C920 HD Pro Webcam |
| **Resolución** | 1920 × 1080 px (operación: 1280 × 720) |
| **FPS** | 30 fps |
| **Montaje** | Top-down sobre el área de trabajo |
| **Conexión** | USB 2.0 / 3.0 |
| **Campo visual** | Cubre el área de trabajo completa (920 × 420 mm) |

La posición fija de la cámara permite calcular una **homografía estática** al inicio de cada sesión usando marcadores ArUco, que transforma coordenadas de píxel a milímetros en el espacio del robot.

---

## 1.3 Área de Trabajo y Zonas de Color

El área de trabajo mide **920 mm × 420 mm** y está dividida en tres zonas fijas de destino para la clasificación:

```
┌─────────────────────────────────────────────────────┐
│                                                     │
│   ZONA VERDE        ZONA ROJA        ZONA AZUL      │
│   (Izquierda)       (Centro)        (Derecha)       │
│   X: -460 a -150    X: -150 a 150   X: 150 a 460   │
│   Y: -510 a -90     Y: -510 a -90   Y: -510 a -90  │
│                                                     │
│         ← ← ← 920 mm → → →                         │
└─────────────────────────────────────────────────────┘
```

| Zona | Color | Posición X (mm) | Posición Y (mm) |
|---|---|---|---|
| **Izquierda** | 🟢 Verde | -300 | -300 |
| **Centro** | 🔴 Rojo | 0 | -300 |
| **Derecha** | 🔵 Azul | 300 | -300 |

Las coordenadas están expresadas en el sistema de referencia de la **base del robot UR3**, con origen en el centro de la brida de montaje.

---

## 1.4 Arquitectura del Sistema

El sistema se compone de cuatro módulos principales que operan concurrentemente:

```
┌─────────────────┐     ┌─────────────────┐
│  camera_detector │────▶│  zone_manager   │
│  (Hilo visión)  │     │  (Lógica SORT)  │
└─────────────────┘     └────────┬────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   robot_controller      │
                    │   (IK + TCP + Gripper)  │
                    └─────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   UR3  ←→  Gripper      │
                    │   TCP:30002  HTTP REST  │
                    └─────────────────────────┘
```

---

## 1.5 Objetivos del Proyecto

### Objetivo General

Desarrollar un sistema autónomo de clasificación robótica que integre visión artificial con cinemática inversa analítica para clasificar cubos de colores mediante un brazo UR3, operando en tiempo real sin intervención humana.

### Objetivos Específicos

1. **Implementar la cinemática directa** del UR3 mediante los parámetros Denavit-Hartenberg para calcular la posición del TCP dado un vector articular.

2. **Derivar y programar la cinemática inversa analítica** del UR3 para calcular los 6 ángulos articulares dado un punto cartesiano y orientación deseados.

3. **Implementar el algoritmo Levenberg-Marquardt** como método numérico alternativo de cinemática inversa y comparar con el método analítico.

4. **Diseñar trayectorias suaves** usando polinomios quínticos y perfil trapezoidal de velocidad para las fases de pick and place.

5. **Calibrar la homografía cámara-robot** usando marcadores ArUco para transformar coordenadas de píxel a milímetros con precisión < 5 mm.

6. **Implementar el pipeline de visión** con segmentación HSV, filtros morfológicos y corrección de distorsión para detectar cubos de colores de forma robusta.

7. **Integrar el sistema completo** en los modos GO (clasificar un cubo individual) y SORT (clasificar todos los cubos mal colocados de forma autónoma).

---

[← Inicio](/) &nbsp;&nbsp; [Cinemática Directa →](../02-cinematica-directa){: .btn .btn-outline }
