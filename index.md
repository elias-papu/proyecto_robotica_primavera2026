---
layout: home
title: Inicio
nav_order: 1
description: "CoBot Manipulador Clasificador de Colores con Visión Artificial y Cinemática Inversa"
permalink: /
---

# CoBot Manipulador Clasificador de Colores
## con Visión Artificial y Cinemática Inversa
{: .no_toc }

---

| Campo | Detalle |
|---|---|
| **Estudiante** | Elias Santiago Jiménez Hernández |
| **Materia** | Control Avanzado de Robots |
| **Año** | 2025 |
| **Institución** | Universidad Iberoamericana (IBERO) |
| **Robot** | Universal Robots UR3 — 6 GDL |

---

## Resumen del Sistema
{: .no_toc }

El **CoBot Clasificador de Colores** es un sistema autónomo de *pick and place* que integra visión artificial por computadora con cinemática inversa analítica para clasificar cubos de colores (rojo, verde, azul) dispersos en un área de trabajo de **920 × 420 mm**. Una cámara Logitech C920 montada en configuración *top-down* captura el escenario; un pipeline OpenCV basado en segmentación HSV y homografía ArUco convierte píxeles en coordenadas cartesianas del robot. El controlador calcula la configuración articular del UR3 en tiempo real mediante cinemática inversa analítica, envía comandos `movej` directamente al puerto TCP 30002 y controla un gripper neumático OnRobot vía HTTP REST, completando ciclos de clasificación sin intervención humana en el **modo SORT**.

---

## Tecnologías Utilizadas
{: .no_toc }

| Tecnología | Versión / Detalle | Rol en el Sistema |
|---|---|---|
| **Python** | 3.10+ | Lógica principal, IK, comunicación |
| **OpenCV** | 4.x | Captura, segmentación HSV, ArUco |
| **NumPy** | 1.24+ | Álgebra matricial, DH, Jacobiano |
| **Socket TCP** | Puerto 30002 | Comandos directos al UR3 |
| **ArUco Markers** | `cv2.aruco` | Calibración de homografía |
| **HTTP REST** | OnRobot API | Control del gripper neumático |
| **Threading** | `threading` | Detección visual en hilo separado |
| **JSON / CSV** | stdlib | Caché de calibración |

---

## Contenido de la Documentación
{: .no_toc }

1. [Introducción y Hardware](01-introduccion) — Contexto, objetivos y descripción del sistema físico
2. [Cinemática Directa](02-cinematica-directa) — Parámetros DH y matrices de transformación
3. [Control Cinemático](03-control-cinematico) — Jacobiano y control de velocidad proporcional
4. [Cinemática Inversa](04-cinematica-inversa) — Método analítico y Levenberg-Marquardt
5. [Implementación Industrial](05-implementacion-industrial) — Flujo completo del sistema
6. [Visión Artificial](06-vision-artificial) — Pipeline de detección y homografía
7. [Resultados](07-resultados) — Pruebas, métricas y comparativas
8. [Código Fuente](08-codigo-fuente) — Descripción de módulos

---

> **Nota:** Este sitio fue generado con [Just the Docs](https://github.com/just-the-docs/just-the-docs) y publicado en GitHub Pages. Todo el código del proyecto está disponible en el repositorio GitHub.

---

[Comenzar: Introducción →](01-introduccion){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 }
