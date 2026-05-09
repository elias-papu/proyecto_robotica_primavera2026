---
layout: default
title: Código Fuente
nav_order: 9
description: "Descripción real de los módulos Python del proyecto"
permalink: /08-codigo-fuente/
---

# 8. Código Fuente
{: .no_toc }

Descripción de los módulos Python reales del proyecto CoBot Clasificador de Colores.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 8.1 Estructura del Repositorio

```
proyecto/
│
├── main.py                   # Orquestador: menú, GO, SORT, SORT-BUCLE
├── camera_detector.py        # Hilo de detección HSV + homografía
├── calibrar_homografia.py    # Calibración ArUco + caché JSON
├── inverse_kinematics.py     # IK analítica + cascada de 5 estrategias
├── ik_numerica.py            # Levenberg-Marquardt (fallback)
├── robot_controller.py       # TCP:30002 + OnRobot HTTP REST
├── trajectory_planner.py     # Polinomio quíntico + IK numérica
├── zone_manager.py           # Zonas, slots libres, detección de lugar
│
├── calibrar_color.py         # Herramienta interactiva de ajuste HSV
├── identificador_cubos.py    # Script standalone de detección (debug)
│
└── homografia_cache.json     # Caché de la matriz H (se regenera con ArUco)
```

---

## 8.2 `main.py` — Orquestador Principal

**Responsabilidad:** Punto de entrada. Inicializa la cámara (instancia compartida), calibra la homografía, lanza el detector en hilo de fondo, conecta con el robot y presenta el menú interactivo.

**Comandos:** `scan`, `go`, `sort`, `sortb`, `home`, `entrega`, `brida open/close`, `config`, `salir`

**Funciones clave:**

| Función | Descripción |
|---|---|
| `main()` | Arranque: cámara → ArUco → detector → robot → menú |
| `flujo_pick(detector, robot)` | GO: scan → selección → IK × 3 → confirmación → ejecutar |
| `flujo_sort(detector, robot)` | SORT: detectar mal colocados → confirmación → `_nucleo_sort` |
| `_nucleo_sort(detector, robot, a_mover)` | Ciclo pick-place sin HOME intermedio entre cubos |
| `flujo_sort_bucle(detector, robot)` | SORT-BUCLE: polling continuo con Ctrl+C para salir |
| `calcular_y_mostrar_ik(cubo)` | IK para PREMOVE + APPROACH + PICK, muestra en terminal |
| `editar_config()` | Editar z_pick, z_approach, z_premove, rama_q2 en tiempo de ejecución |

**Banner de terminal:**
```python
BANNER = """
╔══════════════════════════════════════════════════════════╗
║         UR3 PICK & PLACE — SISTEMA DE VISIÓN             ║
║         Cámara C920  ·  OnRobot Soft Gripper             ║
╚══════════════════════════════════════════════════════════╝
"""
```

---

## 8.3 `camera_detector.py` — Hilo de Detección

**Responsabilidad:** Captura frames de la cámara en un hilo `daemon` independiente. Aplica el pipeline completo de visión (HSV → filtros → centroide → homografía) y expone los cubos detectados de forma thread-safe.

**Clase principal:** `CubeDetector`

| Método | Descripción |
|---|---|
| `start()` | Lanza el hilo `_loop()` como daemon |
| `stop()` | Detiene el hilo; libera el cap solo si es interno |
| `get_cubos()` | Retorna copia thread-safe de los cubos del último frame |
| `_loop()` | Loop de captura y procesamiento (30 fps) |

**Inyección de homografía:**

```python
# En main.py, después de calibrar:
set_homografia(H_calibrada)  # ← modifica _H global en camera_detector.py
```

**Campos de cada cubo detectado:**

```python
{
    "color":    "ROJO",     # "ROJO" | "AZUL" | "VERDE"
    "u_px":     640,        # centroide en píxeles
    "v_px":     360,
    "x_mm":     125.3,      # coordenadas del robot
    "y_mm":    -287.6,
    "area_px":  3200,
    "solidity": 0.92,
    "ar":       1.05,
    "contour":  ...,        # cv2 contorno para dibujar
    "bbox":     (x,y,w,h),
}
```

---

## 8.4 `calibrar_homografia.py` — Calibración ArUco

**Responsabilidad:** Detecta los 4 marcadores ArUco en las esquinas del área de trabajo, promedia 40 frames para estabilidad, calcula la matriz H con `findHomography + RANSAC`, verifica el error de reproyección y guarda en caché JSON.

**Funciones clave:**

| Función | Descripción |
|---|---|
| `calibrar_al_inicio(cap, forzar)` | API pública llamada por `main.py` al arrancar |
| `_detectar_centros(cap, n_frames=40)` | Feed en vivo + captura interactiva con ENTER |
| `_calcular_H(centros_px, coords_robot)` | `findHomography` con RANSAC |
| `_verificar(H, centros, coords)` | Error de reproyección por marcador |
| `_guardar_cache(H, coords)` | JSON con H y coordenadas de robot |
| `_cargar_cache(coords_actuales)` | Carga solo si las coordenadas no cambiaron |

---

## 8.5 `inverse_kinematics.py` — IK Analítica

**Responsabilidad:** IK analítica completa del UR3 con cascada de 5 estrategias de fallback. Implementa la traducción del código MATLAB original al proyecto.

**Parámetros DH:** `_a2=-0.24365`, `_a3=-0.21325`, `_d1=0.1519`, `_d4=0.11235`, `_d5=0.08535`, `_d6=0.0819`

**Funciones clave:**

| Función | Descripción |
|---|---|
| `calcular_ik(px, py, pz, rot, rama_q2, rama_q1)` | API pública con cascada completa |
| `calcular_ik_raw(...)` | IK analítica pura, sin fallback |
| `_intentar_todas_ramas(px, py, pz, rot)` | Prueba las 4 combinaciones elbow×shoulder |
| `_normalizar_rad(ang)` | Normaliza a [−π, π] |
| `imprimir_ik(res, prefijo)` | Imprime ángulos formateados en terminal |
| `mm_a_metros(x, y, z)` | Conversión de unidades |

---

## 8.6 `ik_numerica.py` — Levenberg-Marquardt

**Responsabilidad:** Fallback numérico de la IK analítica. Usa Jacobiano 6×6 por diferencias finitas (dq=1e-6 rad) con amortiguamiento λ=0.05 y pesos W=[10,10,10,0.5,0.5,0.5].

| Parámetro | Valor |
|---|---|
| `max_iter` | 100 |
| `tol` | 1e-4 m (posición) |
| `dq` Jacobiano | 1e-6 rad |
| `lambda` (damping) | 0.05 |

---

## 8.7 `robot_controller.py` — Control del Robot

**Responsabilidad:** Comunicación TCP con el UR3 (puerto 30002) y control del gripper OnRobot vía HTTP REST. Gestiona la secuencia de pick en 3 fases y el waypoint de rodeo.

| Constante | Valor |
|---|---|
| `ROBOT_IP` | 192.168.1.74 |
| `ROBOT_PORT` | 30002 |
| `IP_BRIDA` | 192.168.1.1 |
| `HOME_J` | [0, −90, −90, 0, 90, 0]° |
| `ENTREGA_J` | [−90, −90, −106, −68, 90, 0]° |
| `WAYPOINT_RODEO_J` | [0, −110, −90, −70, 90, 0]° |
| `VEL_J` | 1.0 rad/s |
| `VEL_LENTO` | 0.25 rad/s |

**Métodos públicos:**

| Método | Descripción |
|---|---|
| `inicializar_brida()` | GET al endpoint de inicialización OnRobot |
| `cerrar_brida()` | POST grip + sleep 2.0 s |
| `abrir_brida()` | POST release + sleep 2.0 s |
| `ir_a_home()` | movej a HOME_J |
| `ejecutar_pick(...)` | Secuencia completa PREMOVE→APPROACH→PICK→PLACE |
| `ejecutar_place(...)` | Place en zona destino |

---

## 8.8 `trajectory_planner.py` — Trayectorias

**Responsabilidad:** Genera trayectorias cartesianas suaves con polinomio quíntico e incluye su propia IK numérica (Jacobiano transpuesto) para verificar alcanzabilidad punto a punto.

| Función | Descripción |
|---|---|
| `polinomio_quintico(x0, xf, t_total, n)` | Genera pos/vel/acc para n puntos |
| `planificar_trayectoria_cartesiana(p0, pf, t)` | Interpolación quíntica XYZ independiente |
| `cinematica_directa(q_rad)` | CD interna para verificación |
| `jacobiano(q_rad)` | Jacobiano 6×6 por diferencias finitas |
| `verificar_trayectoria(puntos, q_inicial)` | Resuelve IK en cada punto, retorna joints |
| `planificar_pick_place(x, y, z, ...)` | Planificación completa con rodeo opcional |

---

## 8.9 `zone_manager.py` — Gestión de Zonas

**Responsabilidad:** Define las 3 zonas de clasificación, genera slots en cuadrícula 2×2 por zona, y determina el slot libre más central para el depósito.

```python
# zone_manager.py — Mapa de destinos
DESTINO_POR_COLOR = {
    "VERDE": "VERDE",
    "ROJO":  "ROJO",
    "AZUL":  "AZUL",
}

RADIO_OCUPADO_MM = 30.0  # radio para declarar un slot como ocupado
```

| Función | Descripción |
|---|---|
| `obtener_destino(color, cubos_detectados)` | Slot libre más central en la zona del color |
| `zona_ya_en_destino(cubo)` | True si el cubo ya está dentro de su zona correcta |
| `_generar_slots(zona)` | Lista de (x, y) ordenada por distancia al centro |
| `_slot_ocupado(sx, sy, cubos)` | True si algún cubo está dentro del radio |

---

[← Resultados](../07-resultados) &nbsp;&nbsp; [Ver reconocimiento 🥉](../reconocimiento){: .btn .btn-primary } &nbsp;&nbsp; [↑ Inicio](../){: .btn .btn-outline }
