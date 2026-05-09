---
layout: default
title: Código Fuente
nav_order: 9
description: "Descripción de los módulos Python del proyecto CoBot Clasificador"
permalink: /08-codigo-fuente/
---

# 8. Código Fuente
{: .no_toc }

Descripción de cada módulo Python del proyecto y su responsabilidad en el sistema.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 8.1 Estructura del Repositorio

```
cobot-clasificador/
│
├── main.py                   # Punto de entrada: menú, modos GO y SORT
├── camera_detector.py        # Hilo de detección y segmentación HSV
├── calibrar_homografia.py    # Calibración ArUco al inicio con caché JSON
├── inverse_kinematics.py     # IK analítica completa con parámetros DH
├── robot_controller.py       # Comunicación TCP + pick place + gripper
├── zone_manager.py           # Zonas, slots libres y lógica SORT
├── trayectorias.py           # Generación de trayectorias quínticas
│
├── config/
│   ├── robot_config.py       # IPs, puertos, parámetros de velocidad
│   └── hsv_config.py         # Rangos HSV calibrados por color
│
├── calibracion/
│   └── homografia.json       # Caché de la matriz H (se regenera con ArUco)
│
└── assets/
    └── img/                  # Imágenes del área de trabajo y marcadores
```

---

## 8.2 `main.py` — Menú Principal y Flujos GO / SORT

**Responsabilidad:** Punto de entrada del sistema. Inicializa todos los módulos, presenta el menú interactivo y orquesta los modos de operación GO y SORT.

```python
"""
main.py — CoBot Clasificador de Colores
Elias Santiago Jiménez Hernández
Control Avanzado de Robots 2025, IBERO
"""
import time
from camera_detector     import CameraDetector
from calibrar_homografia import calibrar_o_cargar_homografia
from robot_controller    import RobotController
from zone_manager        import ZoneManager
from inverse_kinematics  import calcular_ik

def main():
    # Inicializar subsistemas
    detector = CameraDetector(camara_id=0)
    robot    = RobotController(robot_ip="192.168.1.10")
    zones    = ZoneManager()

    # Calibración ArUco (usa caché si existe)
    H = calibrar_o_cargar_homografia(detector, cache_path="calibracion/homografia.json")
    detector.set_homografia(H)

    robot.conectar()
    robot.ir_home()

    # Menú principal
    while True:
        print("\n══ CoBot Clasificador ══")
        print("  [1] Modo GO    — Clasificar un cubo por color")
        print("  [2] Modo SORT  — Clasificar todos automáticamente")
        print("  [3] Recalibrar homografía ArUco")
        print("  [0] Salir")
        opcion = input("Selección: ").strip()

        if opcion == "1":
            color = input("Color (rojo/verde/azul): ").strip().lower()
            modo_go(detector, robot, zones, color)

        elif opcion == "2":
            modo_sort(detector, robot, zones)

        elif opcion == "3":
            H = calibrar_o_cargar_homografia(detector, forzar=True,
                                              cache_path="calibracion/homografia.json")
            detector.set_homografia(H)

        elif opcion == "0":
            robot.ir_home()
            robot.desconectar()
            detector.detener()
            print("Sistema detenido.")
            break


def modo_go(detector, robot, zones, color):
    """Clasifica un cubo del color especificado."""
    cubo = detector.obtener_cubo(color)
    if cubo is None:
        print(f"No se detectó cubo {color}.")
        return
    destino = zones.zona_de_color(color)
    robot.pick_and_place(cubo["x"], cubo["y"], destino)


def modo_sort(detector, robot, zones):
    """Clasifica todos los cubos mal colocados."""
    print("SORT iniciado...")
    while True:
        cubos = detector.obtener_todos_cubos()
        mal_colocados = zones.detectar_mal_colocados(cubos)
        if not mal_colocados:
            print("✓ Clasificación completa.")
            break
        for cubo in mal_colocados:
            destino = zones.zona_de_color(cubo["color"])
            robot.pick_and_place(cubo["x"], cubo["y"], destino)
            time.sleep(0.5)


if __name__ == "__main__":
    main()
```

**Funciones clave:**

| Función | Descripción |
|---|---|
| `main()` | Inicializa hardware, llama calibración, presenta menú |
| `modo_go(color)` | Clasifica un cubo del color especificado |
| `modo_sort()` | Loop hasta clasificar todos los cubos mal colocados |

---

## 8.3 `camera_detector.py` — Hilo de Detección y Segmentación HSV

**Responsabilidad:** Captura frames de la cámara en un hilo dedicado y aplica el pipeline de detección (HSV → morfología → contornos → filtros → centroide → homografía).

**Funciones principales:**

| Función | Descripción |
|---|---|
| `iniciar()` | Abre la cámara y lanza el hilo de captura |
| `detener()` | Detiene el hilo y libera la cámara |
| `_loop_deteccion()` | Loop interno: captura, procesa y actualiza estado compartido |
| `_detectar_cubos_en_frame(frame)` | Aplica segmentación HSV y filtros geométricos |
| `_pixel_a_robot(px, py)` | Transforma píxeles a mm usando la homografía |
| `obtener_cubo(color)` | Retorna el cubo detectado del color dado (thread-safe) |
| `obtener_todos_cubos()` | Retorna todos los cubos detectados actualmente |
| `set_homografia(H)` | Actualiza la matriz de homografía usada en conversión |

> **Arquitectura:** El detector corre en un `threading.Thread` independiente, actualizando un diccionario compartido protegido con `threading.Lock`. El hilo principal consulta el estado sin bloquear el control del robot.

---

## 8.4 `calibrar_homografia.py` — Calibración ArUco con Caché JSON

**Responsabilidad:** Detecta los 4 marcadores ArUco en las esquinas del área de trabajo y calcula la matriz de homografía cámara→robot. Guarda el resultado en JSON para evitar recalibrar en cada sesión.

**Funciones principales:**

| Función | Descripción |
|---|---|
| `calibrar_o_cargar_homografia(detector, forzar, cache_path)` | Carga caché si existe y `forzar=False`, o recalibra |
| `calibrar_homografia(frame)` | Detecta marcadores y llama a `findHomography` con RANSAC |
| `guardar_cache(H, path)` | Serializa la matriz H como lista en JSON |
| `cargar_cache(path)` | Deserializa y retorna la matriz H desde JSON |

**Coordenadas de referencia ArUco:**

```python
ARUCO_COORDS_ROBOT = {
    0: ( 460, -510),   # Esquina inferior derecha
    1: (-460, -510),   # Esquina inferior izquierda
    2: ( 460,  -90),   # Esquina superior derecha
    3: (-460,  -90),   # Esquina superior izquierda
}
```

---

## 8.5 `inverse_kinematics.py` — IK Analítica Completa con Parámetros DH

**Responsabilidad:** Implementa la cinemática directa e inversa analítica del UR3 usando los parámetros DH reales. Es el módulo más crítico del sistema.

**Funciones principales:**

| Función | Descripción |
|---|---|
| `dh_matrix(a, d, alpha, theta)` | Matriz de transformación homogénea DH estándar |
| `cinematica_directa(q)` | Retorna T, posición y rotación dado vector articular |
| `calcular_ik(T_deseada, elbow_down, rama_positiva)` | IK analítica completa (6 ángulos) |
| `jacobiano_numerico(q, h)` | Jacobiano 3×6 por diferencias finitas |
| `ik_levenberg_marquardt(p, q0, tol, max_iter)` | IK numérica LM como fallback |

**Parámetros DH:**

```python
DH_A     = [0, 0, -0.24365, -0.21325, 0, 0]
DH_D     = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
DH_ALPHA = [0, π/2, 0, 0, π/2, -π/2]
```

---

## 8.6 `robot_controller.py` — Comunicación TCP, Pick Place y Gripper

**Responsabilidad:** Gestiona la comunicación TCP con el UR3 (puerto 30002), ejecuta las 3 fases de pick (PREMOVE/APPROACH/PICK), controla el gripper OnRobot via HTTP REST, y valida el workspace antes de cada movimiento.

**Funciones principales:**

| Función | Descripción |
|---|---|
| `conectar()` | Abre socket TCP al UR3:30002 |
| `desconectar()` | Cierra socket de forma limpia |
| `mover_articular(q, a, v)` | Envía `movej` URScript al robot |
| `ir_home()` | Mueve a HOME: `[0, -π/2, -π/2, 0, π/2, 0]` |
| `pick_and_place(x, y, zona_destino)` | Ciclo completo: IK + 3 fases + grip + place + release |
| `_ejecutar_pick(x, y)` | PREMOVE (Z=240mm) → APPROACH (Z=210mm) → PICK (Z=160mm) |
| `_ejecutar_place(zona)` | Mueve a zona de destino y libera el gripper |
| `validar_workspace(x, y, z)` | Lanza excepción si la posición está fuera de límites |
| `necesita_rodeo(x, y)` | Detecta zona de barras (Y < -280 o X > 350 mm) |

**Secuencia de pick and place:**

```
HOME → [Rodeo si aplica] → PREMOVE (Z=240) →
APPROACH (Z=210) → PICK (Z=160) → GRIP →
PREMOVE (Z=240) → PLACE → RELEASE → HOME
```

---

## 8.7 `zone_manager.py` — Definición de Zonas, Slots y Lógica SORT

**Responsabilidad:** Define las tres zonas de clasificación (verde/rojo/azul), detecta qué cubos están en la zona incorrecta, y proporciona los slots libres dentro de cada zona para el apilamiento ordenado.

**Funciones principales:**

| Función | Descripción |
|---|---|
| `zona_de_color(color)` | Retorna coordenadas (X, Y) de la zona destino |
| `zona_del_cubo(x, y)` | Determina en qué zona está un cubo dado su posición |
| `color_esperado_en_zona(zona)` | Retorna qué color debe estar en una zona dada |
| `detectar_mal_colocados(cubos)` | Filtra cubos cuya zona actual ≠ zona esperada |
| `obtener_slot_libre(zona, cubos_actuales)` | Calcula el siguiente slot libre en la zona destino |

**Definición de zonas:**

```python
ZONAS = {
    "verde": {"x": -300, "y": -300, "color": "verde"},
    "rojo":  {"x":    0, "y": -300, "color": "rojo" },
    "azul":  {"x":  300, "y": -300, "color": "azul" },
}

LIMITES_ZONA = {
    "verde": {"x_min": -460, "x_max": -150},
    "rojo":  {"x_min": -150, "x_max":  150},
    "azul":  {"x_min":  150, "x_max":  460},
}
```

---

## 8.8 `trayectorias.py` — Trayectorias Quínticas y Trapezoidales

**Responsabilidad:** Calcula los coeficientes y evalúa polinomios quínticos para movimientos suaves entre puntos, así como perfiles trapezoidales de velocidad para movimientos de transporte rápido.

**Funciones principales:**

| Función | Descripción |
|---|---|
| `coeficientes_quintico(s0, sf, v0, vf, a0, af, tf)` | Resuelve el sistema 6×6 para los coeficientes |
| `evaluar_quintico(c, t_vec)` | Evalúa posición, velocidad y aceleración en t_vec |
| `trayectoria_xy(tf, dt)` | Trayectoria de demostración X: 250→300 mm, Y: 100→150 mm |
| `fases_pick(x, y, tf_fase, dt)` | Genera las 3 fases PREMOVE→APPROACH→PICK como trayectorias Z |
| `trayectoria_trapezoidal(s0, sf, v_max, a_max, dt)` | Perfil trapezoidal configurable |
| `graficar_trayectorias_xy()` | Genera gráficas de pos/vel/acel para X e Y |
| `comparar_metodos(s0, sf, tf)` | Compara quíntico vs trapezoidal en la misma gráfica |

---

[← Resultados](../07-resultados) &nbsp;&nbsp; [↑ Inicio](../){: .btn .btn-outline }
