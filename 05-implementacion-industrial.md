---
layout: default
title: Implementación Industrial
nav_order: 6
description: "Flujo real de main.py — modos GO, SORT, SORT-BUCLE y comunicación TCP"
permalink: /05-implementacion-industrial/
---

# 5. Implementación Industrial
{: .no_toc }

Sistema completo: modos GO, SORT y SORT-BUCLE con código real de `main.py` y `robot_controller.py`.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 5.1 Interfaz de Usuario — Menú de Comandos

El sistema se opera desde una terminal con los siguientes comandos (definidos en `main.py`):

```
╔══════════════════════════════════════════════════════════╗
║         UR3 PICK & PLACE — SISTEMA DE VISIÓN             ║
║         Cámara C920  ·  OnRobot Soft Gripper             ║
╚══════════════════════════════════════════════════════════╝

  Comandos disponibles:
  scan         → Ver cubos detectados ahora por la cámara
  go           → Scan + seleccionar cubo + mover robot
  sort         → Ordenar TODOS los cubos a su zona (una vez)
  sortb        → Sort en BUCLE continuo (Ctrl+C para salir)
  home         → Mover robot a posición HOME
  brida open   → Abrir brida manualmente
  brida close  → Cerrar brida manualmente
  config       → Ver / editar parámetros del sistema
  salir        → Cerrar programa
```

---

## 5.2 Flujo de Arranque

```python
# main.py — Flujo de arranque real
def main():
    # 1. Abrir cámara (instancia compartida)
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1.2)   # calentar sensor

    # 2. Calibrar homografía ArUco (usa caché si existe)
    H = calibrar_al_inicio(cap, forzar="--recalib" in sys.argv)
    set_homografia(H)   # inyectar al detector

    # 3. Iniciar detector de cubos en hilo de fondo
    detector = CubeDetector(camera_index=1, show_window=True,
                             cap_externo=cap)  # comparte el mismo cap
    detector.start()
    time.sleep(0.8)

    # 4. Conectar con el robot
    robot = RobotController(modo_sim=False)
    robot.inicializar_brida()   # GET /api/dc/sg/initialize/0/1
```

---

## 5.3 Modo GO — Flujo de Pick Individual

```python
# main.py — flujo_pick() real
def flujo_pick(detector, robot):
    # Captura puntual del estado de la cámara
    cubos = detector.get_cubos()
    mostrar_cubos(cubos)

    cubo = seleccionar_cubo(cubos)   # operador elige por número
    if cubo is None:
        return

    # Calcular IK para las 3 fases
    x_m, y_m, _ = mm_a_metros(cubo["x_mm"], cubo["y_mm"])
    ik_pm = calcular_ik(x_m, y_m, CONFIG["z_premove_mm"]/1000)
    ik_ap = calcular_ik(x_m, y_m, CONFIG["z_approach_mm"]/1000)
    ik_pk = calcular_ik(x_m, y_m, CONFIG["z_pick_mm"]/1000)

    # Confirmación del operador antes de mover
    if not pedir_confirmacion(cubo):
        return

    # Ejecutar secuencia
    robot.ejecutar_pick(
        joints_premove_deg  = ik_pm["joints_deg"],
        joints_approach_deg = ik_ap["joints_deg"],
        joints_prepick_deg  = ik_pk["joints_deg"],
        x_mm = cubo["x_mm"],
        y_mm = cubo["y_mm"],
    )
```

---

## 5.4 Modo SORT — Clasificación Autónoma

```python
# main.py — _nucleo_sort() (bucle principal de clasificación)
def _nucleo_sort(detector, robot, a_mover):
    slots_usados = {"ROJO": [], "AZUL": [], "VERDE": []}

    robot.ir_a_home()
    robot.abrir_brida()

    for i, cb in enumerate(a_mover):
        # Obtener slot destino (libre según cámara + registro interno)
        destino = destino_robusto(cb["color"], detector.get_cubos())
        if destino is None:
            warn(f"Zona {cb['color']} llena — saltando.")
            continue

        dx_mm, dy_mm = destino

        # Calcular IK para las 6 poses (3 pick + 3 place)
        ik_pm_pick  = calcular_ik(x_pick_m,  y_pick_m,  z_pm)
        ik_ap_pick  = calcular_ik(x_pick_m,  y_pick_m,  z_ap)
        ik_pk       = calcular_ik(x_pick_m,  y_pick_m,  z_pk)
        ik_pm_place = calcular_ik(x_place_m, y_place_m, z_pm)
        ik_ap_place = calcular_ik(x_place_m, y_place_m, z_ap)
        ik_pl       = calcular_ik(x_place_m, y_place_m, z_pk)

        # PICK (con waypoint de rodeo si aplica)
        if necesita_rodeo(cb["x_mm"], cb["y_mm"]):
            robot._move_j(WAYPOINT_RODEO_J, "RODEO → PICK")
        robot._move_j(ik_pm_pick["joints_deg"], "PREMOVE", usar_quintico=True)
        robot._move_j(ik_ap_pick["joints_deg"], "APPROACH", lento=True)
        robot._move_j(ik_pk["joints_deg"],      "PICK",     lento=True)
        robot.cerrar_brida()
        robot._move_j(ik_ap_pick["joints_deg"], "RETRACT",  lento=True)

        # PLACE (directo, sin HOME intermedio)
        robot._move_j(ik_pm_place["joints_deg"], "PREMOVE PLACE", usar_quintico=True)
        robot._move_j(ik_ap_place["joints_deg"], "APPROACH PLACE", lento=True)
        robot._move_j(ik_pl["joints_deg"],        "PLACE",         lento=True)
        robot.abrir_brida()
        robot._move_j(ik_ap_place["joints_deg"], "RETRACT PLACE", lento=True)

        slots_usados[cb["color"].upper()].append((dx_mm, dy_mm))

    robot.ir_a_home()
```

> **Optimización clave:** En el modo SORT, el robot va **directo de PICK a PLACE** sin pasar por HOME, reduciendo el tiempo de ciclo total.

---

## 5.5 Modo SORT-BUCLE

```python
# main.py — flujo_sort_bucle()
def flujo_sort_bucle(detector, robot):
    """Sort continuo — ordena, espera a que el operador desordene, y vuelve."""
    while True:
        cubos   = detector.get_cubos()
        a_mover = [cb for cb in cubos if not zona_ya_en_destino(cb)]

        if not a_mover:
            # Esperar cambios (polling cada 1.5 s)
            while True:
                time.sleep(1.5)
                cubos   = detector.get_cubos()
                a_mover = [cb for cb in cubos if not zona_ya_en_destino(cb)]
                if a_mover:
                    break   # detectó cubos fuera de lugar
        else:
            _nucleo_sort(detector, robot, a_mover)
```

---

## 5.6 Comunicación TCP con el UR3

```python
# robot_controller.py — Envío de comandos URScript

ROBOT_IP   = "192.168.1.74"
ROBOT_PORT = 30002

def _build_movej(joints_deg, vel=1.0, acel=0.3, tf=0.0):
    """Construye el comando movej para enviar al puerto 30002."""
    rad = [math.radians(q) for q in joints_deg]
    joints_str = ", ".join(f"{r:.6f}" for r in rad)
    if tf > 0:
        # t=tf fuerza la duración del movimiento (planificación quíntica)
        return f"movej([{joints_str}], a={acel}, v={vel}, t={tf:.4f})\n"
    return f"movej([{joints_str}], a={acel}, v={vel})\n"

def _enviar_script(script, esperar=0.5):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(10)
        s.connect((ROBOT_IP, ROBOT_PORT))
        s.sendall(script.encode("utf-8"))
    time.sleep(esperar)
```

---

## 5.7 Waypoint de Rodeo

```python
# robot_controller.py — Zona de barras laterales
BARRA_Y_FIN   = -280.0   # mm
BARRA_X_LIBRE =  350.0   # mm
WAYPOINT_RODEO_J = [0.0, -110.0, -90.0, -70.0, 90.0, 0.0]  # grados

def necesita_rodeo(x_mm, y_mm):
    """True si el cubo está en la zona de barras laterales."""
    return y_mm < BARRA_Y_FIN or abs(x_mm) > BARRA_X_LIBRE
```

La condición de rodeo es: $$Y < -280\,\text{mm}$$ O $$|X| > 350\,\text{mm}$$

---

[← Trayectorias](../04-cinematica-inversa/04c-trayectorias) &nbsp;&nbsp; [Visión Artificial →](../06-vision-artificial){: .btn .btn-outline }
