---
layout: default
title: Implementación Industrial
nav_order: 6
description: "Sistema completo de pick and place: flujo, modos GO y SORT, comunicación TCP y gripper"
permalink: /05-implementacion-industrial/
---

# 5. Implementación Industrial
{: .no_toc }

Sistema completo de pick and place: arquitectura, flujos de operación, comunicación con el robot y detección de singularidades.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 5.1 Descripción del Sistema

El sistema de pick and place integra todos los módulos en un flujo de producción autónomo:

- **Visión:** Detecta y localiza cubos de colores en tiempo real (30 fps)
- **Calibración:** Calcula la homografía cámara-robot con ArUco al inicio de sesión
- **Planificación:** Calcula IK analítica + trayectorias quínticas para cada movimiento
- **Ejecución:** Envía comandos TCP al UR3 y controla el gripper OnRobot
- **Supervisión:** Valida el workspace, detecta singularidades y gestiona el modo SORT

---

## 5.2 Flujo de Operación Completo

```
┌─────────────────────────────────────────────────────────┐
│                    ARRANQUE DEL SISTEMA                  │
│  1. Inicializar cámara (Logitech C920)                  │
│  2. Conectar socket TCP → UR3:30002                     │
│  3. Conectar HTTP → OnRobot Gripper                     │
└────────────────────────┬────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────┐
│              CALIBRACIÓN ArUco                           │
│  4. Detectar 4 marcadores en esquinas del área         │
│  5. Calcular homografía pixel → robot (RANSAC)         │
│  6. Guardar en caché JSON para reusar                  │
└────────────────────────┬────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────┐
│              DETECCIÓN DE CUBOS                          │
│  7. Capturar frame → convertir a HSV                   │
│  8. Segmentar por color (rojo/verde/azul)               │
│  9. Filtrar por área, solidez, aspecto, vértices        │
│ 10. Calcular centroide → homografía → coords robot      │
└────────────────────────┬────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────┐
│           PLANIFICACIÓN Y EJECUCIÓN                      │
│ 11. Seleccionar cubo (modo GO: usuario; SORT: auto)    │
│ 12. Calcular IK analítica para posición del cubo        │
│ 13. Validar workspace (X: -500..500, Y: -600..100 mm)  │
│ 14. Detectar zona barras → waypoint de rodeo si aplica  │
│ 15. PICK en 3 fases (PREMOVE → APPROACH → PICK)         │
│ 16. GRIP (OnRobot HTTP)                                 │
│ 17. PLACE → zona de color destino                       │
│ 18. RELEASE (OnRobot HTTP)                              │
│ 19. Retorno a HOME                                      │
└─────────────────────────────────────────────────────────┘
```

---

## 5.3 Modo GO y Modo SORT

### Modo GO — Clasificar un cubo

El operador selecciona un cubo detectado (por color o posición) y el sistema ejecuta un solo ciclo de pick and place:

```python
def modo_go(color_objetivo):
    """Clasifica el cubo del color especificado."""
    cubo = camera_detector.obtener_cubo(color_objetivo)
    if cubo is None:
        print(f"No se detectó cubo {color_objetivo}")
        return

    zona_destino = zone_manager.zona_de_color(color_objetivo)
    robot_controller.pick_and_place(cubo.posicion, zona_destino)
    print(f"Cubo {color_objetivo} clasificado en {zona_destino}")
```

### Modo SORT — Clasificación autónoma completa

El sistema detecta **todos los cubos mal colocados** y los clasifica sin intervención humana:

```python
def modo_sort():
    """
    Clasifica todos los cubos que están fuera de su zona correcta.
    Ejecuta sin intervención humana hasta que todos estén clasificados.
    """
    while True:
        cubos_mal_colocados = zone_manager.detectar_mal_colocados()

        if not cubos_mal_colocados:
            print("✓ Todos los cubos están en su zona correcta.")
            break

        for cubo in cubos_mal_colocados:
            zona_correcta = zone_manager.zona_de_color(cubo.color)
            robot_controller.pick_and_place(cubo.posicion, zona_correcta)
            print(f"→ Cubo {cubo.color} en {cubo.zona_actual} → {zona_correcta}")

        # Pausa breve para actualizar visión
        time.sleep(0.5)
```

---

## 5.4 Comunicación TCP con el UR3 — Puerto 30002

El UR3 acepta comandos **URScript** a través de una conexión TCP directa al puerto 30002. No se requiere ningún software adicional.

### Formato del Comando `movej`

```
movej([q1, q2, q3, q4, q5, q6], a=1.4, v=1.05, t=0, r=0)
```

| Parámetro | Descripción | Valor típico |
|---|---|---|
| `[q1..q6]` | Vector articular en radianes | Calculado por IK |
| `a` | Aceleración articular (rad/s²) | 1.4 (lento) – 8.0 (rápido) |
| `v` | Velocidad articular (rad/s) | 1.05 (lento) – 3.14 (rápido) |
| `t` | Tiempo de ejecución (0 = ignorar) | 0 |
| `r` | Radio de blend (m) | 0 (sin blend) |

### Implementación

```python
import socket

class RobotController:
    def __init__(self, robot_ip="192.168.1.10", port=30002):
        self.robot_ip = robot_ip
        self.port = port
        self.sock = None

    def conectar(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.robot_ip, self.port))
        self.sock.settimeout(5.0)
        print(f"✓ Conectado al UR3 en {self.robot_ip}:{self.port}")

    def mover_articular(self, q, aceleracion=1.4, velocidad=1.05):
        """Envía comando movej con el vector articular dado."""
        q_str = ", ".join(f"{angle:.6f}" for angle in q)
        cmd = f"movej([{q_str}], a={aceleracion}, v={velocidad})\n"
        self.sock.send(cmd.encode("utf-8"))

    def ir_home(self):
        """Envía el robot a la posición HOME."""
        q_home = [0, -1.5708, -1.5708, 0, 1.5708, 0]  # radianes
        self.mover_articular(q_home, aceleracion=2.0, velocidad=1.5)
```

---

## 5.5 Control del Gripper OnRobot vía HTTP REST

```python
import requests
import time

class OnRobotGripper:
    def __init__(self, ip="192.168.1.1"):
        self.base_url = f"http://{ip}/api/v1/gripper"

    def grip(self, fuerza=50):
        """Activa el gripper con la fuerza especificada (%)."""
        payload = {"force": fuerza}
        r = requests.post(f"{self.base_url}/grip", json=payload, timeout=2)
        time.sleep(0.3)  # Esperar que el gripper cierre
        return r.status_code == 200

    def release(self):
        """Libera el gripper."""
        r = requests.post(f"{self.base_url}/release", timeout=2)
        time.sleep(0.3)  # Esperar que el gripper abra
        return r.status_code == 200
```

---

## 5.6 Waypoint de Rodeo para Zona de Barras

Cuando el cubo se encuentra en la **zona de barras laterales**, el robot debe rodear las barras físicas antes de descender. Se detecta esta condición y se inserta un waypoint intermedio:

**Condición de rodeo:**
$$Y < -280\,\text{mm} \quad \text{OR} \quad X > 350\,\text{mm}$$

**Waypoint de rodeo:** $$\mathbf{q}_{rodeo} = [0°, -110°, -90°, -70°, 90°, 0°]$$

```python
def necesita_rodeo(x_mm, y_mm):
    """Determina si el cubo está en la zona de barras laterales."""
    return y_mm < -280 or x_mm > 350

def pick_con_rodeo(x_cubo, y_cubo, z_cubo):
    """
    Ejecuta el pick pasando por el waypoint de rodeo si es necesario.
    """
    if necesita_rodeo(x_cubo, y_cubo):
        print("→ Zona de barras detectada. Insertando waypoint de rodeo.")
        q_rodeo = np.deg2rad([0, -110, -90, -70, 90, 0])
        robot.mover_articular(q_rodeo, aceleracion=1.0, velocidad=0.8)
        time.sleep(1.0)  # Esperar que llegue

    # Continuar con las 3 fases normales de pick
    ejecutar_fases_pick(x_cubo, y_cubo, z_cubo)
```

---

## 5.7 Validación del Workspace

Antes de ejecutar cualquier movimiento, se validan los límites del área de trabajo:

```python
WORKSPACE = {
    "x_min": -500, "x_max": 500,   # mm
    "y_min": -600, "y_max": 100,   # mm
    "z_min":  100, "z_max": 350,   # mm
}

def validar_workspace(x, y, z):
    """Verifica que la posición esté dentro del workspace seguro."""
    valido = (WORKSPACE["x_min"] <= x <= WORKSPACE["x_max"] and
              WORKSPACE["y_min"] <= y <= WORKSPACE["y_max"] and
              WORKSPACE["z_min"] <= z <= WORKSPACE["z_max"])
    if not valido:
        raise ValueError(
            f"Posición ({x:.0f}, {y:.0f}, {z:.0f}) mm "
            f"fuera del workspace seguro."
        )
    return True
```

---

## 5.8 Diagrama de Flujo Completo

```
INICIO
  │
  ├── Inicializar hardware (cámara, TCP, gripper)
  │
  ├── Calibración ArUco (si no hay caché)
  │       │
  │       └── Detectar 4 marcadores → calcular homografía → guardar JSON
  │
  ├── Menú principal
  │       │
  │       ├── [1] Modo GO
  │       │       │
  │       │       └── Seleccionar color → Detectar cubo →
  │       │           Calcular IK → Validar workspace →
  │       │           ¿Zona barras? → [Sí] Waypoint rodeo
  │       │           PREMOVE → APPROACH → PICK → GRIP →
  │       │           PLACE → RELEASE → HOME
  │       │
  │       ├── [2] Modo SORT
  │       │       │
  │       │       └── Detectar todos los mal colocados →
  │       │           Para cada cubo: ejecutar ciclo GO →
  │       │           Repetir hasta que todos estén correctos
  │       │
  │       ├── [3] Recalibrar homografía
  │       │
  │       └── [0] Salir → HOME → Desconectar

FIN
```

---

[← Trayectorias](../04-cinematica-inversa/04c-trayectorias) &nbsp;&nbsp; [Visión Artificial →](../06-vision-artificial){: .btn .btn-outline }
