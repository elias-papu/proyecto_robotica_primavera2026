---
layout: default
title: Control Cinemático
nav_order: 4
description: "Jacobiano numérico, control proporcional y planificación quíntica real"
permalink: /03-control-cinematico/
---

# 3. Control Cinemático
{: .no_toc }

Jacobiano numérico del UR3, control proporcional de velocidad y planificación de trayectorias con polinomio quíntico.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 3.1 El Jacobiano en el Proyecto

El sistema usa el **Jacobiano numérico** en dos lugares:

1. **`ik_numerica.py`** — como fallback de la IK analítica (Levenberg-Marquardt)
2. **`trajectory_planner.py`** — para verificar y planificar trayectorias cartesianas

La relación entre velocidades articulares y cartesianas es:

$$\dot{\mathbf{x}} = J(\mathbf{q})\,\dot{\mathbf{q}}$$

donde $$J \in \mathbb{R}^{6\times6}$$ incluye tanto la parte de traslación (filas 1-3) como de rotación (filas 4-6).

---

## 3.2 Jacobiano Numérico — Código Real

Del archivo `ik_numerica.py` del proyecto:

```python
# ik_numerica.py — Jacobiano 6×6 por diferencias finitas
def _jacobiano(q):
    """Jacobiano 6×6 por diferencias finitas. dq = 1e-6 rad."""
    dq = 1e-6
    T0 = _cd(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    J  = np.zeros((6, 6))

    for i in range(6):
        q2    = list(q)
        q2[i] += dq
        T2    = _cd(q2)
        p2    = T2[:3, 3]
        R2    = T2[:3, :3]

        # Jacobiano de posición (primeras 3 filas)
        J[:3, i] = (p2 - p0) / dq

        # Jacobiano de orientación (últimas 3 filas) — axis-angle de dR = R2·R0ᵀ
        dR = R2 @ R0.T
        tr = np.trace(dR)
        if tr > 3 - 1e-6:
            w = np.zeros(3)
        else:
            angle = math.acos(np.clip((tr - 1) / 2, -1.0, 1.0))
            if abs(angle) < 1e-6:
                w = np.zeros(3)
            else:
                s  = 2 * math.sin(angle)
                wx = (dR[2,1] - dR[1,2]) / s
                wy = (dR[0,2] - dR[2,0]) / s
                wz = (dR[1,0] - dR[0,1]) / s
                w  = np.array([wx, wy, wz]) * angle / dq
        J[3:, i] = w

    return J
```

---

## 3.3 Levenberg-Marquardt como Control Iterativo

Del `ik_numerica.py` — la actualización LM con pesos diferenciados:

$$
\Delta\mathbf{q} = (J^T W J + \lambda I)^{-1} J^T W \mathbf{e}
$$

donde $$W = \text{diag}(10, 10, 10, 0.5, 0.5, 0.5)$$ prioriza la posición sobre la orientación.

```python
# ik_numerica.py — Loop de Levenberg-Marquardt (simplificado)
W   = np.diag([10.0, 10.0, 10.0, 0.5, 0.5, 0.5])  # posición >> orientación
lam = 0.05   # factor de amortiguamiento

for _ in range(max_iter):
    T   = _cd(q)
    e   = np.concatenate([p_target - T[:3,3],
                          error_orientacion(R_target, T[:3,:3])])

    if np.linalg.norm(e[:3]) < tol:
        break  # convergió en posición

    J    = _jacobiano(q)
    JtW  = J.T @ W
    delta = np.linalg.solve(JtW @ J + lam * np.eye(6), JtW @ e)
    q   += delta

    # Límites articulares UR3
    q = np.clip(q,
        [-3.14, -3.14, -1.57, -3.14, -3.14, -3.14],
        [ 3.14,  0.0,   1.57,  3.14,  3.14,  3.14])
```

---

## 3.4 Planificación de Trayectorias con Polinomio Quíntico

El módulo `trajectory_planner.py` implementa la generación de trayectorias suaves. Este es el código **real** del proyecto:

```python
# trajectory_planner.py — Polinomio quíntico real del proyecto
import numpy as np

def polinomio_quintico(x0: float, xf: float,
                       t_total: float = 4.0, n: int = 50):
    """
    Genera trayectoria suave de 5° grado entre x0 y xf.
    Condiciones: v(0)=v(tf)=0, a(0)=a(tf)=0.

    El sistema 6×6 se construye como:
      x(t)   = a5·t⁵ + a4·t⁴ + a3·t³ + a2·t² + a1·t + a0
      v(t)   = 5a5·t⁴ + 4a4·t³ + 3a3·t² + 2a2·t + a1
      a(t)   = 20a5·t³ + 12a4·t² + 6a3·t + 2a2
    """
    tf = t_total

    A = np.array([
        [0,       0,      0,      0,    0, 1],   # x(0)  = x0
        [tf**5,  tf**4,  tf**3,  tf**2, tf, 1],  # x(tf) = xf
        [0,       0,      0,      0,    1, 0],   # v(0)  = 0
        [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0], # v(tf) = 0
        [0,       0,      0,      2,    0, 0],   # a(0)  = 0
        [20*tf**3, 12*tf**2, 6*tf, 2,   0, 0],   # a(tf) = 0
    ])
    B = np.array([x0, xf, 0, 0, 0, 0])
    coeffs = np.linalg.solve(A, B)
    a5, a4, a3, a2, a1, a0 = coeffs

    t   = np.linspace(0, tf, n)
    pos = a5*t**5 + a4*t**4 + a3*t**3 + a2*t**2 + a1*t + a0
    vel = 5*a5*t**4 + 4*a4*t**3 + 3*a3*t**2 + 2*a2*t + a1
    acc = 20*a5*t**3 + 12*a4*t**2 + 6*a3*t + 2*a2

    return pos, vel, acc, t
```

---

## 3.5 Integración con `robot_controller.py` — Tiempo Quíntico

La planificación quíntica del tiempo de movimiento se integra directamente en `robot_controller.py`:

```python
# robot_controller.py — calcular_tf_quintico
def calcular_tf_quintico(joints_inicio_deg, joints_fin_deg,
                         vel_max_rad=VEL_J, acel_max_rad=ACEL_J):
    """
    Calcula el tiempo óptimo tf usando el polinomio quíntico.

    Para el polinomio quíntico con v0=vf=a0=af=0:
      v_max = (15/8) · |qf-q0| / tf
      a_max ≈ 10√3/3 · |qf-q0| / tf²

    Restricciones:
      tf ≥ (15/8) · |dq| / vel_max        (velocidad)
      tf ≥ sqrt(10√3/3 · |dq| / acel_max)  (aceleración)
    """
    tf_min = 0.5  # mínimo absoluto

    for q0_deg, qf_deg in zip(joints_inicio_deg, joints_fin_deg):
        dq = abs(math.radians(qf_deg - q0_deg))
        if dq < 1e-6:
            continue
        tf_vel  = (15.0 / 8.0) * dq / vel_max_rad
        tf_acel = math.sqrt((10.0 * math.sqrt(3.0) / 3.0) * dq / acel_max_rad)
        tf_min  = max(tf_min, tf_vel, tf_acel)

    return tf_min
```

El tiempo calculado se pasa directamente al comando `movej` como parámetro `t=tf`:

```python
# robot_controller.py — _move_j con planificación quíntica
def _move_j(self, joints_deg, lento=False, usar_quintico=False):
    vel  = VEL_LENTO if lento else VEL_J     # 0.25 o 1.0 rad/s
    acel = ACEL_LENTO if lento else ACEL_J   # 0.20 o 0.30 rad/s²

    if usar_quintico:
        tf = calcular_tf_quintico(self._pos_actual, joints_deg,
                                   vel_max_rad=vel, acel_max_rad=acel)
        t_espera = tf + 0.8   # margen de seguridad
        # movej([q1..q6], a=acel, v=vel, t=tf)  ← t fuerza duración
        _enviar_script(_build_movej(joints_deg, vel=vel, acel=acel, tf=tf),
                       esperar=t_espera)
    else:
        t_espera = _tiempo_movej(self._pos_actual, joints_deg, vel=vel)
        _enviar_script(_build_movej(joints_deg, vel=vel, acel=acel),
                       esperar=t_espera)
```

---

## 3.6 Comparación: Control Cinemático vs. IK Analítica

| Criterio | Control Cinemático (LM) | IK Analítica |
|---|:---:|:---:|
| **Velocidad** | 2–15 ms | < 1 ms |
| **Orientación** | Controlada (W ponderado) | Exacta |
| **Configuración** | Depende del punto inicial | Explícita (elbow/rama) |
| **Singularidades** | Amortiguamiento λ | Detección explícita |
| **Uso en el proyecto** | Fallback (estrategia 5) | Principal (estrategia 1) |

---

[← Cinemática Directa](../02-cinematica-directa) &nbsp;&nbsp; [Cinemática Inversa →](../04-cinematica-inversa){: .btn .btn-outline }
