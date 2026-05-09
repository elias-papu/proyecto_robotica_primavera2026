---
layout: default
title: Planificación de Trayectorias
parent: Cinemática Inversa
nav_order: 3
description: "Polinomio quíntico y perfil trapezoidal para las fases de pick and place"
permalink: /04-cinematica-inversa/04c-trayectorias/
---

# 4c. Planificación de Trayectorias
{: .no_toc }

Polinomios quínticos y perfil trapezoidal de velocidad para movimientos suaves del UR3.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4c.1 Polinomio Quíntico

### Motivación

Un **polinomio quíntico** (grado 5) permite especificar **6 condiciones de frontera**: posición, velocidad y aceleración en $$t_0$$ y $$t_f$$. Esto garantiza transiciones suaves (sin saltos de aceleración) en ambos extremos, esencial para no excitar vibraciones mecánicas ni sobrepasar los límites de torque.

### Condiciones de Frontera

En $$t_0 = 0$$:
$$s(0) = s_0, \quad \dot{s}(0) = v_0, \quad \ddot{s}(0) = a_0$$

En $$t_f$$:
$$s(t_f) = s_f, \quad \dot{s}(t_f) = v_f, \quad \ddot{s}(t_f) = a_f$$

Para trayectorias de inicio y fin desde reposo: $$v_0 = v_f = a_0 = a_f = 0$$.

### Sistema Matricial

El polinomio $$s(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5$$ satisface:

$$
\mathbf{A}\,\mathbf{c} = \mathbf{b}
$$

$$
\mathbf{A} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 2 & 0 & 0 & 0 \\
1 & t_f & t_f^2 & t_f^3 & t_f^4 & t_f^5 \\
0 & 1 & 2t_f & 3t_f^2 & 4t_f^3 & 5t_f^4 \\
0 & 0 & 2 & 6t_f & 12t_f^2 & 20t_f^3
\end{bmatrix}
, \quad
\mathbf{b} = \begin{bmatrix} s_0 \\ v_0 \\ a_0 \\ s_f \\ v_f \\ a_f \end{bmatrix}
$$

---

## 4c.2 Código Python — `trayectorias.py`

```python
"""
trayectorias.py — Generación de trayectorias para el CoBot Clasificador.
Módulo del proyecto de Elias Santiago Jiménez Hernández.
Control Avanzado de Robots 2025, IBERO.
"""
import numpy as np
import matplotlib.pyplot as plt


def coeficientes_quintico(s0, sf, v0=0, vf=0, a0=0, af=0, tf=10.0):
    """
    Calcula los coeficientes del polinomio quíntico.

    Args:
        s0, sf:    posición inicial y final
        v0, vf:    velocidad inicial y final
        a0, af:    aceleración inicial y final
        tf:        tiempo final (s)

    Returns:
        c: array de 6 coeficientes [c0..c5]
    """
    A = np.array([
        [1,   0,      0,         0,          0,           0        ],
        [0,   1,      0,         0,          0,           0        ],
        [0,   0,      2,         0,          0,           0        ],
        [1,   tf,     tf**2,     tf**3,      tf**4,       tf**5    ],
        [0,   1,      2*tf,      3*tf**2,    4*tf**3,     5*tf**4  ],
        [0,   0,      2,         6*tf,       12*tf**2,    20*tf**3 ],
    ])
    b = np.array([s0, v0, a0, sf, vf, af])
    return np.linalg.solve(A, b)


def evaluar_quintico(c, t_vec):
    """Evalúa posición, velocidad y aceleración del polinomio quíntico."""
    pos  = np.polyval(c[::-1], t_vec)
    vel  = np.polyval(np.polyder(c[::-1]), t_vec)
    acel = np.polyval(np.polyder(c[::-1], 2), t_vec)
    return pos, vel, acel


def trayectoria_xy(tf=10.0, dt=0.05):
    """
    Trayectorias en X e Y para la demostración del sistema.
    X: 0.25 m → 0.30 m
    Y: 0.10 m → 0.15 m
    Condiciones de frontera: velocidades y aceleraciones = 0.
    """
    t_vec = np.arange(0, tf + dt, dt)

    # Coeficientes para X
    cx = coeficientes_quintico(s0=0.25, sf=0.30, tf=tf)
    # Coeficientes para Y
    cy = coeficientes_quintico(s0=0.10, sf=0.15, tf=tf)

    pos_x,  vel_x,  acel_x  = evaluar_quintico(cx, t_vec)
    pos_y,  vel_y,  acel_y  = evaluar_quintico(cy, t_vec)

    return t_vec, pos_x, vel_x, acel_x, pos_y, vel_y, acel_y


def graficar_trayectorias_xy():
    """Grafica posición, velocidad y aceleración para X e Y."""
    t, px, vx, ax_tray, py, vy, ay_tray = trayectoria_xy()

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Trayectorias Quínticas — X e Y\n'
                 'CoBot Clasificador de Colores', fontsize=13)

    for row, (pos, vel, acel, eje, color) in enumerate([
        (px, vx, ax_tray, 'X', 'royalblue'),
        (py, vy, ay_tray, 'Y', 'tomato'),
    ]):
        axes[row, 0].plot(t, pos * 1000, color=color, linewidth=2)
        axes[row, 0].set_title(f'Posición {eje} (mm)')
        axes[row, 0].set_xlabel('t (s)')
        axes[row, 0].grid(True, alpha=0.3)

        axes[row, 1].plot(t, vel * 1000, color=color, linewidth=2)
        axes[row, 1].set_title(f'Velocidad {eje} (mm/s)')
        axes[row, 1].set_xlabel('t (s)')
        axes[row, 1].grid(True, alpha=0.3)

        axes[row, 2].plot(t, acel * 1000, color=color, linewidth=2)
        axes[row, 2].set_title(f'Aceleración {eje} (mm/s²)')
        axes[row, 2].set_xlabel('t (s)')
        axes[row, 2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('assets/img/trayectorias_xy.png', dpi=150)
    plt.show()
```

---

## 4c.3 Tres Fases del Pick and Place

El movimiento de pick se divide en tres fases conectadas por polinomios quínticos:

| Fase | Nombre | Altura Z (mm) | Descripción |
|:---:|---|:---:|---|
| 1 | **PREMOVE** | 240 | Posición de aproximación alta (evita colisiones) |
| 2 | **APPROACH** | 210 | Posición de pre-agarre (verifica alineación) |
| 3 | **PICK** | 160 | Posición de contacto y agarre |

```python
def fases_pick(x_cubo, y_cubo, tf_fase=2.0, dt=0.01):
    """
    Genera las trayectorias Z para las 3 fases de pick.
    Las posiciones X e Y se mantienen constantes (ya se alcanzaron antes).

    Retorna: lista de (t, z) para cada fase
    """
    z_premove  = 0.240  # m
    z_approach = 0.210  # m
    z_pick     = 0.160  # m

    fases = [
        (z_premove,  z_approach, "PREMOVE → APPROACH"),
        (z_approach, z_pick,     "APPROACH → PICK"),
    ]

    resultados = []
    for z_ini, z_fin, nombre in fases:
        c = coeficientes_quintico(s0=z_ini, sf=z_fin, tf=tf_fase)
        t_vec = np.arange(0, tf_fase + dt, dt)
        pos, vel, acel = evaluar_quintico(c, t_vec)
        resultados.append((t_vec, pos, vel, acel, nombre))

    return resultados
```

---

## 4c.4 Trayectoria Trapezoidal de Velocidad

### Descripción

El **perfil trapezoidal** divide el movimiento en tres fases:

1. **Aceleración constante** $$a_{max}$$: de 0 a $$v_{max}$$
2. **Velocidad constante** $$v_{max}$$: crucero
3. **Deceleración constante** $$-a_{max}$$: de $$v_{max}$$ a 0

```
 v ↑
   │      ┌───────┐
v_max     │       │
   │   ╱  │       │  ╲
   │ ╱    │       │    ╲
   └──────────────────── t
     t_a   t_c    t_d
```

### Implementación

```python
def trayectoria_trapezoidal(s0, sf, v_max, a_max, dt=0.01):
    """
    Perfil trapezoidal de velocidad.

    Args:
        s0, sf:  posición inicial y final (m)
        v_max:   velocidad máxima (m/s)
        a_max:   aceleración máxima (m/s²)
        dt:      paso de tiempo (s)

    Returns:
        t_vec, pos, vel, acel
    """
    d = abs(sf - s0)
    sign = np.sign(sf - s0)

    # Tiempo de aceleración/deceleración
    t_a = v_max / a_max

    # Distancia durante aceleración
    d_a = 0.5 * a_max * t_a**2

    if 2 * d_a > d:
        # Perfil triangular (sin fase de crucero)
        t_a = np.sqrt(d / a_max)
        t_c = 0
        v_max = a_max * t_a
    else:
        t_c = (d - 2 * d_a) / v_max

    t_total = 2 * t_a + t_c
    t_vec = np.arange(0, t_total + dt, dt)
    pos = np.zeros_like(t_vec)
    vel = np.zeros_like(t_vec)
    acel = np.zeros_like(t_vec)

    for k, t in enumerate(t_vec):
        if t <= t_a:                        # Aceleración
            a = sign * a_max
            v = sign * a_max * t
            s = s0 + 0.5 * sign * a_max * t**2
        elif t <= t_a + t_c:               # Crucero
            a = 0
            v = sign * v_max
            s = s0 + sign * (d_a + v_max * (t - t_a))
        else:                              # Deceleración
            dt_ = t - t_a - t_c
            a = -sign * a_max
            v = sign * (v_max - a_max * dt_)
            s = s0 + sign * (d - 0.5 * a_max * (t_total - t)**2)
        pos[k], vel[k], acel[k] = s, v, a

    return t_vec, pos, vel, acel
```

---

## 4c.5 Comparativa: Quíntico vs. Trapezoidal

Aplicados al mismo movimiento: APPROACH → PICK (Z: 210 mm → 160 mm).

```python
# Quíntico
tf = 2.0
c = coeficientes_quintico(s0=0.210, sf=0.160, tf=tf)
t_q = np.arange(0, tf + 0.01, 0.01)
pos_q, vel_q, acel_q = evaluar_quintico(c, t_q)

# Trapezoidal
t_t, pos_t, vel_t, acel_t = trayectoria_trapezoidal(
    s0=0.210, sf=0.160, v_max=0.05, a_max=0.08
)

# Comparativa
fig, axes = plt.subplots(1, 3, figsize=(14, 4))
fig.suptitle('Quíntico vs Trapezoidal — APPROACH → PICK (Z)', fontsize=12)

axes[0].plot(t_q, pos_q*1000, 'b-', label='Quíntico', lw=2)
axes[0].plot(t_t, pos_t*1000, 'r--', label='Trapezoidal', lw=2)
axes[0].set_title('Posición Z (mm)')
axes[0].legend(); axes[0].grid(True, alpha=0.3)

axes[1].plot(t_q, vel_q*1000, 'b-', label='Quíntico', lw=2)
axes[1].plot(t_t, vel_t*1000, 'r--', label='Trapezoidal', lw=2)
axes[1].set_title('Velocidad Z (mm/s)')
axes[1].legend(); axes[1].grid(True, alpha=0.3)

axes[2].plot(t_q, acel_q*1000, 'b-', label='Quíntico', lw=2)
axes[2].plot(t_t, acel_t*1000, 'r--', label='Trapezoidal', lw=2)
axes[2].set_title('Aceleración Z (mm/s²)')
axes[2].legend(); axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('assets/img/comparativa_trayectorias.png', dpi=150)
```

![Comparativa quíntico vs trapezoidal](../../assets/img/comparativa_trayectorias.png)

### Tabla de Comparación Directa

| Criterio | Quíntico | Trapezoidal |
|---|:---:|:---:|
| **Suavidad** | ✅ Continua hasta $$\ddot{s}$$ | 🔶 Discontinuidades en aceleración |
| **Aceleración máxima** | 🔶 Mayor (pico en curva) | ✅ Constante y controlada |
| **Tiempo de ciclo** | 🔶 Mayor (tf fijo) | ✅ Menor (llega a v_max antes) |
| **Jerk (∂³s/∂t³)** | ✅ Continuo | ❌ Impulsos en t_a, t_d |
| **Parámetros** | tf fijo | v_max, a_max libres |
| **Aplicación ideal** | Movimientos de precisión | Movimientos rápidos de transporte |

> **Uso en el proyecto:** Se emplean **polinomios quínticos** para las fases PREMOVE→APPROACH y APPROACH→PICK por su mayor suavidad, mientras que el perfil trapezoidal se usa para movimientos rápidos de transporte entre zonas de clasificación.

---

[← Levenberg-Marquardt](../04b-levenberg-marquardt) &nbsp;&nbsp; [Implementación Industrial →](../../05-implementacion-industrial){: .btn .btn-outline }
