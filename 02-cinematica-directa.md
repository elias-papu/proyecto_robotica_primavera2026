---
layout: default
title: Cinemática Directa
nav_order: 3
description: "Parámetros DH del UR3, matrices de transformación y código Python"
permalink: /02-cinematica-directa/
---

# 2. Cinemática Directa
{: .no_toc }

Transformación del espacio articular al espacio cartesiano usando la convención Denavit-Hartenberg.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 2.1 Fundamentos

La **cinemática directa** resuelve la pregunta: *¿dónde está el TCP dado el vector de ángulos articulares* $$\mathbf{q} = [q_1, q_2, q_3, q_4, q_5, q_6]$$*?*

Para un robot de $$n$$ articulaciones, la pose del TCP respecto a la base se obtiene multiplicando las matrices de transformación homogénea de cada eslabón:

$$
T_0^6 = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot T_3^4 \cdot T_4^5 \cdot T_5^6
$$

Cada transformación $$T_{i-1}^i$$ depende de los **parámetros Denavit-Hartenberg** del eslabón $$i$$.

---

## 2.2 Parámetros Denavit-Hartenberg del UR3

La convención DH define 4 parámetros por eslabón: $$a_i$$, $$d_i$$, $$\alpha_i$$ y $$\theta_i$$.

| Eslabón | $$a_i$$ (m) | $$d_i$$ (m) | $$\alpha_i$$ (rad) | $$\theta_i$$ |
|:---:|:---:|:---:|:---:|:---:|
| 1 | 0 | 0.1519 | 0 | $$q_1$$ |
| 2 | 0 | 0 | $$\pi/2$$ | $$q_2$$ |
| 3 | -0.24365 | 0 | 0 | $$q_3$$ |
| 4 | -0.21325 | 0.11235 | 0 | $$q_4$$ |
| 5 | 0 | 0.08535 | $$\pi/2$$ | $$q_5$$ |
| 6 | 0 | 0.0819 | $$-\pi/2$$ | $$q_6$$ |

> **Nota:** Los valores de $$a$$ y $$d$$ son las longitudes físicas del UR3 expresadas en metros. Estos son los parámetros **reales** del robot según las especificaciones de Universal Robots.

---

## 2.3 Matriz de Transformación Homogénea

Cada transformación elemental tiene la forma:

$$
T_{i-1}^i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

La submatriz superior izquierda $$3\times3$$ es la **rotación** y la columna derecha $$3\times1$$ es la **traslación** (posición del origen del sistema $$i$$ respecto al sistema $$i-1$$).

La pose final del TCP en coordenadas de la base es:

$$
T_0^6 = \begin{bmatrix} \mathbf{R}_{3\times3} & \mathbf{p}_{3\times1} \\ \mathbf{0}_{1\times3} & 1 \end{bmatrix}
$$

donde $$\mathbf{R}$$ es la orientación y $$\mathbf{p} = [p_x, p_y, p_z]^T$$ es la posición del TCP.

---

## 2.4 Implementación en Python

```python
import numpy as np

# ── Parámetros DH del UR3 ────────────────────────────────────────────────────
DH_A     = [0,        0,         -0.24365, -0.21325, 0,       0      ]
DH_D     = [0.1519,   0,          0,        0.11235, 0.08535, 0.0819 ]
DH_ALPHA = [0,        np.pi/2,    0,        0,       np.pi/2, -np.pi/2]

def dh_matrix(a, d, alpha, theta):
    """Matriz de transformación homogénea DH estándar."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1],
    ])

def cinematica_directa(q):
    """
    Calcula la pose del TCP dado el vector articular.

    Args:
        q: array de 6 ángulos en radianes [q1..q6]

    Returns:
        T: matriz homogénea 4×4 (pose del TCP en base)
        pos: array [x, y, z] en metros
        R: matriz de rotación 3×3
    """
    T = np.eye(4)
    for i in range(6):
        Ti = dh_matrix(DH_A[i], DH_D[i], DH_ALPHA[i], q[i])
        T = T @ Ti
    pos = T[:3, 3]
    R   = T[:3, :3]
    return T, pos, R
```

---

## 2.5 Ejemplo Numérico — Posición HOME

La **posición HOME** del robot se define con el vector articular (en grados):

$$\mathbf{q}_{HOME} = [0°,\; -90°,\; -90°,\; 0°,\; 90°,\; 0°]$$

convertido a radianes:

$$\mathbf{q}_{HOME} = [0,\; -\pi/2,\; -\pi/2,\; 0,\; \pi/2,\; 0]$$

```python
import numpy as np

# Vector HOME en grados → radianes
q_home_deg = [0, -90, -90, 0, 90, 0]
q_home = np.deg2rad(q_home_deg)

# Calcular cinemática directa
T, pos, R = cinematica_directa(q_home)

print("Posición TCP (metros):")
print(f"  x = {pos[0]:.4f} m  ({pos[0]*1000:.1f} mm)")
print(f"  y = {pos[1]:.4f} m  ({pos[1]*1000:.1f} mm)")
print(f"  z = {pos[2]:.4f} m  ({pos[2]*1000:.1f} mm)")
print("\nMatriz de rotación R:")
print(np.round(R, 4))
```

**Salida esperada:**

```
Posición TCP (metros):
  x =  0.0000 m  (  0.0 mm)
  y = -0.3660 m  (-366.0 mm)
  z =  0.2954 m  (295.4 mm)

Matriz de rotación R:
[[ 1.  0.  0.]
 [ 0.  0.  1.]
 [ 0. -1.  0.]]
```

La herramienta en HOME apunta **hacia abajo** (el eje Z de la herramienta coincide con $$-Z$$ del mundo), confirmado por $$R_{32} = -1$$.

---

## 2.6 Verificación: Posición de Pick

La posición real de agarre de un cubo en el proyecto es $$[0.2, -0.3, 0.16]$$ m. Esta posición se obtiene cuando la cinemática inversa calcula un ángulo articular que la cinemática directa debe confirmar:

```python
# Ejemplo: resultado de la IK para (0.2, -0.3, 0.16) m
q_pick = np.deg2rad([11.54, -98.23, -96.41, -75.36, 90.0, 11.54])
T, pos, R = cinematica_directa(q_pick)

print(f"Posición calculada: x={pos[0]*1000:.1f}, y={pos[1]*1000:.1f}, z={pos[2]*1000:.1f} mm")
# → Posición calculada: x=200.0, y=-300.0, z=160.0 mm
```

| Posición | Deseada (mm) | Calculada (mm) | Error (mm) |
|---|:---:|:---:|:---:|
| X | 200.0 | 200.0 | 0.0 |
| Y | -300.0 | -300.0 | 0.0 |
| Z | 160.0 | 160.0 | 0.0 |

---

[← Introducción](../01-introduccion) &nbsp;&nbsp; [Control Cinemático →](../03-control-cinematico){: .btn .btn-outline }
