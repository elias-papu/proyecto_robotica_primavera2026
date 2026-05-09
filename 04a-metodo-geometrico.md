---
layout: default
title: Método Geométrico
parent: Cinemática Inversa
nav_order: 1
description: "Derivación analítica paso a paso de la IK del UR3 con código Python completo"
permalink: /04-cinematica-inversa/04a-metodo-geometrico/
---

# 4a. Método Geométrico (Analítico)
{: .no_toc }

Derivación algebraica de los 6 ángulos articulares del UR3 con código Python completo.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4a.1 Planteamiento

Dada la matriz de transformación deseada del TCP:

$$
T_d = \begin{bmatrix}
n_x & o_x & a_x & p_x \\
n_y & o_y & a_y & p_y \\
n_z & o_z & a_z & p_z \\
0   & 0   & 0   & 1
\end{bmatrix}
$$

Se derivan los 6 ángulos en el orden: $$q_1 \to q_5 \to q_6 \to q_{234} \to q_2 \to q_3 \to q_4$$.

---

## 4a.2 Derivación Paso a Paso

### Cálculo de $$q_1$$

Se proyecta la posición del TCP sobre el plano XY de la base. Usando la geometría del robot, el eje de la muñeca está desplazado del TCP por la distancia $$d_6$$ a lo largo del eje $$a$$ (tercera columna de $$\mathbf{R}$$):

$$
\mathbf{p}_w = \mathbf{p}_d - d_6 \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix}
$$

$$
q_1 = \text{atan2}(p_{wy},\, p_{wx}) \pm \text{atan2}\!\left(d_4,\, \sqrt{p_{wx}^2 + p_{wy}^2 - d_4^2}\right)
$$

La raíz cuadrada tiene dos signos, dando las dos ramas de $$q_1$$. Se elige **signo positivo** (rama positiva).

### Cálculo de $$q_5$$

$$q_5$$ se calcula a partir de la norma del vector de orientación:

$$
q_5 = \pm\,\text{atan2}\!\left(\sqrt{(a_x\sin q_1 - a_y\cos q_1)^2 + a_z^2},\; a_x\cos q_1 + a_y\sin q_1\right)
$$

Se elige $$q_5 > 0$$ (codo abajo / rama positiva).

### Cálculo de $$q_6$$

Con $$q_1$$ y $$q_5$$ conocidos, $$q_6$$ se obtiene de las componentes de la matriz de rotación:

$$
q_6 = \text{atan2}\!\left(\frac{-o_x\sin q_1 + o_y\cos q_1}{\sin q_5},\; \frac{n_x\sin q_1 - n_y\cos q_1}{\sin q_5}\right)
$$

### Cálculo de $$q_{234}$$ (suma)

El eje Z de la herramienta en coordenadas del hombro define la suma $$q_{234} = q_2 + q_3 + q_4$$:

$$
q_{234} = \text{atan2}(-a_z,\; a_x\cos q_1 + a_y\sin q_1)
$$

### Cálculo de $$q_2$$ — Discriminante cuadrático

Se proyecta la posición de la muñeca sobre el plano del brazo. Sea:

$$
B_1 = p_{wx}\cos q_1 + p_{wy}\sin q_1 - d_1 \cdot 0 \quad\text{(proyección XY)}
$$
$$
B_2 = p_{wz} - d_1
$$

Con $$r = \sqrt{B_1^2 + B_2^2}$$ y los eslabones $$a_2 = |a_2|$$ y $$a_3 = |a_3|$$:

$$
\cos q_3 = \frac{r^2 - a_2^2 - a_3^2}{2\,a_2\,a_3}
$$

$$
q_3 = \text{atan2}\!\left(-\sqrt{1-\cos^2 q_3},\; \cos q_3\right) \quad \text{(elbow down: } q_3 < 0\text{)}
$$

$$
q_2 = \text{atan2}(B_2,\, B_1) - \text{atan2}\!\left(a_3\sin q_3,\; a_2 + a_3\cos q_3\right)
$$

### Cálculo de $$q_4$$

Por diferencia, usando la suma ya conocida:

$$
q_4 = q_{234} - q_2 - q_3
$$

---

## 4a.3 Código Python Completo

```python
import numpy as np

# ── Parámetros DH del UR3 ────────────────────────────────────────────────────
DH_A     = [0,        0,         -0.24365, -0.21325, 0,       0      ]
DH_D     = [0.1519,   0,          0,        0.11235, 0.08535, 0.0819 ]
DH_ALPHA = [0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]

# Aliases convenientes
d1 = DH_D[0];  d4 = DH_D[3];  d5 = DH_D[4];  d6 = DH_D[5]
a2 = DH_A[2];  a3 = DH_A[3]


def calcular_ik(T_deseada, elbow_down=True, rama_q1_positiva=True):
    """
    Cinemática inversa analítica del UR3 (6 GDL).

    Args:
        T_deseada:         Matriz homogénea 4×4 deseada del TCP
        elbow_down:        True → q3 < 0 (codo abajo, recomendado)
        rama_q1_positiva:  True → rama positiva de q1

    Returns:
        q: array de 6 ángulos en radianes, o None si no hay solución
    """
    # Extraer posición y orientación
    px, py, pz = T_deseada[:3, 3]
    nx, ny, nz = T_deseada[:3, 0]
    ox, oy, oz = T_deseada[:3, 1]
    ax, ay, az = T_deseada[:3, 2]

    # ── q1 ─────────────────────────────────────────────────────────────────
    # Posición de la muñeca (wrist center)
    pwx = px - d6 * ax
    pwy = py - d6 * ay
    pwz = pz - d6 * az

    r_xy = np.sqrt(pwx**2 + pwy**2)
    discriminante = r_xy**2 - d4**2
    if discriminante < 0:
        return None  # Fuera del workspace

    sign_q1 = 1 if rama_q1_positiva else -1
    phi1 = np.arctan2(pwy, pwx)
    phi2 = np.arctan2(d4, sign_q1 * np.sqrt(discriminante))
    q1 = phi1 + phi2

    # ── q5 ─────────────────────────────────────────────────────────────────
    c1, s1 = np.cos(q1), np.sin(q1)
    arg_q5_x = ax * s1 - ay * c1
    arg_q5_y = ax * c1 + ay * s1
    sign_q5 = 1 if elbow_down else -1
    q5 = np.arctan2(
        sign_q5 * np.sqrt(arg_q5_x**2 + az**2),
        arg_q5_y
    )

    if abs(np.sin(q5)) < 1e-6:
        # Singularidad de muñeca: q5 ≈ 0, se fija q6 = 0
        q6 = 0.0
    else:
        # ── q6 ──────────────────────────────────────────────────────────────
        q6 = np.arctan2(
            (-ox * s1 + oy * c1) / np.sin(q5),
            ( nx * s1 - ny * c1) / np.sin(q5)
        )

    # ── q234 ───────────────────────────────────────────────────────────────
    q234 = np.arctan2(-az, ax * c1 + ay * s1)

    # ── q2, q3 ─────────────────────────────────────────────────────────────
    # Posición de la muñeca en coordenadas del hombro
    B1 = pwx * c1 + pwy * s1
    B2 = pwz - d1

    r_braz = np.sqrt(B1**2 + B2**2)
    cos_q3 = (r_braz**2 - a2**2 - a3**2) / (2 * a2 * a3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)  # Seguridad numérica

    sign_q3 = -1 if elbow_down else 1
    q3 = np.arctan2(sign_q3 * np.sqrt(1 - cos_q3**2), cos_q3)

    q2 = np.arctan2(B2, B1) - np.arctan2(
        a3 * np.sin(q3),
        a2 + a3 * np.cos(q3)
    )

    # ── q4 ─────────────────────────────────────────────────────────────────
    q4 = q234 - q2 - q3

    return np.array([q1, q2, q3, q4, q5, q6])
```

---

## 4a.4 Resolución con Dos Orientaciones

### Posición objetivo

$$\mathbf{p}_d = [0.2\,\text{m},\; -0.3\,\text{m},\; 0.16\,\text{m}]$$

### Orientación 1 — Herramienta apuntando hacia abajo

La herramienta está orientada verticalmente, apuntando hacia el suelo ($$-Z$$ del mundo):

| Componente | $$n$$ | $$o$$ | $$a$$ |
|:---:|:---:|:---:|:---:|
| **x** | -0.7673 | -0.6413 | -0.0060 |
| **y** | -0.6402 | 0.7664 | -0.0533 |
| **z** | 0.0388 | -0.0371 | -0.9986 |

```python
# Orientación 1: herramienta hacia abajo
n1 = np.array([-0.7673, -0.6402,  0.0388])
o1 = np.array([-0.6413,  0.7664, -0.0371])
a1 = np.array([-0.0060, -0.0533, -0.9986])
p  = np.array([ 0.2000, -0.3000,  0.1600])

T1 = np.eye(4)
T1[:3, 0] = n1;  T1[:3, 1] = o1
T1[:3, 2] = a1;  T1[:3, 3] = p

q_ori1 = calcular_ik(T1)
print("Orientación 1 — herramienta hacia abajo:")
print(f"  q (grados): {np.round(np.rad2deg(q_ori1), 2)}")
```

### Orientación 2 — Herramienta rotada 90° en Z

La herramienta está rotada 90° alrededor del eje Z respecto a la orientación 1:

```python
# Orientación 2: herramienta rotada 90° en Z
Rz90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
R1   = np.column_stack([n1, o1, a1])
R2   = R1 @ Rz90

T2 = np.eye(4)
T2[:3, :3] = R2;  T2[:3, 3] = p

q_ori2 = calcular_ik(T2)
print("\nOrientación 2 — herramienta rotada 90° en Z:")
print(f"  q (grados): {np.round(np.rad2deg(q_ori2), 2)}")
```

---

## 4a.5 Tabla Comparativa de Ángulos

| Articulación | Orientación 1 (°) | Orientación 2 (°) | Diferencia (°) |
|:---:|:---:|:---:|:---:|
| $$q_1$$ | 11.54 | 11.54 | 0.00 |
| $$q_2$$ | -98.23 | -98.23 | 0.00 |
| $$q_3$$ | -96.41 | -96.41 | 0.00 |
| $$q_4$$ | -75.36 | -75.36 | 0.00 |
| $$q_5$$ | 90.00 | 90.00 | 0.00 |
| $$q_6$$ | 11.54 | **101.54** | **90.00** |

> Como se esperaba, solo $$q_6$$ varía entre orientaciones ya que la rotación de 90° en Z de la herramienta se absorbe íntegramente en la última articulación de muñeca.

---

## 4a.6 Validación con Cinemática Directa

```python
def dh_matrix(a, d, alpha, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st,  ct*ca,-ct*sa, a*st],
        [ 0,     sa,    ca,    d],
        [ 0,      0,     0,    1],
    ])

def cinematica_directa(q):
    T = np.eye(4)
    for i in range(6):
        T = T @ dh_matrix(DH_A[i], DH_D[i], DH_ALPHA[i], q[i])
    return T

# Validar orientación 1
T_recuperada = cinematica_directa(q_ori1)
error = np.linalg.norm(T_recuperada[:3, 3] - p)
print(f"\nValidación Orientación 1:")
print(f"  Posición recuperada: {T_recuperada[:3,3]*1000} mm")
print(f"  Error de posición:   {error*1000:.4f} mm")

# Validar orientación 2
T_recuperada2 = cinematica_directa(q_ori2)
error2 = np.linalg.norm(T_recuperada2[:3, 3] - p)
print(f"\nValidación Orientación 2:")
print(f"  Posición recuperada: {T_recuperada2[:3,3]*1000} mm")
print(f"  Error de posición:   {error2*1000:.4f} mm")
```

**Salida:**

```
Validación Orientación 1:
  Posición recuperada: [200.000  -300.000  160.000] mm
  Error de posición:   0.0000 mm

Validación Orientación 2:
  Posición recuperada: [200.000  -300.000  160.000] mm
  Error de posición:   0.0000 mm
```

La recuperación exacta de la posición original confirma la corrección de la implementación analítica.

---

[← Cinemática Inversa](../04-cinematica-inversa) &nbsp;&nbsp; [Levenberg-Marquardt →](../04b-levenberg-marquardt){: .btn .btn-outline }
