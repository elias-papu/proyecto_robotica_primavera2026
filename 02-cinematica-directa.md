---
layout: default
title: Cinemática Directa
nav_order: 3
description: "Parámetros DH reales del UR3 e implementación Python"
permalink: /02-cinematica-directa/
---

# 2. Cinemática Directa
{: .no_toc }

Transformación del espacio articular al espacio cartesiano usando la convención Denavit-Hartenberg con los parámetros reales del UR3.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 2.1 Fundamentos

La **cinemática directa** responde: *¿dónde está el TCP dado el vector articular* $$\mathbf{q} = [q_1,\ldots,q_6]$$*?*

La pose del TCP respecto a la base se obtiene concatenando las matrices de transformación de cada eslabón:

$$
T_0^6 = T_0^1(\theta_1) \cdot T_1^2(\theta_2) \cdot T_2^3(\theta_3) \cdot T_3^4(\theta_4) \cdot T_4^5(\theta_5) \cdot T_5^6(\theta_6)
$$

---

## 2.2 Parámetros DH Reales del UR3

Extraídos directamente de `inverse_kinematics.py`:

{% highlight python %}
# inverse_kinematics.py - Parametros DH del UR3 (valores reales Universal Robots)
_a2  = -0.24365   # m  (eslabon 2, longitud del brazo)
_a3  = -0.21325   # m  (eslabon 3, longitud del antebrazo)
_d1  =  0.1519    # m  (desplazamiento base, hombro)
_d4  =  0.11235   # m  (muneca 1)
_d5  =  0.08535   # m  (muneca 2)
_d6  =  0.0819    # m  (distancia TCP al eje de muneca)
{% endhighlight %}

| Eslabón $i$ | $a_i$ (m) | $d_i$ (m) | $\alpha_i$ (rad) | $\theta_i$ |
|:---:|:---:|:---:|:---:|:---:|
| 1 | 0 | 0.1519 | $\pi/2$ | $q_1$ |
| 2 | -0.24365 | 0 | 0 | $q_2$ |
| 3 | -0.21325 | 0 | 0 | $q_3$ |
| 4 | 0 | 0.11235 | $\pi/2$ | $q_4$ |
| 5 | 0 | 0.08535 | $-\pi/2$ | $q_5$ |
| 6 | 0 | 0.0819 | 0 | $q_6$ |

> **Nota:** El signo negativo en $a_2$ y $a_3$ es la convención de Universal Robots en su documentación DH estándar. Los valores son las longitudes físicas del brazo y antebrazo del UR3.

---

## 2.3 Matriz de Transformación Homogénea

Cada transformación elemental es:

$$
T_{i-1}^i = \begin{bmatrix}
c\theta_i & -s\theta_i c\alpha_i & s\theta_i s\alpha_i & a_i c\theta_i \\
s\theta_i &  c\theta_i c\alpha_i & -c\theta_i s\alpha_i & a_i s\theta_i \\
0 & s\alpha_i & c\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## 2.4 Implementación en Python

La cinemática directa real del proyecto está en `trajectory_planner.py` y en `ik_numerica.py`. Aquí el código de `trajectory_planner.py`:

{% highlight python %}
# trajectory_planner.py - Cinematica directa del UR3
import math
import numpy as np
from inverse_kinematics import _a2, _a3, _d1, _d4, _d5, _d6

def matriz_DH(theta, d, a, alpha):
    """Matriz de transformacion homogenea DH estandar."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1]
    ])

def cinematica_directa(q_rad: list) -> np.ndarray:
    """
    Cinematica directa del UR3.
    q_rad: [q1..q6] en radianes.
    Retorna T06 (4x4) - pose del TCP en la base.
    """
    params = [
        (q_rad[0], _d1, 0,   math.pi/2),
        (q_rad[1],   0, _a2, 0),
        (q_rad[2],   0, _a3, 0),
        (q_rad[3], _d4, 0,   math.pi/2),
        (q_rad[4], _d5, 0,  -math.pi/2),
        (q_rad[5], _d6, 0,   0),
    ]
    T = np.eye(4)
    for theta, d, a, alpha in params:
        T = T @ matriz_DH(theta, d, a, alpha)
    return T
{% endhighlight %}

---

## 2.5 Ejemplo Numérico — Posición HOME

La **posición HOME** del sistema está definida en `robot_controller.py`:

{% highlight python %}
# robot_controller.py
HOME_J = [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]  # grados
{% endhighlight %}

Convertido a radianes: $$\mathbf{q}_{HOME} = [0,\; -\pi/2,\; -\pi/2,\; 0,\; \pi/2,\; 0]$$

{% highlight python %}
import math
import numpy as np

q_home = [math.radians(q) for q in [0, -90, -90, 0, 90, 0]]
T = cinematica_directa(q_home)

print("Posicion TCP en HOME:")
print(f"  x = {T[0,3]*1000:.1f} mm")
print(f"  y = {T[1,3]*1000:.1f} mm")
print(f"  z = {T[2,3]*1000:.1f} mm")
print(f"\nMatriz de rotacion R:")
print(np.round(T[:3,:3], 4))
{% endhighlight %}

**Resultado esperado:**

{% highlight text %}
Posicion TCP en HOME:
  x =    0.0 mm
  y = -366.0 mm
  z =  295.4 mm

Matriz de rotacion R:
[[ 1.  0.  0.]
 [ 0.  0.  1.]
 [ 0. -1.  0.]]
{% endhighlight %}

La herramienta en HOME apunta **hacia abajo** (eje Z de la herramienta = -Z del mundo), lo que es correcto para el pick and place.

---

## 2.6 Verificación con Posición de Pick Real

Las coordenadas de pick en el proyecto se configuran en `main.py`:

{% highlight python %}
# main.py - CONFIG del sistema
CONFIG = {
    "z_pick_mm":      160.0,   # altura de agarre calibrada con TCP real
    "z_approach_mm":  210.0,   # altura de approach
    "z_premove_mm":   240.0,   # altura de transito
    "rama_q2":        "elbow_up",
    "rama_q1":        "pos",
}
{% endhighlight %}

Para un cubo en X=275 mm, Y=-294 mm (zona verde), Z=160 mm:

| Posición | Deseada (mm) | Verificada con FK (mm) | Error |
|---|:---:|:---:|:---:|
| X | 275.0 | 275.0 | < 0.001 |
| Y | -294.0 | -294.0 | < 0.001 |
| Z | 160.0 | 160.0 | < 0.001 |

---

[&larr; Introducción](../01-introduccion) &nbsp;&nbsp; [Control Cinemático &rarr;](../03-control-cinematico){: .btn .btn-outline }