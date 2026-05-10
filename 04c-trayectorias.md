---
layout: default
title: Trayectorias
parent: Cinemática Inversa
nav_order: 3
description: "Polinomio quíntico real — trajectory_planner.py y robot_controller.py"
permalink: /04-cinematica-inversa/04c-trayectorias/
---

# 4c. Planificación de Trayectorias
{: .no_toc }

Polinomio quíntico de `trajectory_planner.py` integrado con el planificador de tiempo de `robot_controller.py`.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4c.1 Las 3 Fases del Pick

El sistema usa la nomenclatura real de `main.py`:

{% highlight text %}
HOME
  │
  ▼  [quíntico — tf calculado] ← calcular_tf_quintico()
PREMOVE  (Z = 240 mm, misma XY del cubo)
  │
  ▼  [movej lento — vel=0.25, acel=0.2]
APPROACH (Z = 210 mm, misma XY)  ← SE DETIENE AQUÍ
  │
  ▼  [movej lento]
PICK     (Z = 160 mm, misma XY)  ← GRIP
  │
  ▼  [movej lento]
APPROACH (retract)
  │
  ▼  [quíntico] → PLACE → RELEASE → HOME
{% endhighlight %}

> **Decisión de diseño:** Los tránsitos (HOME→PREMOVE, PREMOVE PICK→PREMOVE PLACE) usan planificación quíntica para minimizar el tiempo de ciclo. Las fases cerca del cubo (APPROACH→PICK→APPROACH) usan `movej` lento para máximo control.

---

## 4c.2 Código Real — `trajectory_planner.py`

{% highlight python %}
# trajectory_planner.py — Polinomio quíntico completo

def polinomio_quintico(x0, xf, t_total=4.0, n=50):
    """
    Trayectoria suave de 5° grado.
    Condiciones: x(0)=x0, x(tf)=xf, v(0)=v(tf)=0, a(0)=a(tf)=0.
    """
    tf = t_total
    A = np.array([
        [0,         0,        0,       0,    0, 1],   # x(0)=x0
        [tf**5,    tf**4,    tf**3,   tf**2, tf, 1],  # x(tf)=xf
        [0,         0,        0,       0,    1, 0],   # v(0)=0
        [5*tf**4,  4*tf**3,  3*tf**2, 2*tf,  1, 0],  # v(tf)=0
        [0,         0,        0,       2,    0, 0],   # a(0)=0
        [20*tf**3, 12*tf**2,  6*tf,    2,    0, 0],   # a(tf)=0
    ])
    B      = np.array([x0, xf, 0, 0, 0, 0])
    coeffs = np.linalg.solve(A, B)
    a5, a4, a3, a2, a1, a0 = coeffs

    t   = np.linspace(0, tf, n)
    pos = a5*t**5 + a4*t**4 + a3*t**3 + a2*t**2 + a1*t + a0
    vel = 5*a5*t**4 + 4*a4*t**3 + 3*a3*t**2 + 2*a2*t + a1
    acc = 20*a5*t**3 + 12*a4*t**2 + 6*a3*t + 2*a2
    return pos, vel, acc, t


def planificar_trayectoria_cartesiana(p0, pf, t_total=4.0):
    """Interpolación quíntica independiente en X, Y, Z."""
    puntos = []
    x_t, _, _, _ = polinomio_quintico(p0[0], pf[0], t_total)
    y_t, _, _, _ = polinomio_quintico(p0[1], pf[1], t_total)
    z_t, _, _, _ = polinomio_quintico(p0[2], pf[2], t_total)
    for x, y, z in zip(x_t, y_t, z_t):
        puntos.append([float(x), float(y), float(z)])
    return puntos
{% endhighlight %}

---

## 4c.3 Cálculo de `tf` — `robot_controller.py`

{% highlight python %}
# robot_controller.py — calcular_tf_quintico()
def calcular_tf_quintico(joints_inicio_deg, joints_fin_deg,
                         vel_max_rad=1.0, acel_max_rad=0.3):
    """
    Calcula el tiempo mínimo tf para el movimiento quíntico.

    Para el polinomio quíntico con v0=vf=a0=af=0:
      v_max = (15/8) · |dq| / tf   → tf ≥ (15/8) · |dq| / vel_max
      a_max ≈ 10√3/3 · |dq| / tf² → tf ≥ sqrt(10√3/3 · |dq| / acel_max)
    """
    tf_min = 0.5   # mínimo absoluto en segundos

    for q0_deg, qf_deg in zip(joints_inicio_deg, joints_fin_deg):
        dq = abs(math.radians(qf_deg - q0_deg))
        if dq < 1e-6:
            continue
        tf_vel  = (15.0 / 8.0) * dq / vel_max_rad
        tf_acel = math.sqrt((10.0 * math.sqrt(3.0) / 3.0) * dq / acel_max_rad)
        tf_min  = max(tf_min, tf_vel, tf_acel)

    return tf_min
{% endhighlight %}

---

## 4c.4 Parámetros de Velocidad

{% highlight python %}
# robot_controller.py
VEL_J       = 1.0    # rad/s — velocidad normal (tránsitos)
ACEL_J      = 0.3    # rad/s² — aceleración normal
VEL_LENTO   = 0.25   # rad/s — velocidad lenta (cerca del cubo)
ACEL_LENTO  = 0.2    # rad/s² — aceleración lenta
{% endhighlight %}

| Movimiento | Velocidad | Aceleración | Modo |
|---|:---:|:---:|---|
| HOME → PREMOVE | 1.0 rad/s | 0.3 rad/s² | Quíntico |
| PREMOVE → APPROACH | 0.25 rad/s | 0.2 rad/s² | Normal lento |
| APPROACH → PICK | 0.25 rad/s | 0.2 rad/s² | Normal lento |
| PICK → PLACE | 0.25 rad/s | 0.2 rad/s² | Quíntico |

---

[← Levenberg-Marquardt](../04b-levenberg-marquardt) &nbsp;&nbsp; [Implementación →](../../05-implementacion-industrial){: .btn .btn-outline }