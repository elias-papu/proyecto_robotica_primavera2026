---
layout: default
title: Cinemática Inversa
nav_order: 5
has_children: true
description: "IK analítica con cascada de 5 estrategias de fallback"
permalink: /04-cinematica-inversa/
---

# 4. Cinemática Inversa
{: .no_toc }

Sistema de IK con cascada de 5 estrategias: analítica → ramas alternativas → barrido Z → proyección XY → Levenberg-Marquardt.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4.1 El Problema Real

En el proyecto, la IK debe resolverse para tres posiciones diferentes en cada pick:

{% highlight python %}
# main.py — calcular_y_mostrar_ik
ik_premove  = calcular_ik(x_m, y_m, z_premove_m,  ...)  # Z=240 mm
ik_approach = calcular_ik(x_m, y_m, z_approach_m, ...)  # Z=210 mm
ik_pick     = calcular_ik(x_m, y_m, z_pick_m,     ...)  # Z=160 mm
{% endhighlight %}

Si **cualquiera de las tres falla**, el pick no puede ejecutarse. Esto impulsó el diseño de una IK robusta con cascada de fallbacks.

---

## 4.2 Ramas de Solución

El UR3 tiene hasta 8 soluciones para cada pose. Las dos ramas principales configurables en `main.py → CONFIG`:

| Parámetro | Opciones | Default |
|---|---|---|
| `rama_q2` | `"elbow_up"` / `"elbow_down"` | `"elbow_up"` |
| `rama_q1` | `"pos"` / `"neg"` | `"pos"` |

### Configuración elegida para el proyecto

{% highlight python %}
CONFIG = {
    "rama_q2": "elbow_up",   # ← q3 positivo: brazo extendido hacia arriba
    "rama_q1": "pos",        # ← rama positiva de q1
    ...
}
{% endhighlight %}

---

## 4.3 Cascada de 5 Estrategias

La función `calcular_ik()` en `inverse_kinematics.py` implementa una cascada de 5 estrategias para maximizar la tasa de éxito:

{% highlight python %}
# inverse_kinematics.py — calcular_ik() con cascada completa
def calcular_ik(px_m, py_m, pz_m, rot=None, rama_q2="elbow_up", rama_q1="pos"):
    """
    Cascada de 5 estrategias para maximizar la tasa de éxito de la IK.
    """

    # ── ESTRATEGIA 1: rama pedida ─────────────────────────────────────────
    res = calcular_ik_raw(px_m, py_m, pz_m, rot, rama_q2, rama_q1)
    if res["valido"]:
        return res

    # ── ESTRATEGIA 2: las 4 combinaciones de rama (elbow × shoulder) ─────
    for r2, r1 in [("elbow_up","pos"), ("elbow_down","pos"),
                   ("elbow_up","neg"), ("elbow_down","neg")]:
        res = calcular_ik_raw(px_m, py_m, pz_m, rot, r2, r1)
        if res["valido"]:
            res["mensaje"] = "OK (rama alternativa)"
            return res

    # ── ESTRATEGIA 3: barrido de Z ±60 mm ────────────────────────────────
    for dz_mm in [20, -20, 40, -40, 60, -60]:
        pz_try = pz_m + dz_mm / 1000.0
        res = _intentar_todas_ramas(px_m, py_m, pz_try, rot)
        if res["valido"]:
            res["mensaje"] = f"OK (Z ajustada {dz_mm:+d} mm)"
            return res

    # ── ESTRATEGIA 4: proyectar XY al radio seguro (0.40 m) ──────────────
    R_MAX = 0.40
    r_xy = math.sqrt(px_m**2 + py_m**2)
    if r_xy > R_MAX:
        factor  = R_MAX / r_xy
        px_proj = px_m * factor
        py_proj = py_m * factor
        for pz_try in [pz_m] + [pz_m + dz/1000 for dz in [20,-20,40,-40]]:
            res = _intentar_todas_ramas(px_proj, py_proj, pz_try, rot)
            if res["valido"]:
                return res

    # ── ESTRATEGIA 5: IK numérica Levenberg-Marquardt ────────────────────
    from ik_numerica import ik_numerica
    return ik_numerica(px_m, py_m, pz_m)
{% endhighlight %}

---

## 4.4 Orientación de Herramienta por Defecto

La orientación estándar (herramienta apuntando hacia abajo) está definida en `inverse_kinematics.py`:

{% highlight python %}
# inverse_kinematics.py — orientación por defecto
_DEFAULT_ROT = {
    "nx": -0.7673, "ny": -0.6402, "nz":  0.0388,
    "ox": -0.6413, "oy":  0.7664, "oz": -0.0371,
    "ax": -0.0060, "ay": -0.0533, "az": -0.9986,
}
{% endhighlight %}

Esta orientación fue obtenida empíricamente moviendo el robot a la posición de pick con el teach pendant y leyendo la pose del TCP.

---

## 4.5 Por qué IK Analítica como Principal

| Criterio | Analítica | LM Numérico |
|---|:---:|:---:|
| **Tiempo de cómputo** | < 1 ms | 2–50 ms |
| **Determinismo** | ✅ Siempre la misma solución | ❌ Depende del punto inicial |
| **Selección de rama** | ✅ Explícita | ❌ Implícita |
| **Manejo de singularidades** | ✅ Detección y mensaje | ✅ Amortiguamiento λ |
| **Tiempo real** | ✅ Sí | 🔶 Marginal |
| **Rol en el proyecto** | ✅ Principal (estrategias 1-4) | 🔶 Fallback (estrategia 5) |

---

[← Control Cinemático](../03-control-cinematico) &nbsp;&nbsp; [Método Analítico →](04a-metodo-geometrico){: .btn .btn-outline }