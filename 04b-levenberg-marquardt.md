---
layout: default
title: Levenberg-Marquardt
parent: Cinemática Inversa
nav_order: 2
description: "IK numérica LM — fallback de la IK analítica"
permalink: /04-cinematica-inversa/04b-levenberg-marquardt/
---

# 4b. Levenberg-Marquardt — `ik_numerica.py`
{: .no_toc }

Algoritmo numérico de IK usado como último fallback cuando las 4 ramas analíticas fallan.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4b.1 Fundamento

El algoritmo minimiza el error cartesiano entre la pose calculada por la cinemática directa y la pose deseada:

$$
\Delta\mathbf{q} = \left(J^T W J + \lambda I\right)^{-1} J^T W \mathbf{e}
$$

Con pesos $$W = \text{diag}(10, 10, 10, 0.5, 0.5, 0.5)$$ que priorizan la posición sobre la orientación, y factor de amortiguamiento $$\lambda = 0.05$$.

---

## 4b.2 Código Real — `ik_numerica.py`

```python
# ik_numerica.py — Función completa ik_numerica()

def ik_numerica(px_m, py_m, pz_m, q_inicial=None,
                max_iter=100, tol=1e-4):
    """
    IK por Levenberg-Marquardt con Jacobiano 6×6 numérico.
    Fallback de la IK analítica — solo se activa con estrategia 5.
    """
    # Configuración inicial: HOME si no se especifica
    if q_inicial is None:
        q = np.array([0.0, -math.pi/2, -math.pi/2, 0.0, math.pi/2, 0.0])
    else:
        q = np.array([math.radians(x) for x in q_inicial])

    p_target = np.array([px_m, py_m, pz_m])
    R_target = np.array([
        [_DEFAULT_ROT["nx"], _DEFAULT_ROT["ox"], _DEFAULT_ROT["ax"]],
        [_DEFAULT_ROT["ny"], _DEFAULT_ROT["oy"], _DEFAULT_ROT["ay"]],
        [_DEFAULT_ROT["nz"], _DEFAULT_ROT["oz"], _DEFAULT_ROT["az"]]
    ])

    # Pesos: posición importa 10× más que orientación
    W   = np.diag([10.0, 10.0, 10.0, 0.5, 0.5, 0.5])
    lam = 0.05

    for _ in range(max_iter):
        T = _cd(q)                          # cinemática directa
        e_pos = p_target - T[:3, 3]         # error de posición
        e_ori = _error_orientacion(R_target, T[:3, :3])
        e = np.concatenate([e_pos, e_ori])

        if np.linalg.norm(e_pos) < tol:     # convergió
            q_norm = [_normalizar_rad(qi) for qi in q]
            return {
                "valido":     True,
                "mensaje":    "OK (IK numérica)",
                "joints_rad": q_norm,
                "joints_deg": [math.degrees(qi) for qi in q_norm],
            }

        J    = _jacobiano(q)
        JtW  = J.T @ W
        delta = np.linalg.solve(JtW @ J + lam * np.eye(6), JtW @ e)
        q   += delta

        # Límites articulares UR3
        q = np.clip(q,
            [-3.14, -3.14, -1.57, -3.14, -3.14, -3.14],
            [ 3.14,  0.0,   1.57,  3.14,  3.14,  3.14])

    return {"valido": False, "mensaje": f"No convergió en {max_iter} iter"}
```

---

## 4b.3 Comparativa Analítica vs. Numérica

| Criterio | IK Analítica | Levenberg-Marquardt |
|---|:---:|:---:|
| **Tiempo de cómputo** | < 1 ms | 2–50 ms |
| **Error de posición** | < 10⁻¹⁰ m | < 10⁻⁴ m (tol) |
| **Error de orientación** | Exacto | Ponderado (W) |
| **Selección de rama** | ✅ Explícita | ❌ Depende del inicial |
| **Singularidades** | ✅ Detectadas | ✅ Amortiguadas (λ) |
| **Rol en el proyecto** | Principal | Fallback estrategia 5 |

---

[← Método Analítico](../04a-metodo-geometrico) &nbsp;&nbsp; [Trayectorias →](../04c-trayectorias){: .btn .btn-outline }
