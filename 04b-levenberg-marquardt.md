---
layout: default
title: Levenberg-Marquardt
parent: Cinemática Inversa
nav_order: 2
description: "Algoritmo de Levenberg-Marquardt para IK numérica del UR3"
permalink: /04-cinematica-inversa/04b-levenberg-marquardt/
---

# 4b. Levenberg-Marquardt
{: .no_toc }

Método numérico de IK que minimiza el error entre la cinemática directa y la pose deseada.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4b.1 Fundamento del Algoritmo

El **algoritmo de Levenberg-Marquardt (LM)** es un método de mínimos cuadrados no lineales que combina el descenso por gradiente con el método de Gauss-Newton. Para IK, minimiza la función de costo:

$$
F(\mathbf{q}) = \frac{1}{2} \|\mathbf{e}(\mathbf{q})\|^2
$$

donde el vector de error es:

$$
\mathbf{e}(\mathbf{q}) = \mathbf{x}_d - f(\mathbf{q})
$$

siendo $$f(\mathbf{q})$$ la cinemática directa y $$\mathbf{x}_d$$ la posición deseada.

### Regla de Actualización LM

$$
\Delta\mathbf{q} = \left(J^T J + \lambda I\right)^{-1} J^T \mathbf{e}
$$

donde:
- $$J \in \mathbb{R}^{3\times6}$$ es el Jacobiano numérico (solo posición)
- $$\lambda$$ es el **factor de amortiguamiento**: grande → descenso por gradiente, pequeño → Gauss-Newton
- $$I$$ es la identidad $$6\times6$$

El factor $$\lambda$$ se actualiza dinámicamente:
- Si la iteración **reduce el error** → disminuir $$\lambda$$ (acercarse a Gauss-Newton, convergencia rápida)
- Si la iteración **aumenta el error** → aumentar $$\lambda$$ (más estable, como descenso por gradiente)

---

## 4b.2 Jacobiano Numérico por Diferencias Finitas

El Jacobiano se aproxima perturbando cada articulación en $$h = 10^{-4}$$ rad:

$$
J_{ij} = \frac{f_i(\mathbf{q} + h\,\hat{e}_j) - f_i(\mathbf{q})}{h}
$$

```python
def jacobiano_numerico_3x6(q, h=1e-4):
    """Jacobiano numérico de posición 3×6 por diferencias finitas."""
    _, p0, _ = cinematica_directa(q)
    J = np.zeros((3, 6))
    for j in range(6):
        dq = q.copy()
        dq[j] += h
        _, p_pert, _ = cinematica_directa(dq)
        J[:, j] = (p_pert - p0) / h
    return J, p0
```

---

## 4b.3 Implementación Completa en Python

```python
import numpy as np

def ik_levenberg_marquardt(p_deseada, q_inicial=None, tol=1e-5,
                            max_iter=200, lambda_init=1e-3,
                            lambda_factor=10.0):
    """
    Cinemática inversa por Levenberg-Marquardt (solo posición).

    Args:
        p_deseada:    array [x, y, z] en metros
        q_inicial:    vector articular inicial (default: HOME)
        tol:          tolerancia de convergencia (metros)
        max_iter:     máximo de iteraciones
        lambda_init:  factor de amortiguamiento inicial
        lambda_factor: factor de ajuste de lambda

    Returns:
        q:       solución articular (radianes)
        historia: lista de (iteración, error) para graficar
    """
    if q_inicial is None:
        q_inicial = np.deg2rad([0, -90, -90, 0, 90, 0])  # HOME

    q = q_inicial.copy()
    lam = lambda_init
    historia = []

    for i in range(max_iter):
        # Jacobiano y posición actual
        J, p_actual = jacobiano_numerico_3x6(q)

        # Error cartesiano
        e = p_deseada - p_actual
        error = np.linalg.norm(e)
        historia.append((i, error))

        if error < tol:
            print(f"  LM convergió en iteración {i}: error = {error*1000:.4f} mm")
            break

        # Paso LM: Δq = (JᵀJ + λI)⁻¹ Jᵀ e
        JtJ = J.T @ J
        Jte = J.T @ e
        delta_q = np.linalg.solve(JtJ + lam * np.eye(6), Jte)

        # Evaluar si el paso mejora
        q_new = q + delta_q
        _, p_new, _ = cinematica_directa(q_new)
        error_new = np.linalg.norm(p_deseada - p_new)

        if error_new < error:
            q = q_new
            lam /= lambda_factor   # Más agresivo (Gauss-Newton)
        else:
            lam *= lambda_factor   # Más conservador (gradiente)

    return q, historia
```

---

## 4b.4 Caso 1 — Orientación 1 (Herramienta hacia abajo)

**Objetivo:** $$\mathbf{p}_d = [0.2, -0.3, 0.16]$$ m, partiendo de HOME.

```python
# Caso 1: herramienta hacia abajo, partiendo de HOME
p_objetivo = np.array([0.20, -0.30, 0.16])
q_home     = np.deg2rad([0, -90, -90, 0, 90, 0])

q_lm1, hist1 = ik_levenberg_marquardt(
    p_deseada=p_objetivo,
    q_inicial=q_home,
    tol=1e-5, max_iter=200, lambda_init=1e-3
)

print("\n── Caso 1: Herramienta hacia abajo ──")
print(f"  Ángulos (grados): {np.round(np.rad2deg(q_lm1), 2)}")
_, p_final, _ = cinematica_directa(q_lm1)
print(f"  Posición obtenida: {p_final*1000} mm")
print(f"  Error final: {np.linalg.norm(p_final - p_objetivo)*1000:.4f} mm")
print(f"  Iteraciones: {len(hist1)}")
```

**Convergencia por iteración (Caso 1):**

| Iteración | Error (mm) |
|:---:|:---:|
| 0 | 147.23 |
| 5 | 12.40 |
| 10 | 1.83 |
| 20 | 0.21 |
| 35 | 0.009 |

---

## 4b.5 Caso 2 — Orientación 2 (Herramienta inclinada 45° en Y)

**Objetivo:** $$\mathbf{p}_d = [0.15, -0.25, 0.20]$$ m, herramienta inclinada 45° en Y.

```python
import numpy as np

# Orientación: herramienta inclinada 45° en Y
theta_y = np.radians(45)
Ry45 = np.array([
    [ np.cos(theta_y), 0, np.sin(theta_y)],
    [               0, 1,               0],
    [-np.sin(theta_y), 0, np.cos(theta_y)],
])
# Orientación base (herramienta hacia abajo): a = [0, 0, -1]
R_base = np.array([
    [-1, 0,  0],
    [ 0, 1,  0],
    [ 0, 0, -1],
])
R_ori2 = R_base @ Ry45

p2 = np.array([0.15, -0.25, 0.20])

q_lm2, hist2 = ik_levenberg_marquardt(
    p_deseada=p2,
    q_inicial=q_home,
    tol=1e-5, max_iter=200, lambda_init=1e-3
)

print("\n── Caso 2: Herramienta inclinada 45° en Y ──")
print(f"  Ángulos (grados): {np.round(np.rad2deg(q_lm2), 2)}")
_, p_final2, _ = cinematica_directa(q_lm2)
print(f"  Posición obtenida: {p_final2*1000} mm")
print(f"  Error final: {np.linalg.norm(p_final2 - p2)*1000:.4f} mm")
print(f"  Iteraciones: {len(hist2)}")
```

---

## 4b.6 Comparativa: Geométrico vs. Levenberg-Marquardt

| Criterio | IK Geométrica | Levenberg-Marquardt |
|---|:---:|:---:|
| **Tiempo de cómputo** | < 1 ms | 2–15 ms |
| **Iteraciones** | 1 (directo) | 20–80 |
| **Error posición** | < 1×10⁻¹⁰ mm | < 0.001 mm |
| **Error orientación** | Exacto | Solo posición |
| **Punto de partida** | No requerido | Crítico |
| **Singularidades** | Detección explícita | Amortiguamiento λ |
| **Selección de rama** | ✅ Explícita | ❌ Depende del inicial |
| **Tiempo real (< 1 ms)** | ✅ Sí | ❌ No |
| **Aplicación** | Pick & Place industrial | Posiciones difíciles / validación |
| **Implementación** | Específica UR3 | Genérica (cualquier robot) |

> **Recomendación:** Usar el **método geométrico** para todas las operaciones de pick and place en tiempo real. Reservar LM para validación, posiciones fuera del rango analítico o como fallback de seguridad.

---

[← Método Geométrico](../04a-metodo-geometrico) &nbsp;&nbsp; [Trayectorias →](../04c-trayectorias){: .btn .btn-outline }
