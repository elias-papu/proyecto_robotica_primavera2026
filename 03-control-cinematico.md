---
layout: default
title: Control Cinemático
nav_order: 4
description: "Jacobiano del UR3, control de velocidad proporcional y ejemplo práctico"
permalink: /03-control-cinematico/
---

# 3. Control Cinemático
{: .no_toc }

Uso del Jacobiano para control de velocidad cartesiana y convergencia hacia una posición deseada.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 3.1 Motivación

En muchas aplicaciones de robótica es necesario especificar **posiciones cartesianas** (coordenadas $$x, y, z$$) en lugar de ángulos articulares. El **control cinemático** permite mover el TCP a una posición cartesiana deseada usando una ley de control iterativa basada en el **Jacobiano**, sin necesidad de resolver la cinemática inversa analítica de forma explícita.

Dado un punto cartesiano deseado $$\mathbf{p}_d = [x_d, y_d, z_d]^T$$, el objetivo es encontrar los incrementos articulares $$\Delta\mathbf{q}$$ que minimicen el error:

$$
\mathbf{e}(t) = \mathbf{p}_d - \mathbf{p}(\mathbf{q}(t))
$$

---

## 3.2 El Jacobiano del UR3

El **Jacobiano geométrico** $$J(\mathbf{q}) \in \mathbb{R}^{6\times6}$$ relaciona las velocidades articulares con las velocidades cartesianas del TCP:

$$
\dot{\mathbf{x}} = J(\mathbf{q})\,\dot{\mathbf{q}}
$$

Para el control de posición (solo traslación), usamos las primeras 3 filas $$J_v \in \mathbb{R}^{3\times6}$$:

$$
\dot{\mathbf{p}} = J_v(\mathbf{q})\,\dot{\mathbf{q}}
$$

La columna $$j$$ del Jacobiano para una articulación **rotacional** es:

$$
J_j = \begin{bmatrix} \mathbf{z}_{j-1} \times (\mathbf{p}_n - \mathbf{p}_{j-1}) \\ \mathbf{z}_{j-1} \end{bmatrix}
$$

donde $$\mathbf{z}_{j-1}$$ es el eje de rotación de la articulación $$j$$ y $$\mathbf{p}_n$$ es la posición del TCP.

### Jacobiano Numérico (Diferencias Finitas)

En la implementación se calcula el Jacobiano **numéricamente** perturbando cada articulación:

$$
J_{ij} \approx \frac{f_i(\mathbf{q} + h\,\mathbf{e}_j) - f_i(\mathbf{q})}{h}
$$

con $$h = 10^{-4}$$ rad.

```python
def jacobiano_numerico(q, h=1e-4):
    """
    Calcula el Jacobiano numérico 3×6 por diferencias finitas.
    Solo posición (traslación), no orientación.
    """
    _, p0, _ = cinematica_directa(q)
    J = np.zeros((3, 6))
    for j in range(6):
        dq = q.copy()
        dq[j] += h
        _, p_perturb, _ = cinematica_directa(dq)
        J[:, j] = (p_perturb - p0) / h
    return J
```

---

## 3.3 Ley de Control Proporcional

El controlador aplica la siguiente ley iterativa:

$$
\Delta\mathbf{q} = k_p \cdot J_v^+(\mathbf{q}) \cdot \mathbf{e}
$$

donde $$J_v^+ = J_v^T(J_v J_v^T + \lambda I)^{-1}$$ es la pseudoinversa amortiguada (evita singularidades) y $$k_p$$ es la ganancia proporcional.

Las articulaciones se actualizan:

$$
\mathbf{q}_{k+1} = \mathbf{q}_k + \Delta\mathbf{q}_k
$$

---

## 3.4 Implementación en Python

```python
import numpy as np

def control_cinematico(p_deseada, q_inicial, kp=0.8, tol=1e-4,
                       max_iter=1000, lambda_damp=1e-4):
    """
    Control cinemático proporcional para alcanzar posición cartesiana.

    Args:
        p_deseada: array [x, y, z] en metros
        q_inicial: array de 6 ángulos iniciales en radianes
        kp:        ganancia proporcional
        tol:       tolerancia de error (metros)
        max_iter:  máximo de iteraciones
        lambda_damp: amortiguamiento para pseudoinversa

    Returns:
        q: ángulos articulares solución (radianes)
        historial_error: lista de errores por iteración
    """
    q = q_inicial.copy()
    historial_error = []

    for i in range(max_iter):
        # Posición actual del TCP
        _, p_actual, _ = cinematica_directa(q)

        # Error cartesiano
        error = p_deseada - p_actual
        norma_error = np.linalg.norm(error)
        historial_error.append(norma_error)

        # Convergencia
        if norma_error < tol:
            print(f"Convergencia en iteración {i}: error = {norma_error*1000:.3f} mm")
            break

        # Jacobiano numérico (solo posición)
        J = jacobiano_numerico(q)

        # Pseudoinversa amortiguada (evita singularidades)
        JJt = J @ J.T
        J_pinv = J.T @ np.linalg.inv(JJt + lambda_damp * np.eye(3))

        # Actualizar articulaciones
        delta_q = kp * J_pinv @ error
        q = q + delta_q

    return q, historial_error
```

---

## 3.5 Ejemplo Práctico — De HOME a Posición de Pick

### Objetivo

Mover el TCP desde la posición HOME hasta $$[0.2\,\text{m},\; -0.3\,\text{m},\; 0.16\,\text{m}]$$, que corresponde a la posición de agarre de un cubo real en el proyecto.

```python
import numpy as np
import matplotlib.pyplot as plt

# Condición inicial: HOME
q_home = np.deg2rad([0, -90, -90, 0, 90, 0])

# Posición deseada: punto de pick de un cubo
p_deseada = np.array([0.20, -0.30, 0.16])  # metros

# Ejecutar control cinemático
q_sol, errores = control_cinematico(
    p_deseada=p_deseada,
    q_inicial=q_home,
    kp=0.8,
    tol=1e-4,
    max_iter=500
)

# Verificar resultado
_, p_final, _ = cinematica_directa(q_sol)
print("\n--- RESULTADO ---")
print(f"Posición deseada:  x={p_deseada[0]*1000:.1f}, y={p_deseada[1]*1000:.1f}, z={p_deseada[2]*1000:.1f} mm")
print(f"Posición obtenida: x={p_final[0]*1000:.1f}, y={p_final[1]*1000:.1f}, z={p_final[2]*1000:.1f} mm")
print(f"Error final: {np.linalg.norm(p_final - p_deseada)*1000:.3f} mm")
print(f"Iteraciones: {len(errores)}")
print(f"\nÁngulos solución (grados): {np.round(np.rad2deg(q_sol), 2)}")
```

**Salida:**

```
Convergencia en iteración 47: error = 0.092 mm

--- RESULTADO ---
Posición deseada:  x=200.0, y=-300.0, z=160.0 mm
Posición obtenida: x=200.0, y=-300.0, z=160.0 mm
Error final: 0.092 mm
Iteraciones: 48
Ángulos solución (grados): [ 11.54  -98.23  -96.41  -75.36   90.0   11.54]
```

### Gráfica de Convergencia del Error

```python
# Gráfica de convergencia
plt.figure(figsize=(10, 5))
plt.semilogy(np.array(errores) * 1000, 'b-o', markersize=3, linewidth=1.5)
plt.axhline(0.1, color='r', linestyle='--', label='Tolerancia 0.1 mm')
plt.xlabel('Iteración')
plt.ylabel('Error cartesiano (mm)')
plt.title('Convergencia del Control Cinemático\nHOME → [200, -300, 160] mm')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('assets/img/convergencia_control.png', dpi=150)
plt.show()
```

![Gráfica de convergencia del error cartesiano]( ../assets/img/convergencia_control.png)

---

## 3.6 Comparación con Cinemática Inversa Analítica

| Criterio | Control Cinemático | IK Analítica |
|---|---|---|
| **Velocidad** | ~50 iteraciones, ~5 ms | Cálculo directo, < 1 ms |
| **Precisión** | ~0.1 mm (ajustable) | Exacta (precisión numérica) |
| **Singularidades** | Manejo con amortiguamiento | Detección explícita |
| **Orientación** | Solo posición (3 DOF) | Posición + orientación (6 DOF) |
| **Configuración** | Puede converger a solución subóptima | Selección explícita de rama |
| **Tiempo real** | Adecuado para trayectorias lentas | Óptimo para pick and place |
| **Implementación** | Simple, genérica | Específica para cada robot |

### ¿Cuándo usar cada enfoque?

- **Control cinemático:** Útil para seguimiento de trayectorias continuas, teleoperation o cuando no se dispone de solución analítica. Requiere ajuste fino de $$k_p$$ y $$\lambda$$.

- **IK analítica:** Preferida para pick and place industrial donde se necesita máxima velocidad, solución exacta y selección de configuración (codo arriba/abajo). Es el método utilizado en este proyecto.

> **Elección del proyecto:** Se utiliza la **IK analítica** para todas las operaciones de pick and place del CoBot, ya que garantiza convergencia instantánea (< 1 ms) y permite seleccionar explícitamente la rama de solución deseada (codo abajo, rama positiva de $$q_1$$).

---

[← Cinemática Directa](../02-cinematica-directa) &nbsp;&nbsp; [Cinemática Inversa →](../04-cinematica-inversa){: .btn .btn-outline }
