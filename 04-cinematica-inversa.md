---
layout: default
title: Cinemática Inversa
nav_order: 5
has_children: true
description: "Métodos analítico y numérico de cinemática inversa para el UR3"
permalink: /04-cinematica-inversa/
---

# 4. Cinemática Inversa
{: .no_toc }

Métodos para calcular los ángulos articulares del UR3 dada una pose cartesiana deseada.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4.1 Planteamiento del Problema

La **cinemática inversa (IK)** resuelve el problema dual a la cinemática directa: dado un punto cartesiano y una orientación deseados del TCP, encontrar el vector articular $$\mathbf{q}$$ que los produce.

Formalmente, para el UR3 con 6 GDL:

$$
\text{Dado: } \mathbf{T}_d = \begin{bmatrix} \mathbf{R}_d & \mathbf{p}_d \\ \mathbf{0} & 1 \end{bmatrix} \quad \Rightarrow \quad \text{Encontrar: } \mathbf{q} = [q_1, q_2, q_3, q_4, q_5, q_6]
$$

tal que $$T_0^6(\mathbf{q}) = \mathbf{T}_d$$.

---

## 4.2 Ramas de Solución

Para un robot de 6 GDL como el UR3, la cinemática inversa tiene **hasta 8 soluciones** (configuraciones) que alcanzan la misma pose. Las ramas principales son:

### Rama de $$q_1$$: Positiva / Negativa

$$q_1$$ tiene dos soluciones geométricamente distintas según el signo de la raíz cuadrada en la derivación:

| Rama | Descripción | Uso en el proyecto |
|---|---|---|
| **Positiva** | $$q_1 > 0$$ (robot girado a la izquierda) | Zona verde e izquierda |
| **Negativa** | $$q_1 < 0$$ (robot girado a la derecha) | Zona roja y azul |

### Rama del Codo: Arriba / Abajo

La articulación $$q_3$$ tiene dos soluciones ($$q_3 > 0$$ o $$q_3 < 0$$) que dan lugar a las configuraciones de codo arriba (*elbow up*) y codo abajo (*elbow down*).

| Configuración | $$q_3$$ | Características |
|---|:---:|---|
| **Elbow Down** | $$< 0$$ | Mayor alcance, evita colisiones con mesa |
| **Elbow Up** | $$> 0$$ | Menor alcance, útil para posiciones altas |

> **Configuración elegida:** El proyecto usa **elbow down + rama positiva de $$q_1$$** para todas las operaciones de clasificación, ya que el área de trabajo está bajo el plano del hombro y esta configuración evita colisiones con la superficie de la mesa.

---

## 4.3 Método Analítico vs. Métodos Numéricos

Para seleccionar el método de IK adecuado para una aplicación industrial en tiempo real, se comparan las principales alternativas:

| Criterio | Analítico | Levenberg-Marquardt | Jacobiano Transpuesto | CCD |
|---|:---:|:---:|:---:|:---:|
| **Velocidad** | ⚡ < 1 ms | 🔶 2–10 ms | 🔶 1–5 ms | 🔶 1–5 ms |
| **Solución exacta** | ✅ Sí | ✅ Con tolerancia | ✅ Con tolerancia | ✅ Con tolerancia |
| **Selección de rama** | ✅ Explícita | ❌ Depende del inicial | ❌ Depende del inicial | ❌ Depende del inicial |
| **Manejo de singularidades** | 🔶 Detectables | ✅ Amortiguamiento | 🔶 Puede oscilar | 🔶 Puede oscilar |
| **Implementación** | 🔶 Específica por robot | ✅ Genérica | ✅ Genérica | ✅ Genérica |
| **Tiempo real (< 1 ms)** | ✅ Sí | ❌ No siempre | ❌ No siempre | ❌ No siempre |

### ¿Por qué se eligió el método analítico?

1. **Velocidad garantizada:** El cálculo analítico completa los 6 ángulos en una sola pasada (< 1 ms), permitiendo ciclos de control de hasta 1 kHz.

2. **Determinismo:** No depende del punto de partida ni de parámetros de convergencia. Siempre produce la misma solución para una rama dada.

3. **Selección de configuración:** La rama (codo arriba/abajo) se puede especificar explícitamente, evitando movimientos inesperados del robot.

4. **Aplicación industrial:** En pick and place real, el robot ejecuta entre 20 y 50 picks por minuto. Cada milisegundo de latencia se acumula y afecta el throughput.

> **Conclusión:** Para el CoBot Clasificador de Colores, el método analítico es la elección natural. Se implementa también el método Levenberg-Marquardt como referencia académica y para posiciones fuera del workspace del analítico.

---

## 4.4 Estructura de esta Sección

Esta sección cubre tres temas:

1. [**Método Geométrico (Analítico)**](04a-metodo-geometrico) — Derivación paso a paso de los 6 ángulos, código Python completo y validación con dos orientaciones.

2. [**Levenberg-Marquardt**](04b-levenberg-marquardt) — Algoritmo numérico de minimización, código Python y comparativa de rendimiento.

3. [**Planificación de Trayectorias**](04c-trayectorias) — Polinomios quínticos y perfil trapezoidal para las fases de pick and place.

---

[← Control Cinemático](../03-control-cinematico) &nbsp;&nbsp; [Método Geométrico →](04a-metodo-geometrico){: .btn .btn-outline }
