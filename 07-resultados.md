---
layout: default
title: Resultados
nav_order: 8
description: "Pruebas de cinemática inversa, comparativas de métodos y conclusiones del proyecto"
permalink: /07-resultados/
---

# 7. Resultados
{: .no_toc }

Métricas de precisión, comparativas de métodos y conclusiones del CoBot Clasificador de Colores.
{: .fs-6 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 7.1 Pruebas de Cinemática Inversa

Se evaluó la precisión de la IK analítica calculando la posición obtenida mediante cinemática directa y comparándola con la posición deseada. Se realizaron 10 pruebas en posiciones representativas del área de trabajo:

| # | X deseada (mm) | Y deseada (mm) | Z deseada (mm) | X obtenida (mm) | Y obtenida (mm) | Z obtenida (mm) | Error (mm) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | 200.0 | -300.0 | 160.0 | 200.0 | -300.0 | 160.0 | 0.000 |
| 2 | -200.0 | -300.0 | 160.0 | -200.0 | -300.0 | 160.0 | 0.000 |
| 3 | 0.0 | -300.0 | 160.0 | 0.0 | -300.0 | 160.0 | 0.000 |
| 4 | 300.0 | -250.0 | 180.0 | 300.0 | -250.0 | 180.0 | 0.000 |
| 5 | -300.0 | -400.0 | 160.0 | -300.0 | -400.0 | 160.0 | 0.000 |
| 6 | 150.0 | -200.0 | 200.0 | 150.0 | -200.0 | 200.0 | 0.000 |
| 7 | -150.0 | -350.0 | 140.0 | -150.0 | -350.0 | 140.0 | 0.000 |
| 8 | 400.0 | -150.0 | 220.0 | 400.0 | -150.0 | 220.0 | 0.000 |
| 9 | 0.0 | -500.0 | 160.0 | 0.0 | -500.0 | 160.0 | 0.000 |
| 10 | 250.0 | -300.0 | 160.0 | 250.0 | -300.0 | 160.0 | 0.000 |

> **Resultado:** La IK analítica alcanza **error numérico de 0.000 mm** (precisión de punto flotante de doble precisión, ~10⁻¹² m) en todas las posiciones dentro del workspace. El método es exacto, no aproximado.

---

## 7.2 Comparativa de Métodos IK

| Criterio | IK Geométrica | Levenberg-Marquardt | Control Cinemático |
|---|:---:|:---:|:---:|
| **Tiempo de cómputo** | < 1 ms | 2–15 ms | 3–8 ms |
| **Error de posición** | < 10⁻¹⁰ mm | < 0.001 mm | < 0.1 mm |
| **Error de orientación** | < 10⁻¹⁰ mm | No controlado | No controlado |
| **Iteraciones** | 1 (directo) | 20–80 | 40–120 |
| **Selección de rama** | ✅ Explícita | ❌ Depende del inicial | ❌ Depende del inicial |
| **Robustez** | ✅ Alta | ✅ Alta | 🔶 Media |
| **Singularidades** | ✅ Detectables | ✅ Amortiguamiento λ | 🔶 Posible oscilación |
| **Tiempo real** | ✅ Sí | 🔶 Marginal | 🔶 Marginal |
| **Implementación** | 🔶 Específica UR3 | ✅ Genérica | ✅ Genérica |
| **Uso en proyecto** | ✅ Principal | 📚 Referencia académica | 📚 Referencia académica |

---

## 7.3 Comparativa de Métodos de Trayectoria

Evaluados para el movimiento APPROACH → PICK (Z: 210 mm → 160 mm, tf = 2 s):

| Criterio | Polinomio Quíntico | Trapezoidal |
|---|:---:|:---:|
| **Suavidad** | ✅ Continua hasta $$\ddot{s}$$ | 🔶 Discontinua en $$\ddot{s}$$ |
| **Aceleración máxima** | 65 mm/s² | 40 mm/s² (configurable) |
| **Jerk** | ✅ Continuo | ❌ Impulsos en $$t_a, t_d$$ |
| **Tiempo de ciclo** | 2.0 s (tf fijo) | 1.6 s (alcanza v_max) |
| **Velocidad máxima** | 37.5 mm/s | 50 mm/s |
| **Posición en t=0 y t=tf** | ✅ Exacta | ✅ Exacta |
| **Velocidad en extremos** | ✅ = 0 | ✅ = 0 |
| **Aceleración en extremos** | ✅ = 0 | ❌ Discontinuidad |
| **Vibración mecánica** | ✅ Mínima | 🔶 Posible |
| **Aplicación ideal** | Pick y Place de precisión | Transporte rápido entre zonas |

---

## 7.4 Error de Detección Visual

Se midió el error de localización de cubos comparando la posición detectada por visión con la posición real (medida con calibrador digital):

### Antes de Calibración ArUco

| Prueba | X real (mm) | X detectada (mm) | Y real (mm) | Y detectada (mm) | Error (mm) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | 200.0 | 218.3 | -300.0 | -284.7 | 23.9 |
| 2 | -150.0 | -132.4 | -250.0 | -238.1 | 21.4 |
| 3 | 0.0 | 8.2 | -300.0 | -291.5 | 11.9 |
| 4 | 300.0 | 324.7 | -150.0 | -136.2 | 28.3 |
| **Media** | — | — | — | — | **21.4 mm** |

### Después de Calibración ArUco

| Prueba | X real (mm) | X detectada (mm) | Y real (mm) | Y detectada (mm) | Error (mm) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | 200.0 | 201.3 | -300.0 | -298.7 | 1.8 |
| 2 | -150.0 | -151.2 | -250.0 | -251.4 | 1.8 |
| 3 | 0.0 | 0.7 | -300.0 | -299.1 | 1.1 |
| 4 | 300.0 | 301.8 | -150.0 | -151.3 | 2.2 |
| **Media** | — | — | — | — | **1.7 mm** |

> **Mejora:** La calibración ArUco reduce el error de localización de **21.4 mm a 1.7 mm**, una mejora del **92%**. El error residual de 1.7 mm proviene principalmente de imprecisiones en la detección del centroide y la distorsión óptica residual.

---

## 7.5 Conclusiones

### Cinemática

1. **La IK analítica es la solución óptima** para pick and place industrial con el UR3. Ofrece exactitud numérica, velocidad < 1 ms, y selección explícita de configuración articular.

2. **El método Levenberg-Marquardt** es valioso como referencia y fallback, pero no adecuado para tiempo real en ciclos de alta frecuencia.

3. **Los polinomios quínticos** garantizan la continuidad de aceleración necesaria para evitar vibraciones en el manipulador, a costa de un tiempo de ciclo ligeramente mayor respecto al perfil trapezoidal.

### Visión Artificial

4. **La calibración ArUco es esencial:** sin ella, el error de localización (21.4 mm) supera la tolerancia del gripper. Con calibración, el error cae a 1.7 mm, perfectamente manejable con el Soft Gripper.

5. **Los filtros geométricos** (área, solidez, aspecto, vértices) son suficientes para distinguir cubos de otros objetos en el área de trabajo. La solidez > 0.85 es el filtro más discriminante.

### Sistema Integrado

6. **El modo SORT** demostró la capacidad del sistema para operar de forma completamente autónoma, clasificando hasta 6 cubos desordenados sin intervención humana.

7. **El waypoint de rodeo** para la zona de barras laterales resuelve de forma elegante las restricciones cinemáticas del espacio de trabajo sin reprogramar la IK.

8. **La arquitectura multihilo** (detección visual en hilo separado) permite que el robot continúe en movimiento mientras la cámara actualiza la localización de los cubos, reduciendo el tiempo de ciclo total.

---

[← Visión Artificial](../06-vision-artificial) &nbsp;&nbsp; [Código Fuente →](../08-codigo-fuente){: .btn .btn-outline }
