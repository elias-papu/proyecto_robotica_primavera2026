---
layout: default
title: Resultados
nav_order: 8
description: "Métricas de precisión, comparativas y conclusiones del proyecto"
permalink: /07-resultados/
---

# 7. Resultados
{: .no_toc }

Métricas de precisión, comparativas de métodos y conclusiones obtenidas con el sistema real.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 7.1 Precisión de la IK Analítica

La IK analítica se validó calculando la cinemática directa sobre la solución obtenida y comparando con la posición deseada:

| Posición objetivo | X err (mm) | Y err (mm) | Z err (mm) | Error total |
|---|:---:|:---:|:---:|:---:|
| Zona verde: (275, −294, 160) | < 0.001 | < 0.001 | < 0.001 | **< 10⁻³ mm** |
| Zona roja: (0, −294, 160) | < 0.001 | < 0.001 | < 0.001 | **< 10⁻³ mm** |
| Zona azul: (−275, −294, 160) | < 0.001 | < 0.001 | < 0.001 | **< 10⁻³ mm** |
| PREMOVE cubo rojo: (0, −200, 240) | < 0.001 | < 0.001 | < 0.001 | **< 10⁻³ mm** |
| Posición lejana: (350, −450, 160) | < 0.001 | < 0.001 | < 0.001 | **< 10⁻³ mm** |

> El método analítico alcanza **precisión de punto flotante de doble precisión** (∼10⁻¹² m). Es exacto, no aproximado.

---

## 7.2 Comparativa de Métodos IK

| Criterio | IK Analítica | Levenberg-Marquardt |
|---|:---:|:---:|
| **Tiempo de cómputo** | < 1 ms | 5–50 ms |
| **Error de posición** | < 10⁻³ mm | < 0.1 mm (tol=1e-4 m) |
| **Error de orientación** | Exacto | Ponderado (W) |
| **Configuraciones** | 4 ramas explícitas | Depende del punto inicial |
| **Singularidades** | ✅ Detectadas y reportadas | ✅ Amortiguadas (λ=0.05) |
| **Éxito en workspace** | ~95% directo, ~99% con cascada | ~85% desde HOME |
| **Rol en el proyecto** | Principal | Estrategia 5 (fallback) |

---

## 7.3 Comparativa de Trayectorias

| Criterio | Polinomio Quíntico | `movej` Normal |
|---|:---:|:---:|
| **Suavidad** | ✅ Continua hasta $$\ddot{q}$$ | 🔶 Controlada por UR internamente |
| **Tiempo calculado** | `calcular_tf_quintico()` | `_tiempo_movej()` (×2 margen) |
| **Parámetro `t`** | ✅ Enviado explícitamente | ❌ Solo vel/acel |
| **Uso** | Tránsitos (HOME→PREMOVE) | Cerca del cubo (APPROACH→PICK) |

---

## 7.4 Precisión de Detección Visual

| Métrica | Sin calibración ArUco | Con calibración ArUco |
|---|:---:|:---:|
| **Error medio de posición** | ~22 mm | **< 3 mm** |
| **Error máximo observado** | ~35 mm | **< 8 mm** |
| **Tasa de detección (condiciones lab.)** | 95% | 95% |
| **Falsos positivos** | ~5% | ~2% |

El error mayor después de calibración proviene de la resolución de la cámara (∼0.7 mm/px a la distancia de trabajo) y la precisión del centroide del contorno.

---

## 7.5 Rendimiento del Sistema Completo

| Métrica | Valor |
|---|---|
| **Tiempo de ciclo pick & place (1 cubo)** | 15–25 s (depende de la posición) |
| **Cubos clasificados correctamente en pruebas** | 100% (10/10 pruebas) |
| **Activaciones del waypoint de rodeo** | ~30% de los picks (zona baja del área) |
| **Activaciones de estrategia LM** | < 5% de los picks |
| **Tasa de fallo de grip** | 0% en pruebas (gripper suave robusto) |

---

## 7.6 Videos del Sistema en Operación

### Rutina completa de clasificación

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; margin: 1.5rem 0; box-shadow: 0 6px 20px rgba(0,0,0,0.12);">
  <iframe src="https://www.youtube.com/embed/3k_M8NYh3sA"
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: none;"
    allowfullscreen loading="lazy">
  </iframe>
</div>

### Vista de la cámara — detección HSV en tiempo real

<div style="position: relative; padding-bottom: 177.78%; height: 0; overflow: hidden; border-radius: 10px; margin: 1.5rem 0; box-shadow: 0 6px 20px rgba(0,0,0,0.12);">
  <iframe src="https://www.youtube.com/embed/gWFa7Z1iGTo"
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: none;"
    allowfullscreen loading="lazy">
  </iframe>
</div>

### Rutina vista frontal

<div style="position: relative; padding-bottom: 177.78%; height: 0; overflow: hidden; border-radius: 10px; margin: 1.5rem 0; box-shadow: 0 6px 20px rgba(0,0,0,0.12);">
  <iframe src="https://www.youtube.com/embed/yGHsUg1K5iI"
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: none;"
    allowfullscreen loading="lazy">
  </iframe>
</div>

--- 

## 7.7 Conclusiones

1. **La IK analítica es la elección correcta** para pick and place industrial con el UR3. Tiempo < 1 ms, exactitud numérica y selección explícita de configuración son ventajas decisivas.

2. **La cascada de 5 estrategias** aumentó la cobertura del workspace de ~70% (solo rama principal) a ~99%, eliminando fallas de IK en la práctica.

3. **La calibración ArUco es esencial:** reduce el error de localización de ~22 mm a < 3 mm, diferencia entre un sistema funcional y uno que falla constantemente.

4. **El modo SORT-BUCLE** demostró la autonomía completa del sistema: opera indefinidamente sin intervención humana, detectando y corrigiendo el desorden en cuanto aparece.

5. **La decisión de quíntico en tránsitos + movej lento en picks** resultó en un buen balance entre velocidad de ciclo y control preciso cerca del cubo.

---

[← Visión Artificial](../06-vision-artificial) &nbsp;&nbsp; [Código Fuente →](../08-codigo-fuente){: .btn .btn-outline }
