---
layout: default
title: Método Analítico
parent: Cinemática Inversa
nav_order: 1
description: "IK analítica real del proyecto — calcular_ik_raw() paso a paso"
permalink: /04-cinematica-inversa/04a-metodo-geometrico/
---

# 4a. Método Analítico — `calcular_ik_raw()`
{: .no_toc }

Derivación completa de los 6 ángulos del UR3. Código real de `inverse_kinematics.py`.
{: .fs-5 .fw-300 }

## Tabla de Contenidos
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 4a.1 Derivación de los 6 Ángulos

### Cálculo de $$q_1$$

Se proyecta la posición de la muñeca sobre el plano XY. La muñeca está desplazada del TCP en $$d_6$$ a lo largo del vector de aproximación:

$$
q_1 = \text{atan2}(p_y - a_y d_6,\; p_x - a_x d_6) - \text{atan2}(-d_4,\; \pm\sqrt{r^2 - d_4^2})
$$

### Cálculo de $$q_5$$

$$
q_5 = \text{atan2}\!\left(\sqrt{(-s_1 n_x + c_1 n_y)^2 + (-s_1 o_x + c_1 o_y)^2},\; s_1 a_x - c_1 a_y\right)
$$

### Cálculo de $$q_6$$

$$
q_6 = \text{atan2}\!\left(\frac{-s_1 o_x + c_1 o_y}{\sin q_5},\; \frac{s_1 n_x - c_1 n_y}{\sin q_5}\right)
$$

### Cálculo de $$q_{234}$$

$$
q_{234} = \text{atan2}\!\left(\frac{-a_z}{\sin q_5},\; \frac{-(c_1 a_x + s_1 a_y)}{\sin q_5}\right)
$$

### Cálculo de $$q_2$$ — Discriminante cuadrático

$$
B_1 = c_1 p_x + s_1 p_y - d_5 \sin q_{234} + d_6 \cos q_{234} \sin q_5
$$
$$
B_2 = p_z - d_1 + d_5 \cos q_{234} + d_6 \sin q_{234} \sin q_5
$$
$$
q_2 = \text{atan2}(B_2, B_1) - \text{atan2}\!\left(C_c,\; \pm\sqrt{A_c^2 + B_c^2 - C_c^2}\right)
$$

donde $$A_c = -2B_2 a_2$$, $$B_c = 2B_1 a_2$$, $$C_c = B_1^2 + B_2^2 + a_2^2 - a_3^2$$.

### Cálculo de $$q_3$$ y $$q_4$$

$$
q_{23} = \text{atan2}\!\left(\frac{B_2 - a_2\sin q_2}{a_3},\; \frac{B_1 - a_2\cos q_2}{a_3}\right)
$$
$$
q_4 = q_{234} - q_{23}, \qquad q_3 = q_{23} - q_2 - 2\pi
$$

---

## 4a.2 Código Completo — `calcular_ik_raw()`

```python
# inverse_kinematics.py — Función principal IK analítica

def calcular_ik_raw(px_m, py_m, pz_m, rot=None,
                    rama_q2="elbow_up", rama_q1="pos"):
    """
    Calcula los 6 ángulos articulares del UR3.
    Ángulos normalizados a [-180°, 180°] para evitar saltos bruscos.
    """
    if rot is None:
        rot = _DEFAULT_ROT

    nx, ny, nz = rot["nx"], rot["ny"], rot["nz"]
    ox, oy, oz = rot["ox"], rot["oy"], rot["oz"]
    ax, ay, az = rot["ax"], rot["ay"], rot["az"]

    try:
        # ── q1 ────────────────────────────────────────────────────────────
        arg_x  = py_m - ay * _d6
        arg_y  = px_m - ax * _d6
        d4_arg = -_d4
        bajo_raiz = arg_x**2 + arg_y**2 - d4_arg**2
        if bajo_raiz < 0:
            return {"valido": False, "mensaje": "q1: fuera del workspace"}

        signo = 1 if rama_q1 == "pos" else -1
        q1 = (math.atan2(arg_x, arg_y) -
              math.atan2(d4_arg, signo * math.sqrt(bajo_raiz)))

        # ── q5 ────────────────────────────────────────────────────────────
        s5_val = math.sqrt(
            (-math.sin(q1)*nx + math.cos(q1)*ny)**2 +
            (-math.sin(q1)*ox + math.cos(q1)*oy)**2
        )
        q5 = math.atan2(s5_val, math.sin(q1)*ax - math.cos(q1)*ay)

        # ── q6 ────────────────────────────────────────────────────────────
        if abs(s5_val) < 1e-9:
            return {"valido": False, "mensaje": "q6: singularidad S5≈0"}

        c6_num = (-math.sin(q1)*ox + math.cos(q1)*oy) / s5_val
        s6_num =  (math.sin(q1)*nx - math.cos(q1)*ny) / s5_val
        q6 = math.atan2(c6_num, s6_num)

        # ── q234 ──────────────────────────────────────────────────────────
        denom_q234 = -(math.cos(q1)*ax + math.sin(q1)*ay) / s5_val
        q234 = math.atan2(-az / s5_val, denom_q234)

        # ── B1, B2 ────────────────────────────────────────────────────────
        B1 = (math.cos(q1)*px_m + math.sin(q1)*py_m
              - _d5*math.sin(q234) + _d6*math.cos(q234)*s5_val)
        B2 = (pz_m - _d1 + _d5*math.cos(q234)
              + _d6*math.sin(q234)*s5_val)

        # ── q2 ────────────────────────────────────────────────────────────
        A_c = -2 * B2 * _a2
        B_c =  2 * B1 * _a2
        C_c = B1**2 + B2**2 + _a2**2 - _a3**2

        disc = A_c**2 + B_c**2 - C_c**2
        if disc < 0:
            return {"valido": False, "mensaje": "q2: discriminante negativo"}

        signo2 = 1 if rama_q2 == "elbow_up" else -1
        q2 = (math.atan2(B_c, A_c) -
              math.atan2(C_c, signo2 * math.sqrt(disc)))

        # ── q3, q4 ────────────────────────────────────────────────────────
        q23 = math.atan2(
            (B2 - _a2*math.sin(q2)) / _a3,
            (B1 - _a2*math.cos(q2)) / _a3
        )
        q4 = q234 - q23
        q3 = q23 - q2 - 2*math.pi   # convención del MATLAB original

        # ── Normalizar a [-π, π] ──────────────────────────────────────────
        qs = [_normalizar_rad(q) for q in [q1, q2, q3, q4, q5, q6]]

        return {
            "valido":     True,
            "mensaje":    "OK",
            "joints_deg": [math.degrees(q) for q in qs],
            "joints_rad": qs,
            "q1_deg": math.degrees(qs[0]),
            # ... (q2_deg .. q6_deg igual)
        }

    except Exception as e:
        return {"valido": False, "mensaje": f"Error: {e}"}
```

---

## 4a.3 Ejemplo — Pick de cubo en zona verde

Posición destino: $$X = 275\,\text{mm},\; Y = -294\,\text{mm},\; Z = 160\,\text{mm}$$

```python
from inverse_kinematics import calcular_ik, imprimir_ik

res = calcular_ik(0.275, -0.294, 0.160)
imprimir_ik(res, "Pick zona verde")
```

```
────────────────────────────────────────────────────────
  ✓ Cinemática Inversa UR3
────────────────────────────────────────────────────────
  q1 =   13.422°   (+0.23427 rad)
  q2 =  -91.456°   (-1.59633 rad)
  q3 =  -98.331°   (-1.71619 rad)
  q4 =  -80.213°   (-1.39995 rad)
  q5 =   90.000°   (+1.57080 rad)
  q6 =   13.422°   (+0.23427 rad)
────────────────────────────────────────────────────────
```

---

## 4a.4 Test rápido en `__main__`

```python
# Al final de inverse_kinematics.py
if __name__ == "__main__":
    res = calcular_ik(0.35, -0.10, 0.30335)
    imprimir_ik(res, "TEST: px=0.35  py=-0.10  pz=0.30335")
    print("joints_deg:", [f"{q:.2f}" for q in res.get("joints_deg", [])])
```

---

[← Cinemática Inversa](../04-cinematica-inversa) &nbsp;&nbsp; [Levenberg-Marquardt →](../04b-levenberg-marquardt){: .btn .btn-outline }
