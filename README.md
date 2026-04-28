# Modeling and Control of Cart-Pole System using Modern Control Theories

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-✓-orange.svg)](https://www.mathworks.com/products/simulink.html)

A comprehensive study and implementation of the classic **cart-pole (inverted pendulum)** problem, carried out as an advanced control project at the University of Tehran. The work includes mathematical modelling, linearisation, controllability/observability analysis, and the design of several modern controllers – both PID and state‑space based – with extensive simulation and robustness tests.

---

## 📄 Overview

The cart‑pole system is a highly non‑linear, open‑loop unstable benchmark in control engineering. In this project we:

- Derived the exact non‑linear equations of motion using the **Lagrangian** formulation.
- Linearised the system around the unstable equilibrium (pendulum upright) using **Jacobian linearisation** and **small‑angle approximations**.
- Designed and compared multiple control strategies:
  - **PID controller** (double loop for cart position and pendulum angle)
  - **State‑feedback controller** with pole placement (Ackermann’s formula)
  - **Output tracking** via static pre‑compensator and integral control
  - **Full‑order and reduced‑order observers**
- Assessed system **controllability & observability** (Kalman rank condition & PBH tests).
- Evaluated **robustness** against sinusoidal input disturbances.
- Visualised the system behaviour in **Simulink** and with a 3D model in **Simscape Multibody** (imported from SolidWorks).

A detailed report (in Persian) is included in the repository.

---

## 🎯 Control Objectives

| Specification        | Target                          |
| -------------------- | ------------------------------- |
| Settling time        | < 5 s                           |
| Rise time            | < 0.5 s                         |
| Pendulum angle deviation | < 15° (0.26 rad)               |
| Steady‑state error   | < 2%                            |
| Cart displacement    | ≤ 10 cm from origin             |

---

## 🧠 Methodology

### 1. System Modelling
- **Parameters**:  
  Cart mass `M = 2 kg`, pole mass `m = 0.2 kg`, pole length `l = 0.5 m`, moment of inertia `I = 0.099 kg·m²`, gravity `g = 9.81 m/s²`.
- Non‑linear state equations derived from the Lagrange function:

(M + m)ẍ - mlθ̈cosθ + mlθ̇²sinθ = F  
lθ̈ - g sinθ = ẍ cosθ

- These are rewritten as two explicit second‑order ODEs (see the report for full expressions).

### 2. Linearisation
Two approaches were used, both around the equilibrium `x = 0, θ = 0`:

- **Simplifying assumption**: `sinθ ≈ 0, cosθ ≈ 1` for small `θ`.
- **Jacobian matrix**: Systematic linearisation of the state‑space representation.

Resulting linear state‑space model (using Jacobian method):


A = [0  1       0         0;
     0  0   mg/M         0;
     0  0       0         1;
     0  0  -(M+m)g/(M l)  0]
B = [0; 1/M; 0; -1/(M l)]
C = [1 0 0 0;
     0 0 1 0]

### 3. Controller Design & Analysis
- **PID Control**: Two independent PID loops (cart position & pendulum angle) tuned in Simulink.
- **State Feedback**: Pole placement using Ackermann’s formula. Different speed sets tested (`slow`, `moderate`, `fast`).
- **Output Tracking**:
  - Static pre‑compensation (`u = r·[-C(A-BK)⁻¹B]⁻¹`)
  - Integral control with augmented state.
- **Observers**: Full‑order and reduced‑order Luenberger observers for state estimation.
- **Performance metrics**: Overshoot, settling time, rise time, steady‑state error, and `L²`/`L∞` norms.

All designs were validated on the **non‑linear model** in Simulink.

---

## 📊 Key Results

- **State feedback** outperforms PID in both speed and overshoot.
- Faster pole locations reduce settling time but increase control effort and initial overshoot.
- Both output tracking methods achieve zero steady‑state error; the static pre‑compensator is faster, while the integral controller is smoother.
- The system is fully **controllable** and **observable** from both cart position and pendulum angle (using the full C matrix), but observation from angle alone is rank‑deficient.
- The controller shows **robustness** to sinusoidal input noise (0.01 amplitude, 20 rad/s).

---

## 🛠 Tools & Software

| Tool                      | Purpose                                      |
| ------------------------- | -------------------------------------------- |
| MATLAB R2023a+            | Numerical computation, controller design     |
| Simulink                  | Non‑linear system simulation, PID tuning     |
| Simscape Multibody        | 3D animation and physical modelling          |
| SolidWorks                | CAD model of the cart‑pole assembly          |
| Control System Toolbox    | `place()`, `ctrb()`, `obsv()`, `eig()`, etc. |

---

## 📁 Repository Structure

Will fill this later sorry!

> **Note:** The report is written in **Persian (Farsi)**. An English abstract is provided at the end of the document.
---

## 🚀 How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/cart-pole-control.git
   cd cart-pole-control
   ```
2. Open MATLAB and navigate to the project folder.
3. Run any `.m` script to see the analysis (e.g., `linearization.m`).
4. Open the Simulink models (`.slx` files) and press **Run** to observe system responses and animations.

Make sure you have the **Control System Toolbox** and **Simscape Multibody** installed.

will make proper how to run section later sorry
---

## 📚 Reference

- Pati, Jnyana Ranjan. *“Modeling, identification and control of cart-pole system.”* (2014).  
  Used as the primary reference for the modelling approach.

---

## 👥 Authors

- **Reza Nadimi**  
- **Soheil Lak‑Ghashghaei**  

Supervisor: **Dr. Khalil Alipoor**  
University of Tehran, College of Interdisciplinary Science and Technology  
Department of Intelligent Systems, Mechatronics Engineering – Fall 1404 (2025–2026)

---

## 📃 License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---

If you have any questions or suggestions, feel free to open an issue or pull request.
