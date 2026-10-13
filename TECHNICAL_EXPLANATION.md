# Comprehensive Technical Explanation: 3-DOF Battery-Aware Model Predictive Speed and Path Control for USVs

---

## 1. 3-DOF Marine Vessel Hydrodynamics

The motion of an Autonomous Surface Vehicle (USV) operating on the horizontal water surface is governed by the 3-Degrees-of-Freedom (Surge, Sway, Yaw) nonlinear equations of motion (Fossen 2011):

$$\boldsymbol{\dot{\eta}} = \mathbf{R}(\psi) \boldsymbol{\nu}$$
$$\mathbf{M} \boldsymbol{\dot{\nu}} + \mathbf{C}(\boldsymbol{\nu}) \boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu}_r) \boldsymbol{\nu}_r = \boldsymbol{\tau}_{ctrl} + \boldsymbol{\tau}_{env}$$

Where:
- $\boldsymbol{\eta} = [x, y, \psi]^T$ = Global NED position and heading angle $[m, m, rad]$.
- $\boldsymbol{\nu} = [u, v, r]^T$ = Body-fixed surge velocity, sway velocity, and yaw rate $[m/s, m/s, rad/s]$.
- $\boldsymbol{\nu}_r = [u - u_c, v - v_c, r]^T$ = Relative velocity to surrounding water current.
- $\mathbf{R}(\psi)$ = 3-DOF Kinematic transformation matrix:
  $$\mathbf{R}(\psi) = \begin{bmatrix} \cos(\psi) & -\sin(\psi) & 0 \\ \sin(\psi) & \cos(\psi) & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

---

## 2. Mass & Inertia Matrices

The total inertia matrix $\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$ combines rigid-body mass and hydrodynamic added mass:

$$\mathbf{M}_{RB} = \begin{bmatrix} m & 0 & 0 \\ 0 & m & m x_g \\ 0 & m x_g & I_z \end{bmatrix}, \quad \mathbf{M}_A = -\begin{bmatrix} X_{\dot{u}} & 0 & 0 \\ 0 & Y_{\dot{v}} & Y_{\dot{r}} \\ 0 & N_{\dot{v}} & N_{\dot{r}} \end{bmatrix}$$

$$\mathbf{M} = \begin{bmatrix} m - X_{\dot{u}} & 0 & 0 \\ 0 & m - Y_{\dot{v}} & m x_g - Y_{\dot{r}} \\ 0 & m x_g - N_{\dot{v}} & I_z - N_{\dot{r}} \end{bmatrix}$$

Parameters:
- Mass $m = 180\text{ kg}$, Yaw inertia $I_z = 75\text{ kg}\cdot\text{m}^2$, Longitudinal CG offset $x_g = 0.05\text{ m}$.
- Added mass: $X_{\dot{u}} = -20\text{ kg}$, $Y_{\dot{v}} = -45\text{ kg}$, $N_{\dot{r}} = -22\text{ kg}\cdot\text{m}^2$, $Y_{\dot{r}} = N_{\dot{v}} = -1.5\text{ kg}\cdot\text{m}$.

---

## 3. Coriolis and Centripetal Effects

$$\mathbf{C}(\boldsymbol{\nu}) = \mathbf{C}_{RB}(\boldsymbol{\nu}) + \mathbf{C}_A(\boldsymbol{\nu}_r)$$

$$\mathbf{C}_{RB}(\boldsymbol{\nu}) = \begin{bmatrix} 0 & 0 & -m(x_g r + v) \\ 0 & 0 & m u \\ m(x_g r + v) & -m u & 0 \end{bmatrix}$$

With $a_1 = -X_{\dot{u}} u_r$, $a_2 = -Y_{\dot{v}} v_r - Y_{\dot{r}} r$:
$$\mathbf{C}_A(\boldsymbol{\nu}_r) = \begin{bmatrix} 0 & 0 & Y_{\dot{v}} v_r + Y_{\dot{r}} r \\ 0 & 0 & -X_{\dot{u}} u_r \\ -(Y_{\dot{v}} v_r + Y_{\dot{r}} r) & X_{\dot{u}} u_r & 0 \end{bmatrix}$$

---

## 4. Hydrodynamic Damping (Linear + Nonlinear Drag)

Damping opposes relative flow velocity through the water:

$$\mathbf{D}(\boldsymbol{\nu}_r) = \begin{bmatrix} -(X_u + X_{uu}|u_r|) & 0 & 0 \\ 0 & -(Y_v + Y_{vv}|v_r|) & -(Y_r + Y_{rr}|r|) \\ 0 & -(N_v + N_{vv}|v_r|) & -(N_r + N_{rr}|r|) \end{bmatrix}$$

$$\boldsymbol{\tau}_D = \mathbf{D}(\boldsymbol{\nu}_r) \boldsymbol{\nu}_r$$

---

## 5. Environmental Disturbance Forces

1. **Water Current**:
   $$u_c = V_{cx} \cos(\psi) + V_{cy} \sin(\psi), \quad v_c = -V_{cx} \sin(\psi) + V_{cy} \cos(\psi)$$
2. **Aerodynamic Wind Load**:
   $$q_{wind} = \frac{1}{2} \rho_{air} V_{rw}^2$$
   $$\boldsymbol{\tau}_{wind} = \begin{bmatrix} -q_{wind} A_{frontal} C_{dx} \cos(\gamma_w) \\ q_{wind} A_{lateral} C_{dy} \sin(\gamma_w) \\ 0.15 q_{wind} A_{lateral} L C_{dn} \sin(2\gamma_w) \end{bmatrix}$$
3. **Wave Disturbances**:
   $$\boldsymbol{\tau}_{wave}(t) = \begin{bmatrix} A_x \cos(\omega_e t + \beta_w) + 0.3 A_x \cos^2(\beta_w) \\ A_y \sin(\omega_e t + \beta_w) + 0.3 A_y \sin^2(\beta_w) \\ M_w \sin(\beta_w) \cos(\omega_e t + \pi/4) \end{bmatrix}$$

---

## 6. Differential Electric Propulsion & Battery Model

1. **Thruster Allocation**:
   $$T_L = \frac{T}{2} - \frac{N}{B}, \quad T_R = \frac{T}{2} + \frac{N}{B}$$
   $$\boldsymbol{\tau}_{ctrl} = [T_L + T_R, \; 0, \; (T_R - T_L) \frac{B}{2}]^T$$
2. **Propulsion & Total Electrical Power**:
   $$P_L = \frac{|T_L| \max(|u|, 0.1)}{\eta_{motor}} + k_{loss} T_L^2, \quad P_R = \frac{|T_R| \max(|u|, 0.1)}{\eta_{motor}} + k_{loss} T_R^2$$
   $$P_{batt} = P_L + P_R + P_{hotel}$$
3. **Coulomb Counting & SOC Dynamics**:
   $$\text{SOC}(t) = \text{SOC}_0 - \left[ \frac{\int_0^t P_{batt}(\tau) d\tau}{E_{capacity}} \right] \times 100\%$$

---

## 7. Line-of-Sight (LOS) Path Following Guidance

For an active waypoint line from $P_k = (x_k, y_k)$ to $P_{k+1} = (x_{k+1}, y_{k+1})$:
- Path Orientation: $\alpha_k = \text{atan2}(y_{k+1} - y_k, x_{k+1} - x_k)$
- Cross-track error (perpendicular distance):
  $$e_y = -\sin(\alpha_k)(x - x_k) + \cos(\alpha_k)(y - y_k)$$
- Desired LOS Heading:
  $$\psi_{ref} = \alpha_k + \text{atan2}(-e_y, \Delta_{LOS})$$

---

## 8. Battery-Aware 3-DOF Model Predictive Control (MPC)

At every control timestep $T_s = 0.25\text{ s}$, the MPC solves a constrained Quadratic Program (QP) over prediction horizon $N_p = 20$ and control horizon $N_c = 6$:

$$\min_{\mathbf{T}, \mathbf{N}} \sum_{i=1}^{N_p} \left[ w_u (u(k+i) - u_{ref})^2 + w_\psi (\psi(k+i) - \psi_{ref})^2 + w_r r(k+i)^2 + w_{cte} e_y(k+i)^2 \right] + \sum_{j=1}^{N_c} \left[ w_{\Delta T} (\Delta T(j))^2 + w_{\Delta N} (\Delta N(j))^2 + w_{power} P_{pred}(j) \right]$$

Subject to physical constraints:
$$0 \le T(j) \le T_{max}, \quad |N(j)| \le N_{max}$$
$$|\Delta T(j)| \le \Delta T_{max}, \quad |\Delta N(j)| \le \Delta N_{max}$$
$$\text{SOC}_{pred}(k+N_p) \ge \text{SOC}_{reserve}$$

---

---

## 9. Machine Learning Adaptive Gain Scheduling (ANN + Adam Optimizer)

To provide dynamic, real-time optimal parameter tuning under varying vessel states, environmental drag, and battery depletion levels, an **Artificial Neural Network (ANN / Multi-Layer Perceptron)** trained with the **Adam Optimizer** is deployed:

### 1. Neural Network Architecture (6 -> 32 -> 16 -> 13)
* **Input Layer (6 Features)**:
  $$\mathbf{x} = [u_{meas}, u_{ref}, e_\psi, r_{meas}, \text{SOC}, e_y]^T$$
* **Hidden Layer 1**: 32 Neurons with Rectified Linear Unit ($\text{ReLU}$) activation:
  $$\mathbf{a}_1 = \max(0, \mathbf{W}_1 \mathbf{x} + \mathbf{b}_1)$$
* **Hidden Layer 2**: 16 Neurons with Hyperbolic Tangent ($\tanh$) activation:
  $$\mathbf{a}_2 = \tanh(\mathbf{W}_2 \mathbf{a}_1 + \mathbf{b}_2)$$
* **Output Layer (13 Outputs)**:
  $$\hat{\mathbf{y}} = \mathbf{W}_3 \mathbf{a}_2 + \mathbf{b}_3$$
  * Outputs $1\dots 6$: Adaptive PID gains $[K_{p,u}, K_{i,u}, K_{d,u}, K_{p,\psi}, K_{i,\psi}, K_{d,\psi}]$
  * Outputs $7\dots 13$: Adaptive MPC weights $[w_{speed}, w_{cte}, w_{\psi}, w_{yaw}, w_{power}, w_{rate\_T}, w_{rate\_N}]$

### 2. Adam Optimizer (Adaptive Moment Estimation)
Trained using mini-batch gradient descent with momentum and adaptive learning rates:
$$m_t = \beta_1 m_{t-1} + (1 - \beta_1) g_t, \quad v_t = \beta_2 v_{t-1} + (1 - \beta_2) g_t^2$$
$$\hat{m}_t = \frac{m_t}{1 - \beta_1^t}, \quad \hat{v}_t = \frac{v_t}{1 - \beta_2^t}$$
$$\theta_{t+1} = \theta_t - \frac{\alpha}{\sqrt{\hat{v}_t} + \epsilon} \hat{m}_t$$
where learning rate $\alpha = 0.005$, decay factors $\beta_1 = 0.9, \beta_2 = 0.999$, and $\epsilon = 10^{-8}$.


