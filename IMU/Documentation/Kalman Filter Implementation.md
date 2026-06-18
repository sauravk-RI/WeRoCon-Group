# How to Implement a Two-State Kalman Filter for MPU9250/MPU6050

## Introduction

This project implements a two-state linear Kalman Filter for orientation estimation using IMU data. The filter estimates both the angle and gyroscope bias simultaneously, making it significantly more accurate than simple gyro integration or a single-state Kalman Filter.

The implementation is based on the Kalman Filtering concepts studied from a reference book covering:

* State Space Models
* State Prediction
* Covariance Propagation
* Process Noise Modeling
* Measurement Updates
* Kalman Gain Computation
* Sensor Fusion

---

## Why a Two-State Kalman Filter?

A gyroscope provides smooth angular velocity measurements but suffers from drift due to bias.

For example:

Actual angular velocity:
```
0 deg/s
```

Gyroscope output:
```
0.5 deg/s
```

This constant offset is called **gyro bias**.

If gyro measurements are integrated without compensation, orientation estimates continuously drift over time.

To solve this problem, the Kalman Filter estimates:

1. Orientation Angle
2. Gyroscope Bias

simultaneously.

---

## State Vector

The state vector is defined as:

$$x = \begin{bmatrix} \theta \\ b \end{bmatrix}$$

where:
- $\theta$ = estimated angle
- $b$ = estimated gyroscope bias

Implementation:

```python
self.x = np.zeros((2,1))
```

```python
x[0] = angle
x[1] = gyro_bias
```

---

## State Transition Model

The angle evolves according to:

$$\theta_k = \theta_{k-1} + (\omega - b)\Delta t$$

where:
* $\omega$ = measured angular velocity
* $b$ = estimated gyro bias

The state transition matrix becomes:

$$F = \begin{bmatrix} 1 & -dt \\ 0 & 1 \end{bmatrix}$$

Implementation:

```python
F = np.array([
    [1.0, -dt],
    [0.0, 1.0]
])
```

---

## Control Matrix

Gyroscope measurements act as system input.

$$B = \begin{bmatrix} dt \\ 0 \end{bmatrix}$$

Implementation:

```python
B = np.array([
    [dt],
    [0.0]
])
```

---

## Prediction Step

The Kalman Filter first predicts the next state.

Equation:

$$x_k^- = Fx_{k-1} + Bu$$

Implementation:

```python
self.x = F @ self.x + B * gyro_rate
```

This produces a predicted estimate of:
* Angle
* Gyroscope Bias

before receiving a measurement.

---

## Covariance Matrix

The covariance matrix represents uncertainty in the estimated states.

$$P = \begin{bmatrix} P_{\theta\theta} & P_{\theta b} \\ P_{b\theta} & P_{bb} \end{bmatrix}$$

Implementation:

```python
self.P = np.eye(2)
```

Initially:
```
P =
[
 [1, 0]
 [0, 1]
]
```

representing moderate uncertainty.

---

## Process Noise Matrix

The process noise matrix models uncertainty in the prediction model.

$$Q = \begin{bmatrix} Q_{angle} & 0 \\ 0 & Q_{bias} \end{bmatrix}$$

Implementation:

```python
Q = np.diag([
    self.Q_angle,
    self.Q_bias
])
```

Where:
- `Q_angle` models uncertainty in angle prediction
- `Q_bias` models uncertainty in bias estimation

---

## Covariance Propagation

After prediction:

$$P_k^- = FP_{k-1}F^T + Q$$

Implementation:

```python
self.P = F @ self.P @ F.T + Q
```

This step increases uncertainty because every prediction introduces possible error.

---

## Measurement Model

The accelerometer provides an angle estimate.

Measurement:

$$z = \theta_{acc}$$

The measurement matrix becomes:

$$H = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

Implementation:

```python
self.H = np.array([
    [1.0, 0.0]
])
```

The accelerometer directly measures angle but does not measure gyro bias.

---

## Residual Calculation

The residual represents the disagreement between prediction and measurement.

Equation:

$$y = z - Hx$$

Implementation:

```python
y = meas_angle - (H @ self.x)
```

If the prediction and measurement agree:
```
Residual ≈ 0
```

If they disagree:
```
Residual becomes large
```

---

## Dynamic Measurement Noise

Instead of using a fixed measurement noise covariance $R$, this implementation uses $R_{dynamic}$.

Implementation:

```python
update(gyro_rate, meas_angle, R_dynamic)
```

When the leg is stationary:
```
Accelerometer is trusted more → Small R
```

When the leg is moving rapidly:
```
Accelerometer becomes noisy → Large R
```

This improves robustness during gait analysis.

---

## Innovation Covariance

Equation:

$$S = HPH^T + R$$

Implementation:

```python
S = (H @ P @ H.T) + R_dynamic
```

This represents expected uncertainty in the residual.

---

## Kalman Gain

The Kalman Gain determines how much correction should be applied.

Equation:

$$K = PH^TS^{-1}$$

Implementation:

```python
self.K = (P @ H.T) / S
```

Interpretation:
- **Large K** → Trust measurement more
- **Small K** → Trust prediction more

---

## State Update

The state estimate is corrected using the residual.

Equation:

$$x^+ = x^- + Ky$$

Implementation:

```python
self.x = self.x + self.K * y
```

Both angle and gyro bias are corrected simultaneously.

---

## Covariance Update

Equation:

$$P^+ = (I - KH)P^-$$

Implementation:

```python
self.P = (np.eye(2) - self.K @ self.H) @ self.P
```

After incorporating a measurement, uncertainty decreases.

---

## Estimated Outputs

### Angle Estimate $\theta$

```python
self.x[0, 0]
```

### Gyroscope Bias Estimate $b$

```python
self.x[1, 0]
```

Accessed through:

```python
@property
def bias(self):
    return float(self.x[1,0])
```

---

## Advantages of This Implementation

Compared to simple gyro integration:
* Reduces drift
* Estimates gyro bias
* Improves long-term stability

Compared to a single-state Kalman Filter:
* Models sensor bias explicitly
* Produces more accurate orientation estimates
* Better suited for gait analysis applications

---

## Conclusion

A two-state Kalman Filter was implemented where the state vector consists of orientation angle and gyroscope bias. Gyroscope measurements are used during prediction, while accelerometer-derived angles are used during measurement updates. Dynamic measurement noise adaptation improves performance during periods of high motion, making the filter suitable for real-time human gait analysis and wearable sensing applications.
