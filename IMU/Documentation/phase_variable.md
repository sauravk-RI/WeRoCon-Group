## What is a phase variable?

- A normalized 0-to-1 representation of the gait cycle, where 0 = start of stride, 1 = end of stride.
- Repeats once per gait cycle regardless of walking speed:
  - Faster walking -> same 0-to-1 progression in less time -> steeper slope vs. time
  - Slower walking -> shallower slope
- Used instead of elapsed time because gait cadence is not constant, and a prosthetic leg needs a cadence-independent way to know where it is in the cycle.

  Phase variable should idealy look like as shown below
  <img width="1672" height="800" alt="image" src="https://github.com/user-attachments/assets/65b511b5-bade-409a-8255-d344659241f5" />


## Where it comes from

- Derived from two signals: **thigh angle** and its **angular velocity**.
- Plotting these against each other over one gait cycle traces a closed, roughly elliptical loop — the **phase portrait**.
- The angular position around this loop (via `atan2`) progresses smoothly from 0 to 1 per cycle — this angular position *is* the phase variable.

## Preprocessing checklist before computing the phase variable

**1. Coordinate convention — fix this first**

Thigh angle needs a consistent zero and sign:
- **Zero** = vertical (leg straight down) or anatomical neutral — pick one and stick to it
- **Positive** = thigh swinging forward (flexion)
- **Negative** = thigh swinging backward (extension)

If the raw angle drifts or has an arbitrary zero, subtract the standing-still calibration offset first.

**2. Thigh angle must be centered around zero**

The phase portrait only forms a clean closed orbit if the signal oscillates symmetrically around zero.

Check this:
- Plot thigh angle over several strides
- The mean over a full cycle should be close to zero
- If the mean is, say, +8°, subtract it (DC offset removal)

**Why it matters:** if the signal is offset, the `atan2` circle is shifted from the origin, so phase becomes nonuniform and jumps near the offset point.

**3. Angular velocity must also be zero-centered**

Gyro drift causes a slow nonzero mean in angular velocity even during periodic gait.
- Apply a high-pass filter with a very low cutoff (~0.1 Hz) to remove gyro drift
- This doesn't affect the gait signal itself (which is ~1 Hz and above)
- This is the only filtering needed here — removing DC drift, not noise

**4. Normalization — the most critical step**

Before feeding θ and θ̇ into `atan2`, scale them so they have equal amplitude.

Define:
- A = expected peak amplitude of thigh angle (from calibration, e.g. 20°)
- B = expected peak amplitude of angular velocity (from calibration, e.g. 80°/s)

Then θ̃ = θ/A, θ̃̇ = θ̇/B — both should now oscillate between roughly -1 and +1.

**Without this**, you get an ellipse instead of a circle — phase moves fast in one part of the cycle and slow in another, so the 0-to-1 output is nonuniform.

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/40055f7c-e53d-4222-86a0-13c21b647714" />



**5. Signal must be smooth enough for the portrait to be a closed orbit**

Thigh angle over one gait cycle should look like a smooth, sinusoid-like curve — not perfectly sinusoidal, but smooth and unimodal (one peak, one trough per cycle).

Check:
- No double peaks within one stride
- No flat regions (standing still mid-cycle)
- If these exist, a state machine must detect and handle them — not `atan2`

**6. Gyro alignment**

The gyro must measure thigh angular velocity specifically in the sagittal plane.
- If the IMU is tilted or measuring the wrong axis, θ̇ from the gyro won't match dθ/dt of the angle signal
- Verify by numerically differentiating the angle signal and comparing it to the gyro reading — they should match closely after drift removal
- If they don't match, there's an axis-alignment or sign issue to fix first

**Summary checklist**

| Condition | What to do |
|---|---|
| Angle zero reference | Subtract standing calibration offset |
| DC offset in angle | Subtract per-stride mean, or high-pass filter |
| Gyro drift in velocity | High-pass filter at ~0.1 Hz |
| Amplitude normalization | Divide θ by A, θ̇ by B |
| Sign convention | Forward flexion positive; confirm gyro sign matches |
| Gyro axis alignment | Verify gyro matches numerical derivative of angle |

