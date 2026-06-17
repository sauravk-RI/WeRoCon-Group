# IMU Orientation Measurement — Euler Angles, Quaternions & Sensor Fusion

![Topic](https://img.shields.io/badge/Topic-IMU-blue) ![Sensor](https://img.shields.io/badge/Sensor-MPU9250-orange) ![Filter](https://img.shields.io/badge/Filter-Kalman-red) ![Math](https://img.shields.io/badge/Math-Quaternion-purple) ![Board](https://img.shields.io/badge/Board-Raspberry%20Pi-c51a4a) ![Domain](https://img.shields.io/badge/Domain-Embedded%20Systems-yellow) ![Status](https://img.shields.io/badge/Status-Active-brightgreen) ![Docs](https://img.shields.io/badge/Type-Documentation-lightgrey)

A simple explanation of how devices like a Raspberry Pi + MPU-9250 figure out which way they are facing in 3D space.

## What's inside

This repo contains a documentation file (`IMU_Orientation_Sensor_Fusion.docx`) that explains:

- How orientation (roll, pitch, yaw) is measured
- Why a common method (Euler angles) breaks down in certain positions
- How a smarter method (quaternions) fixes that problem
- How a real sensor chip (MPU-9250) combines 3 sensors to get accurate readings

## Why this matters

If you're working with drones, robots, IMUs, or anything that needs to know its own orientation, you'll run into these exact concepts. This doc breaks them down without assuming prior knowledge.

## Key concepts (quick summary)

**Orientation basics**
- Every object in 3D has 3 axes: X, Y, Z
- Roll = tilt around X-axis
- Pitch = tilt around Y-axis
- Yaw = turn around Z-axis
- These 3 angles together are called Euler angles

**The problem — Gimbal Lock**
- When pitch hits 90°, the roll and yaw axes line up with each other
- Once that happens, the system can no longer tell roll and yaw apart
- Result: you lose one full direction of movement tracking
- This is called Gimbal Lock

**The fix — Quaternions**
- Instead of 3 numbers, use 4: (w, x, y, z)
- x, y, z = the axis you're rotating around
- w = how much you've rotated (the angle part)
- This 4-number system never breaks down, no matter the angle
- No gimbal lock, ever

**Why you still see Euler angles on screen**
- The processor calculates everything internally using quaternions (more stable math)
- But quaternions are hard for humans to read
- So it converts the result back to roll/pitch/yaw before displaying it
- Same idea as a calculator working in binary but showing you decimal numbers

## How the MPU-9250 sensor actually works

The MPU-9250 has 3 sensors packed into one chip. Each one measures something different:

| Sensor | What it measures | Good at | Bad at |
|---|---|---|---|
| Gyroscope | Speed of rotation (rad/s) | Fast, short-term changes | Drifts over time |
| Accelerometer | Gravity direction (m/s²) | Steady, long-term roll & pitch | Gets confused during fast movement/vibration |
| Magnetometer | Earth's magnetic field | Long-term yaw (compass direction) | Gets confused near metal/magnets |

**The core problem:** none of these 3 sensors is accurate on its own.

- Gyroscope alone → drifts and slowly becomes wrong
- Accelerometer alone → can't tell yaw at all, and shakes mess it up
- Magnetometer alone → can't tell roll/pitch, and interference messes it up

## The fix — Sensor Fusion

- Combine all 3 sensors instead of relying on just one
- Use a math tool called a **Kalman Filter** to blend them smartly
- The filter constantly asks: "which sensor is most trustworthy right now?"
- It gives more weight to whichever sensor is more reliable at that exact moment

**Simple example of how trust shifts:**
- Device shaking fast → trust gyroscope more, trust accelerometer less
- Device sitting still → trust accelerometer more
- Near a magnet/motor → trust magnetometer less, rely on gyroscope for yaw instead

- This blending happens continuously, many times per second
- The output is one final, accurate roll/pitch/yaw reading

## One-line summary

> Gyroscope is fast but drifts. Accelerometer and magnetometer are stable but get fooled easily. Quaternions avoid gimbal lock in the math, and the Kalman filter blends all 3 sensors so their weaknesses cancel out — giving an accurate, stable orientation reading.

## File

- `IMU_Orientation_Sensor_Fusion.docx` — Full documentation with explanations, diagrams, and formulas

## Tags

`imu` `mpu9250` `sensor-fusion` `kalman-filter` `quaternion` `quaternions` `euler-angles` `gimbal-lock` `raspberry-pi` `embedded-systems` `embedded-systems-engineering` `robotics` `gyroscope` `accelerometer` `magnetometer` `9-axis` `orientation-estimation` `attitude-estimation` `drone` `motion-tracking` `arduino` `inertial-navigation` `ahrs` `complementary-filter` `documentation` `electronics`
