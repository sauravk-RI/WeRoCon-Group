# Chapter 14 — The Complete Phrasebook: Every Control Mode from Python

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

Chapter 13 built the whole control stack — Pi, HAT, `can0`, EPICally Powerful — and then, deliberately, moved the motor in exactly **one** control mode: the Velocity Loop, wrapped in a sine wave. That was the right way to prove the plumbing. But you spent Chapters 5–11 learning *seven* control modes, and Chapter 12 taught you how to choose between them. It would be a strange ending if only one of them ever made the jump from GUI buttons to Python.

So this chapter is the **phrasebook**: one small, complete, runnable script for every control mode the AK80-9 V3.0 provides. No new theory — each recipe is a mode you already understand, now spoken from Python. Each script does one simple, visible thing on the bench, and each one is short enough to read in a minute. The plan:

- **§1** — the ground rules and the shared script skeleton (read once, it applies to every recipe);
- **§2** — the map: which EP call speaks which control mode;
- **§3** — the six Servo-mode recipes: Duty Cycle, Current Loop, Current Brake, Velocity Loop, Position Loop, Position–Velocity;
- **§4** — the MIT-mode recipe
- **§5** — a short mode-specific troubleshooting table (everything else is still Chapter 13 §6).

**Starting condition for every recipe in this chapter:** Chapter 13 works on your bench *today* — `can0` up (Ch 13 §3.4 or §3.7), `(epenv)` active (Ch 13 §4.3), and the `ep-stream-actuator` smoke test passing (Ch 13 §4.4). Motor calibrated per Chapter 4, CAN ID **100**, output shaft **bare** (the tape flag from Chapter 13 §5 earns its keep again here), E-stop within reach, and every rule of Chapter 4 §6 in force. Run the recipes **one at a time**, reading each one's warning before touching Enter.

## 1. The Ground Rules and the Shared Skeleton

### 1.1 One Motor, Two Operating Modes — Still

Chapter 13 §4.2 established the fact that governs this whole chapter's structure: the V3.0 firmware runs in one of two **operating modes** — **Servo** (the factory state) or **MIT** — and this is a *commissioning* setting, which can be set by the programmer right through the code itself, that's right, you do not need to connect your motor to the upper computer through the R-Link and set the mode to Servo or MIT manually everytime you want to switch between these modes - you can do this at the runtime from your script itself. The operating mode sets which CAN dialect the motor listens to: a Servo-mode motor executes Servo frames and **silently discards** MIT frames — and vice versa (Ch 13 §6).
Hence, if a recipe runs, prints healthy telemetry, and the shaft does absolutely nothing — you are almost certainly reading this paragraph too late. 😄

> **Note:** In this lesson, we will be doing all of our control works (including MIT mode control) solely using the Servo Mode. Because, from the Author's ***personal experience*** -> there are some bugs/limitations in this motor's firmware which emerges when we directly try to invoke the the MIT mode (Control Mode 8) code that is stored inside the motor's driver from our RPi and only send the Position, Velocity, Torque, Kp, and Kd values to the mototr driver (and the motor driver is supposed to calculate the torque required and current required all by itself) -- thuis methiod has got some serious limitations. Instead we will implement the MIT mode using the Servo Mode control in this motor. How you ask ? Stay tuned and follow the lesson to learn more !

### 1.2 The Skeleton Every Script Shares

All the recipes below are variations of ONE SKELETON (BOILER-PLATE CODE), which is Chapter 13 §5's demo stripped right to its bones. Read it once here, and the per-recipe listings will feel like what they are — i.e. one interchangeable middle section:

```python
#!/usr/bin/env python3
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID = 100                                       # your motor's CAN ID. Set it to the actual
                                                   # CAN ID of the motor you are working with (Ch 13 §4.2)

# Open the CAN bus + register the motor.
# Each call opens can0 for itself, so a second call would fight the first for the bus.

# Use this for Servo Mode:
actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

# And when SEVERAL motors share the same two CAN wires (Ch 13 §7), it is still one
# group and one line — one dict entry per motor, each with its own unique CAN ID
# and the dialect matching THAT motor's commissioned operating mode:
# actuators = ActuatorGroup.from_dict({100: 'AK80-9-V3-servo',
#                                      101: 'AK80-9-V3-servo'})

def lifecycle(code, can_id=CAN_ID):                # the firmware power-on/off frames —
    actuators.bus.send(can.Message(                # full backstory in Ch 13 §5
        arbitration_id=can_id,                     # addressed to ONE motor's plain CAN ID
        data=[0xFF] * 7 + [code],
        is_extended_id=True))

lifecycle(0xFC)                                    # wake up and STAY awake
# lifecycle(0xFC, 101)                             # multi-motor: wake EACH motor by its ID!
time.sleep(0.5)                                    # let first telemetry arrive

try:
    pass                                           # ← the main control recipe goes here

finally:                                           # runs even on Ctrl+C
    actuators.set_torque(CAN_ID, 0.0)              # 0 A → motor limp
    lifecycle(0xFD)                                # polite power-off
    # lifecycle(0xFD, 101)                         # multi-motor: power off EACH motor too

print("Done.")
```

Three reminders from Chapter 13 that stay true in every recipe. **First, the `lifecycle()` frames are not optional** — without the `0xFC` power-on, the motor obeys for about half a second and then shuts itself down mid-motion (Ch 13 §5, §6). **Second, the recipes stream their command on a `TimedLoop` metronome** rather than sending it once — *decide a target → send one frame → read telemetry → repeat* is the canonical controller shape (Ch 13 §5.2). **Third, Ctrl+C is always safe**: EP catches it, the `finally` block still runs, and the motor ends limp.

### 1.3 Two API Layers — and Why Three Recipes Look Different

Here is the one genuinely new fact of this chapter. EP's `ActuatorGroup` gives polished methods for only **three** of the six Servo control modes:

- `set_torque(id, amps)` — Current Loop,
- `set_velocity(id, rad_s, 0)` — Velocity Loop,
- `set_position(id, target, 0, 0)` — Position Loop.

For the other three — **Duty Cycle, Current Brake, and Position–Velocity** — EP ships the low-level *frame builders* (they live in `epicallypowerful.actuation.cubemars.cubemars_servo`, and you can read them: `make_duty_cycle_message`, `make_current_brake_message`, `make_position_velocity_mode_message`) but no `ActuatorGroup` method on top. So those recipes import the builder and hand its frame to `actuators.bus.send(...)` — the exact same trick the `lifecycle()` helper already plays.

Do not let that intimidate you; it is a gift. You already know these frames byte-for-byte from Chapters 5, 7, and 10 — using the builders directly is Part III knowledge cashing in. One caution comes with the territory, flagged where it bites (§3.6): the builders write your numbers **straight into the wire fields**, so *you* must supply them in the wire units the manual defined — no unit conversion happens for you.

## 2. The Map: Mode → EP Call → Wire

Bookmark this table; it is the chapter in one screen.

| Motor control mode | Learned in | Operating mode | How you call it in EP | Wire frame (Control Mode ID) | Command unit at your fingertips |
|---|---|---|---|---|---|
| Duty Cycle | Ch 5 | Servo | `make_duty_cycle_message(...)` + `bus.send` | 0 | duty fraction (0.04 = 4%) |
| Current Loop | Ch 6 | Servo | `actuators.set_torque(id, i)` | 1 | **amperes** (⚠ not N·m — × 0.5701 N·m/A, Ch 2 §6) |
| Current Brake | Ch 7 | Servo | `make_current_brake_message(...)` + `bus.send` | 2 | amperes (holding magnitude) |
| Velocity Loop | Ch 8 | Servo | `actuators.set_velocity(id, v, 0)` | 3 | rad/s at the output shaft (EP does the ×189, Ch 3 §2) |
| Position Loop | Ch 9 | Servo | `actuators.set_position(id, p, 0, 0)` | 4 | rad (or degrees with `degrees=True`) — ⚠ max-speed travel |
| Set Origin | Ch 4 §1 | Servo | `actuators.zero_encoder(id)` | 5 | ⚠ **permanent** flash write — commissioning only, never demoed here (Ch 13 §4.5) |
| Position–Velocity | Ch 10 | Servo | `make_position_velocity_mode_message(...)` + `bus.send` | 6 | position in degrees; speed/accel in **wire counts of 10 ERPM** (§3.6!) |
| MIT Force Control | Ch 11 | Servo | `actuators.set_torque(id, i)`  | 1 | **amperes** |

## 3. The Six Servo Recipes

Each recipe below is a **complete file** — copy it whole into `~/ak80-9/`, activate `(epenv)`, run it with `python <name>.py`, and watch the shaft. Only the middle section changes from script to script; the skeleton of §1.2 is repeated verbatim so that every file stands alone.

### 3.1 Duty Cycle Mode — the Open-Loop Handshake (Ch 5)

The simplest sentence the motor speaks: a fraction of the bus voltage, no feedback loop regulating anything. The task: 4% duty one way for two seconds, 4% the other way for two seconds, stop. Remember Chapter 5's character sketch — with nothing regulating speed, an unloaded shaft settles wherever electrical physics takes it, so keep the duty small and the run short.

```python
#!/usr/bin/env python3
"""duty_cycle_demo.py — Ch 5's Duty Cycle Mode from Python (Servo mode, Control Mode ID 0)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop
from epicallypowerful.actuation.cubemars.cubemars_servo import make_duty_cycle_message

CAN_ID = 100
DUTY   = 0.04     # 4% of bus voltage — gentle, visible spin
RUN_S  = 4.0      # 2 s forward + 2 s reverse # Total time for which we are gonna operate our motor

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Duty-cycle demo: ±{DUTY:.0%} for {RUN_S:.0f}s. Ctrl+C stops.\n")

clock = TimedLoop(rate=50)     # refresh the command 50×/s  # control frequency
t0 = time.perf_counter()  # recording the exact starting time
i = 0
try:

    # Note: This complete while block is gonna run 50 times every second.  
    while clock():                          # while the clock is running
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        duty = DUTY if t < RUN_S / 2 else -DUTY        # flip halfway
        actuators.bus.send(make_duty_cycle_message(CAN_ID, duty))
        if i % 25 == 0:               # print twice a second
            print(f"t={t:4.1f}s  duty={duty:+.2f}  "
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s  "
                  f"|i|={abs(actuators.get_torque(CAN_ID)):4.2f} A")
        i += 1
        
finally:
    actuators.bus.send(make_duty_cycle_message(CAN_ID, 0.0))  # drive off
    time.sleep(0.3)
    actuators.set_torque(CAN_ID, 0.0)                          # 0 A → limp
    lifecycle(0xFD)

print("Done.")
```

**What you should observe.** The shaft spins up to a modest steady speed, reverses at the two-second mark, and coasts to limp at the end. Watch the `vel` column: unlike every closed-loop recipe below, *nothing* is holding that number anywhere — brush the flag lightly with a fingertip (gently!) and the speed sags, because no controller is fighting for it. That sag **is** the lesson of Chapter 5.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video16.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.1: Motor shaft action at Duty-Cycle = 0.04 for 4 seconds.</i>
      </p>
</figure>
</div>

### 3.2 Current Loop Mode — Commanding the Push (Ch 6)

One `ActuatorGroup` call, one big caution. The call is `set_torque` — but in the Servo dialect its argument is **current in amperes, not newton-metres** (Ch 13 §4.5). Multiply by 0.5701 N·m/A when you want to know the output torque (Ch 2 §6). And Chapter 6 §5's runaway warning applies word for word: a constant current on an unloaded shaft is a constant *push*, and a constant push means the shaft **accelerates** for as long as you hold it. So this recipe holds a small current for a deliberately short burst.

```python
#!/usr/bin/env python3
"""current_loop_demo.py — Ch 6's Current Loop from Python (Servo mode, Control Mode ID 1)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID = 100
AMPS   = 0.6                  # ≈ 0.34 N·m at the output (× 0.5701) — small on purpose
RUN_S  = 6                  # a short run of 6s: the shaft is ACCELERATING the whole time

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Current-loop demo: {AMPS:.1f} A for {RUN_S:.1f}s — expect steady acceleration.\n")

clock = TimedLoop(rate=100)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        actuators.set_torque(CAN_ID, AMPS)             # amperes, not N·m!
        if i % 10 == 0:
            print(f"t={t:4.2f}s  i_cmd={AMPS:.2f} A  "
                  f"vel={actuators.get_velocity(CAN_ID):+6.2f} rad/s   <- climbing!")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)                  # push off → shaft coasts, then limp
    lifecycle(0xFD)

print("Done.")
```

**What you should observe.** The `vel` column climbs the whole run — there is no target speed, only a target *push*. At 0 A the shaft coasts freely down on friction alone. If you experiment further, Chapter 6 §5's rule stands: unloaded-bench currents to about ±1 A, short bursts only.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video17.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.2: Motor shaft action at Current = 0.6 A Control Loop for 6 seconds.</i>
      </p>
</figure>
</div>

### 3.3 Current Brake Mode — the Electrical Handhold (Ch 7)

The mirror image of §3.2: a current spent *refusing* motion instead of creating it. The task: hold the shaft with a 1.5 A brake for six seconds while you (gently!) test the hold by hand, then release.

```python
#!/usr/bin/env python3
"""current_brake_demo.py — Ch 7's Current Brake from Python (Servo mode, Control Mode ID 2)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop
from epicallypowerful.actuation.cubemars.cubemars_servo import make_current_brake_message

CAN_ID  = 100
BRAKE_A = 20                 # holding current, amperes (0–60 A range; stay tiny)
RUN_S   = 6.0

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Brake demo: {BRAKE_A:.1f} A hold for {RUN_S:.0f}s — "
      "gently try turning the flag by hand, then let go.\n")

clock = TimedLoop(rate=50)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        actuators.bus.send(make_current_brake_message(CAN_ID, BRAKE_A))
        if i % 25 == 0:
            print(f"t={t:4.1f}s  pos={actuators.get_position(CAN_ID):+6.2f} rad  "
                  f"temp={actuators.get_temperature(CAN_ID):3.0f} °C")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)                  # brake released → limp
    lifecycle(0xFD)

print("Done.")
```

**What you should observe.** During the hold, the shaft resists your hand with a springless, syrupy firmness and mostly stays put; the moment the script ends, the same shaft turns freely. Two safety notes ride along. Test the hold with **fingertips on the flag only**, motor firmly mounted, nothing else near the shaft. And keep an eye on the `temp` column — Chapter 7 §5's lesson is that braking current is pure heat at standstill, which is exactly why this demo lasts six seconds and not six minutes.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video18.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.3: Motor shaft action in Braking Current = 20 A Control Loop for 6 seconds.</i>
      </p>
</figure>
</div>

### 3.4 Velocity Loop Mode — the Plain Version (Ch 8)

Chapter 13 §5 already gave this mode the full ceremony — a sine wave, 100 Hz, telemetry commentary. So this recipe is deliberately the *plain* version, for symmetry with its neighbours: cruise at +2 rad/s for 5 seconds, −2 rad/s for 5 seconds, stop. Note the units luxury: you write **rad/s at the output shaft**, and EP performs Chapter 3 §2's ×189 ERPM conversion for you.

```python
#!/usr/bin/env python3
"""velocity_loop_demo.py — Ch 8's Velocity Loop from Python (Servo mode, Control Mode ID 3)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID = 100
SPEED  = 2.0                  # rad/s at the output shaft (≈ 19 rpm)
RUN_S  = 10.0                  # 5s forward + 5s reverse

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Velocity demo: ±{SPEED:.0f} rad/s cruise. Ctrl+C stops.\n")

clock = TimedLoop(rate=100)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        v = SPEED if t < RUN_S / 2 else -SPEED
        actuators.set_velocity(CAN_ID, v, 0)           # third arg unused in Servo — pass 0
        if i % 10 == 0:
            print(f"t={t:4.1f}s  v_cmd={v:+4.1f}  "
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s  "
                  f"|i|={abs(actuators.get_torque(CAN_ID)):4.2f} A")
        i += 1
finally:
    actuators.set_velocity(CAN_ID, 0.0, 0)             # controlled stop first...
    time.sleep(0.3)
    actuators.set_torque(CAN_ID, 0.0)                  # ...then limp
    lifecycle(0xFD)

print("Done.")
```

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video19.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.4: Motor shaft action at Velocity = 2 rad/s Control Loop for 10 seconds (5s forward+ 5 seconds reverse).</i>
      </p>
</figure>
</div>

### 3.5 Position Loop Mode — Handle with Caution (Ch 9)

> ### ⚠️ CHAPTER 9's MAXIMUM-SPEED WARNING, STILL IN FORCE
>
> **In Position Loop Mode the motor travels to every target at maximum speed and maximum acceleration** — that is how the mode works, not a setting (Ch 9 §1). And the command is **absolute**: the length of the sprint is the distance from wherever the shaft *is* to the target you name. This recipe stays safe by doing what Chapter 9 taught: it **reads the current position first** and commands a target only 30° away. Do not raise that number until the small version has run cleanly, and never run this recipe with anything attached to the shaft.

```python
#!/usr/bin/env python3
"""position_loop_demo.py — Ch 9's Position Loop from Python (Servo mode, Control Mode ID 4)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID  = 100
STEP_DEG = 30.0               # a NEARBY target 
HOLD_S   = 3.0                # hold the position for 3s

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)

home = actuators.get_position(CAN_ID, degrees=True)    # WHERE IS THE SHAFT? Ask first!
target = home + STEP_DEG
print(f"Position demo: {home:+.1f}° -> {target:+.1f}° -> back. Small on purpose.\n")

clock = TimedLoop(rate=50)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > 2 * HOLD_S:
            break
        goal = target if t < HOLD_S else home          # out, then back
        actuators.set_position(CAN_ID, goal, 0, 0, degrees=True)   # gains unused in Servo — pass 0, 0
        if i % 25 == 0:                                            
            print(f"t={t:4.1f}s  goal={goal:+7.1f}°  "
                  f"pos={actuators.get_position(CAN_ID, degrees=True):+7.1f}°")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)                  # release the hold → limp
    lifecycle(0xFD)

print("Done.")
```

**What you should observe.** A snap. The 30° trip happens so fast it reads as a single tick of the flag, then the shaft *holds* there — push it gently and it pushes back hard, because a position loop with an error is a motor with a mission. At `t = 3 s` it snaps home. Also notice what repeating the same command does: nothing — the loop streams `goal` fifty times a second, and forty-nine of those frames ask for a place the shaft already occupies. Absolute targets name *places*, not *distances* (Ch 9 §1).

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video20.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.5: Motor shaft action at Position = 30 rad Control Loop for 3 seconds.</i>
      </p>
</figure>
</div>

### 3.6 Position–Velocity Mode — the Civilised Journey (Ch 10)

Same destination as §3.5, but now *you* choose the cruise speed and the acceleration — Chapter 10's trapezoidal profile, and the reason this mode won Chapter 12's recommendation for real positioning work. This recipe is also where the low-level builder demands respect:

> ### ⚠️ WIRE UNITS AHEAD
>
> `make_position_velocity_mode_message(id, position, velocity, acceleration)` writes your speed and acceleration numbers **straight into the frame's int16 fields** — and Chapter 10 §5 defined those fields as **1 count = 10 ERPM** and **1 count = 10 ERPM/s²**. No ÷10 happens for you (position, by contrast, *is* converted — you pass plain degrees). The recipe below wraps the call in a tiny helper so that you think in ERPM and the helper does the ÷10 — copy that habit. Get this wrong by the factor of ten and your stately 4,000 ERPM cruise becomes either an invisible crawl or a 40,000 ERPM lunge.

```python
#!/usr/bin/env python3
"""pos_vel_demo.py — Ch 10's Position–Velocity Mode from Python (Servo mode, Control Mode ID 6)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop
from epicallypowerful.actuation.cubemars.cubemars_servo import make_position_velocity_mode_message

CAN_ID    = 100
TRAVEL_DEG = 270.0           
SPEED_ERPM = 4000             # ≈ 21 rpm ≈ 2.2 rad/s at the shaft (÷189, Ch 3 §2)
ACC_ERPM_S = 8000             # gentle ramps (≈ 42 shaft rpm gained per second)
LEG_S      = 3.0              # time budget per leg of the trip

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

def send_pos_vel(pos_deg, speed_erpm, acc_erpm_s2):
    """Engineering units in; wire counts (1 count = 10 ERPM) out. Ch 10 §5."""
    actuators.bus.send(make_position_velocity_mode_message(
        CAN_ID, pos_deg, int(speed_erpm / 10), int(acc_erpm_s2 / 10)))

lifecycle(0xFC)
time.sleep(0.5)

home = actuators.get_position(CAN_ID, degrees=True)    # absolute targets: ask first
target = home + TRAVEL_DEG
print(f"Pos–Vel demo: {home:+.1f}° -> {target:+.1f}° at {SPEED_ERPM} ERPM, then back.\n")

clock = TimedLoop(rate=50)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > 2 * LEG_S:
            break
        goal = target if t < LEG_S else home           # out, then back
        send_pos_vel(goal, SPEED_ERPM, ACC_ERPM_S)     # speed = positive cruise magnitude
        if i % 25 == 0:
            print(f"t={t:4.1f}s  goal={goal:+7.1f}°  "
                  f"pos={actuators.get_position(CAN_ID, degrees=True):+7.1f}°  "
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)
    lifecycle(0xFD)

print("Done.")
```

**What you should observe.** Run this back-to-back with §3.5 — that is the entire point of the pairing. Same kind of destination, opposite personality: the flag *ramps up*, cruises one calm turn to 270 degree from current position at about 21 rpm, ramps down, dwells, and glides home. The `vel` column tells the trapezoid story in numbers — up, flat, down — where §3.5's version was a single spike. This before/after is Chapter 10's argument made physical.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video21.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.6: Motor shaft action at Position = 270 rad, Speed = 4000 ERPM, & Accelaration = 8000 ERPM Position-Velocity Control Loop for 6 seconds (3 secinds to go and 3 seconds to come back).</i>
      </p>
</figure>
</div>

> **Note:** In CubeMarsTool, Multi and Single are a selector in the Trap Control panel, and the serial protocol really does have two separate commands (COMM_SET_POS_MULTI / COMM_SET_POS_SINGLE — Ch 10 §7). But the CAN protocol's enum contains exactly seven packets (manual p. 32; Ch 6 §8's table) — and only one of them is a position–velocity frame: Control Mode ID 6, whose position field is the multi-turn ±36,000° coordinate. There is no single-turn CAN frame.

**And Set Origin, the seventh frame?** It exists in EP as `actuators.zero_encoder(CAN_ID)`, but Chapter 13 §4.5's warning is repeated here on purpose: in the Servo dialect it is a **permanent** flash write inside the driver. It is a commissioning tool for when you mount the motor into a mechanism — not a per-run ritual, and not a demo. No recipe in this chapter calls it.

## 4. The MIT Recipes

### 4.1 The Same Eight Bytes, Now with Papers

Here is a small poetry of protocol. The `FF FF FF FF FF FF FF FC` frame that Chapter 13 §5 sent as a firmware power-on, carries itself now into the in MIT mode — same bytes, same plain CAN ID (and `FF…FD` is Exit Motor Mode). So the `lifecycle()` helper carries over into the MIT scripts unchanged.

### 4.2 The Signature Recipe: the Impedance Spring

If MIT mode has one demo that explains the whole idea, it is this one (*the code given below*). You can use this code for testing all the three controllers - Position, Velocity and Torque - available in the MIT Mode.

One thing worth flagging before you read the script: this recipe also uses the `Servo` mode dialect and not the `MIT` mode that is hardcoded into the motor driver (*reason stated above in the lesson*). Open it and you'll find `'AK80-9-V3-servo'` on the actuator line and `acts.set_torque` sending amperes — the exact Servo-dialect furniture of §3.2, not §4.4's. That's not a mistake or a shortcut; it's the whole point of calling this "the recipe that explains the whole idea." Rather than hand five numbers to the driver and trust its onboard math, the script computes Chapter 11's equation itself, out in the open, on every one of its `LOOP_HZ` ticks:

```
tau = DES_TORQUE + KP * (DES_POS - pos) + KD * (DES_VEL - vel)
```

— then converts that `tau` to amps and sends it exactly the way §3.2 already taught you to. Read the loop below with that in mind, and you're watching Chapter 11's master equation happen in plain Python, on hardware you already trust completely.

> **Note — impedance control vs. "MIT mode": same thing?** Yes, with one distinction of location, not law. The equation above is *impedance control* — a classic idea from control theory going back to Neville Hogan's 1985 paper (the "doorway" Chapter 11 §12 gestured at without naming it). Instead of commanding a position or a torque outright, you command a mechanical *relationship*: make the shaft behave like a spring–damper of whatever stiffness (`KP`) and damping (`KD`) you choose, centered on a moving target (`DES_POS`, `DES_VEL`), with an optional push (`DES_TORQUE`) added on top. "MIT mode" isn't a different law from that — it's the robotics community's name for one particular *application instance* of it. The **MIT Biomimetic Robotics Lab**, building the Mini Cheetah Project, put this exact equation to run inside the motor driver itself and defined the compact CAN frame (Control Mode ID 8) that carries its five numbers. So: the impedance control is the equation; MIT mode is that same equation running onboard, fed by that frame — and this below given recipe (code) is the same equation running over, on the Pi, fed by ordinary Servo-dialect current commands.

With that settled, here is the script:

```python
#!/usr/bin/env python3
"""
mit_impedance_ep.py — Impedance (MIT-law) control of the AK80-9 V3.0 from
a Raspberry Pi, through EP's Servo dialect. The law runs on the Pi:

        tau = DES_TORQUE + KP*(DES_POS - pos) + KD*(DES_VEL - vel)

and is delivered as motor current = tau / 0.5701 [A]. This is the same
equation the "MIT mode" that runs inside the driver on 
giving commands from the CubeMars Upper Computer Software
— here it runs on Python instead, 200x per second.

No value is clamped or checked by this code. YOU choose every number;
the comments below tell you the physics and the motor ratings to respect.
"""

# ============================================================================================
#  USER SETTINGS — Set Your Own Values for Position, Velocity and Torque Control Respectively.
# ============================================================================================
DES_POS    = 0.0   # [rad] target, relative to position at startup.
                    # Any value works (multi-turn OK, error is unwrapped).

DES_VEL    = 0.0    # [rad/s] velocity setpoint / feedforward.
                    # Physics cap: no-load top speed ~ 1.16 rad/s per volt
                    # of bus voltage (KV100 motor / 9:1 gear), e.g. ~28
                    # rad/s at 24 V, ~41 rad/s at the 48 V rating.

DES_TORQUE = 0.0    # [N*m] feedforward torque.
                    # AK80-9 ratings: ~9 N*m continuous (~16 A), 18 N*m
                    # peak (~32 A) for seconds only — sustained peak
                    # overheats the windings in minutes. Watch the temp
                    # column; back off above ~60 C.

KP         = 0.0    # [N*m/rad] SPRING STRENGTH. Bigger = pulls harder to
                    # the target and feels stiffer to your finger.
                    # Careful: the Pi learns the shaft's position a split
                    # second LATE (the reading travels over the wire).
                    # A very strong spring reacting to old information
                    # over-corrects, and the shaking GROWS instead of
                    # settling — like catching a ball while watching it
                    # on delayed video. Good zone here: KP = 1 to 3.
                    # Rule of thumb for this rig:
                    #     KP_max ~ LOOP_HZ / 100    (with KD ~ KP / 10)
                    # e.g. 200 Hz -> ~2-3, 500 Hz -> ~5, 1000 Hz -> ~10.
                    # It flattens at the top (the CAN HAT has a fixed few
                    # ms of delay no loop rate removes) — approach in
                    # small steps and back off at the first shake.
                    # (The manual's "up to 500" is for the math running
                    # INSIDE the motor, which reacts ~100x faster.)
                    # FORCE CEILING: the spring can only pull as hard as
                    # the power supply allows. With a PSU current limit
                    # I_lim, pushing the shaft beyond about
                    #     (I_lim * 0.5701) / KP   radians
                    # hits that limit -> supply voltage folds -> driver
                    # undervoltage-cuts and the torque VANISHES until you
                    # ease off (e.g. 3 A limit, KP=2 -> gives up at ~49
                    # deg of push). Raise the PSU limit or lower KP to
                    # move this ceiling. Not a bug — it's the weakest
                    # link in the power chain announcing itself.

KD         = 0.0   # [N*m/(rad/s)] SHOCK ABSORBER. Brakes the motion so
                    # the shaft doesn't bounce around the target.
                    # Too little -> it rings/bounces like a spring with
                    # no damper. Too much (> ~1 here) -> the speed
                    # reading's small noise gets multiplied into a
                    # buzzing, chattering motor.
                    # Easy rule: start with  KD = KP / 10.
                    # KD = 0 -> pure bouncy spring (fun to feel once;
                    # use small KP and keep the shaft free).

RUN_S      = 30.0   # [s] run time; Ctrl+C always stops earlier.

CAN_ID     = 100    # Set this value according to your motor's CAN ID.

LOOP_HZ    = 200    # how many times per second the Pi checks the shaft
                    # and corrects. More checks = fresher information =
                    # a stronger spring stays stable. 200 suits this Pi.
                    # Hardware limits of this rig (Pi 5 + MCP2515 HAT):
                    # 500 = safe maximum (also set the motor's telemetry
                    # rate to 500 Hz in CubeMarsTool to match). ~1000 =
                    # absolute ceiling, where the CAN HAT starts dropping
                    # frames (check with: ip -s link show can0).

SIGN       = +1.0   # measured polarity of THIS unit (+A -> +rad).
                    # If the motor, wiring, or mounting changes, re-run
                    # the polarity probe (velocity sign under a small
                    # current ramp) — a wrong sign turns the spring into
                    # a repeller and the shaft oscillates at the target's
                    # antipode.

KT         = 0.5701 # [N*m/A] torque per amp at the output (9:1 gear).
# ===========================================================================

import math
import os
import time
import can
from epicallypowerful.actuation import ActuatorGroup

acts = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'},
                               exit_manually=True)

def lifecycle(code):
    """This firmware needs FF..FC to wake and stay awake in
    the Servo dialect; FF..FD releases it."""
    acts.bus.send(can.Message(arbitration_id=CAN_ID,
                              data=[0xFF] * 7 + [code], is_extended_id=True))

def rel(a, b):                       # shortest signed angle a-b
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

DT = 1.0 / LOOP_HZ

try:
    lifecycle(0xFC)
    # Event-driven start: wait for FRESH telemetry AFTER the wake frame
    # (a few new frames, ~30-50 ms total), so the firmware has processed
    # FF..FC before the first command lands. (A naive timestamp check
    # passes instantly on stale pre-wake frames — bench-learned bug.)
    ts0 = acts.get_data(CAN_ID).timestamp
    t0, fresh = time.perf_counter(), 0
    while fresh < 5 and time.perf_counter() - t0 < 1.0:
        ts = acts.get_data(CAN_ID).timestamp
        if ts > ts0:
            fresh, ts0 = fresh + 1, ts
        time.sleep(0.002)
    lifecycle(0xFC)                    # re-assert wake, belt and braces
    time.sleep(0.02)

    d = acts.get_data(CAN_ID)
    raw_prev, pos_c = d.current_position, 0.0     # unwrapped position
    print(f"Impedance running ({RUN_S:.0f}s, Ctrl+C stops): "
          f"pos={DES_POS} rad, vel={DES_VEL}, tau_ff={DES_TORQUE}, "
          f"kp={KP}, kd={KD}\n")

    t0, i = time.perf_counter(), 0
    while time.perf_counter() - t0 < RUN_S:
        d = acts.get_data(CAN_ID)
        pos_c += rel(d.current_position, raw_prev)   # incremental unwrap
        raw_prev = d.current_position

        tau = (DES_TORQUE + KP * (DES_POS - pos_c)
               + KD * (DES_VEL - d.current_velocity))
        acts.set_torque(CAN_ID, SIGN * tau / KT)     # amps, unclamped

        i += 1
        if i % 100 == 0:
            print(f"t={time.perf_counter() - t0:5.1f}s  "
                  f"target={math.degrees(DES_POS):+7.1f} deg  "
                  f"actual={math.degrees(pos_c):+7.1f} deg  "
                  f"tau={tau:+5.2f} N*m  cmd={SIGN * tau / KT:+5.2f} A  "
                  f"vel={d.current_velocity:+6.2f}  "
                  f"temp={getattr(d, 'temperature', float('nan')):4.1f} C")
        time.sleep(DT)

    print("\nTime limit reached — exiting.")

except KeyboardInterrupt:
    print("\nCtrl+C — stopping.")
finally:
    # Stop path: zero current, release the firmware. If the bus died
    # mid-run, recover can0 once and retry — then it's the E-stop's job.
    for attempt in (1, 2):
        try:
            for _ in range(10):
                acts.set_torque(CAN_ID, 0.0)
                time.sleep(0.005)
            lifecycle(0xFD)
            break
        except can.CanOperationError:
            os.system('sudo /sbin/ip link set can0 down')
            os.system('sudo /sbin/ip link set can0 txqueuelen 1000 up '
                      'type can bitrate 1000000')
            time.sleep(0.3)
    try:
        acts.disable_actuators()
        acts.notifier.stop()
        acts.bus.shutdown()
    except Exception:
        pass
    print("Done. Motor limp; verify by hand before approaching.")
```

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video22.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.7: Impedance (Position) Control Mode at Desired POS = 1.57 rad, Kp = 2.0, Kd = 0.0, Des VEL = 0.0, Des TOR = 0.0. Notice the weird behaviour of the motor - The code is working fine and so is the motor; its just the question of how well you understand the importance of terms Kp and Kd here and how well you tune them. What happened here, with the damping constant Kd = 0, the rotor reacted so fast that the pi was unable to track its postion on time. By the time pi came to know that the rotor has arrived at its destination, it was already way past its intended goal. The feedback rate (200 Hz) is rthe culprit here. See the next video to know the PRO way of executing this motion.</i>
      </p>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video23.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.8: Impedance (Position) Control Mode at Desired POS = 1.57 rad, Kp = 2.0, Kd = 0.2, Des VEL = 0.0, Des TOR = 0.0.</i>
      </p>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video24.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.9: Impedance (Velocity) Control Mode at Desired POS = 0.0 rad, Kp = 0.0, Kd = 0.8, Des VEL = 5.0, Des TOR = 0.0. The rotor is moving at a constant velocity close to about 2 rad/s.</i>
      </p>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos/video25.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 14.10: Impedance (Torque) Control Mode at Desired POS = 0.0 rad, Kp = 0.0, Kd = 0.0, Des VEL = 0.0, Des TOR = 2.0. The rotor will rotate at full speed here.</i>
      </p>
</figure>
</div>


## 5. Troubleshooting, Mode by Mode

Chapter 13 §6's chain-walk table still handles everything electrical and everything about the stack. These rows are the traps specific to this chapter:

| Symptom | Most likely cause | Fix |
|---|---|---|
| Motion dies after ~half a second, LED off, telemetry freezes | the `lifecycle(0xFC)` frame never ran (Ch 13 §5, §6) | confirm it sits right after `from_dict`, power-cycle the motor, rerun |
| MIT spring feels dead (no push-back) | `KP` effectively zero, or the wake-up frame missing (§4.2) | Check if the wake up frame is written in the code before the start of the control sequence. Increase the Kp value and try again |

## Key Takeaways

- Every control mode from Chapters 5–11 is reachable from the Chapter 13 stack; this chapter is the mapping, one runnable file per mode (§2's table is the index).
- EP speaks three Servo modes through `ActuatorGroup` methods (`set_torque` in **amperes**, `set_velocity` in rad/s, `set_position` absolute-and-fast) and leaves three to its readable frame builders (Duty, Brake, Position–Velocity) plus `bus.send` — Part III knowledge, cashed in (§1.3, §3).
- The builders use **wire units**: Position–Velocity speed and acceleration go down in counts of 10 ERPM — wrap the call in a converting helper and never think in counts again (§3.6).
- The `FF…FC`/`FF…FD` lifecycle frames run in every script.

## Safety Notes

- **Chapter 4 §6 governs every powered minute** of every recipe: bare shaft for all first runs, small numbers, hands and cables clear, temperatures watched, hardware E-stop within reach — and Ctrl+C as the second line of defence, never the first.
- Run the recipes **one at a time and read the mode's warning first**. The dangerous ones announce themselves: Position Loop travels at maximum speed to an *absolute* target (§3.5's box), and constant current or constant torque on an unloaded shaft *accelerates* — Servo amps (§3.2) and MIT newton-metres (§4.4) alike. Short bursts, small values.
- Hand-testing the brake (§3.3) and the spring (§4.3) means **fingertips on the flag only**, motor firmly mounted, no loose clothing, no second person near the shaft.
- Braking and holding currents are heat at standstill — keep hold demos short and watch `get_temperature` (Ch 7 §5).
- Raise any gain, speed, current, or torque **one small step at a time**.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — §4.1 (p. 31–37: the six Servo control modes and their CAN frames — Duty ID 0 with the ×100,000 scaling; Current Loop ID 1, int32 ±60,000 ↔ ±60 A; Current Brake ID 2, 0–60 A magnitude; Velocity ID 3, int32 ERPM; Position ID 4, int32 ×10,000 ↔ ±36,000°; Set Origin ID 5; Position–Velocity ID 6 with speed/acceleration int16 at 1 LSB = 10 ERPM and 10 ERPM/s²), the basis of §2's table and §3's recipes; §4.2 (p. 37–39: the MIT / Force-Control frame under ID 8, Enter/Exit Motor Mode codes, and the AK80-9 parameter ranges ±12.56 rad / ±65 rad/s / ±18 N·m / Kp 0–500 / Kd 0–5), §4; §4.3.1 (p. 42: the telemetry upload frame behind every `get_*` call), throughout.
2. **EPICally Powerful — source code**, github.com/gatech-epic-power/epically-powerful, package `epicallypowerful` v1.0.5 on PyPI (accessed August 2026) — `actuation/cubemars/cubemars_servo.py`: the `ActuatorGroup`-level Servo methods (`set_torque` in amperes, `set_velocity` with the rad/s→ERPM conversion, `set_position` in degrees×10,000, the permanent-flag `zero_encoder`) *and* the module-level frame builders used directly in §3.1/§3.3/§3.6 (`make_duty_cycle_message` with the ×100,000 duty scaling; `make_current_brake_message` with ×1,000; `make_position_velocity_mode_message`, which packs speed and acceleration **unscaled** into the int16 wire fields — the basis of §3.6's wire-units warning); `actuation/cubemars/cubemars_v3.py`: the MIT dialect of §4 (`set_control`, `set_position(kp, kd)`, `set_velocity(kd)`, `set_torque` in N·m, all packing the ID-8 frame; and `set_control` raising `NotImplementedError` in the Servo class, §4.4); `actuation/actuator_group.py` (`from_dict` type routing, `bus` and graceful-exit behaviour every recipe relies on).
3. **EPICally Powerful — documentation**, gatech-epic-power.github.io/epically-powerful (accessed August 2026) — the *API → Actuation* pages for `ActuatorGroup`, `CubeMarsServo`, and `CubeMarsV3` (method signatures, the amperes-not-N·m Servo convention, available type strings), cited in §1.3, §3, §4.
4. **TMotorCANControl — source code**, pypi.org/project/TMotorCANControl (v1.2.6, accessed August 2026) — `servo_can.py`'s `power_on()`/`power_off()`, the origin of the `lifecycle()` frames (via Ch 13 §5), reinterpreted in §4.2 as MIT mode's documented Enter/Exit Motor Mode codes.
5. **This tutorial series** — Ch 2 §6 (0.5701 N·m/A, used in §3.2), §9 ("encodable is not operable", quoted in the Safety Notes); Ch 3 §2 (the ÷189), §6 (the master units table behind §2 and §3.6); Ch 4 §1 (Set Origin), §6 (the ground rules); Ch 5–Ch 11 (each mode's full chapter, referenced from its recipe); Ch 12 §7.2 (the gait-controller framing of §4.4); Ch 13 §§3–6 (the stack, the EP vocabulary, the `lifecycle()` backstory, and the troubleshooting chain every recipe assumes).