# Chapter 10 — Position–Velocity Loop Mode: Multi-Turn and Single-Turn

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 11 — MIT (Force) Control Mode →](11_MIT_Force_Control_Mode.md)

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In Chapter 9, we learned something a little scary about plain Position Mode: the motor always rushes to its target at **maximum speed and maximum acceleration**. You cannot slow it down inside that mode. This chapter gives you the solution. **Position–Velocity Loop Mode** — Servo Control Mode ID 6 — asks you for three numbers instead of one: *where to go*, *how fast to travel*, and *how quickly to speed up and slow down*. The motor then makes the trip smoothly, following a shape called a **trapezoidal motion profile** (do not worry — we will explain this word in §1). The destination and the holding behavior are the same as Chapter 9. Only the journey changes: it becomes a gentle glide that *you* designed, instead of a sprint.

This mode comes in two versions. In CubeMarsTool, you will find them in the **Trap Control** section (Ch 4 §3), named **Multi Mode** and **Single Mode**. Both move the motor in exactly the same smooth way. The only difference between them is **what the position number means** — and that is easier to understand than it sounds. Here is our path through this chapter:

- what this mode adds compared to Ch 9, and what a "trapezoid" means here (§1);
- Multi-turn position: ±36,000°, where full turns are counted (§2);
- Single-turn position: 0–359°, one turn only (§3);
- a simple table comparing Multi and Single (§4);
- the units and the CAN message details (§5);
- what happens to current, load, and holding (§6);
- a small mistake in the manual that serial-port programmers should know (§7), and short comparisons with other modes (§8);

followed by two bench demos, advice on when to use this mode, safety notes, and sources. As always, we assume your motor is connected and calibrated, and that you follow Ch 4 §6's safety ground rules.

## 1. What Position–Velocity Mode Adds

First, remember Chapter 9's demo: we commanded a 90° step, and the motor snapped there at full power. Why? Because in plain Position Mode, speed and acceleration "are default to the maximum value" (manual §4.1, p. 31). The manual describes our new mode in one simple sentence: *"Position-Velocity Loop Mode: A specified position, speed, and acceleration are given to the motor"* (§4.1, p. 31). And the GUI instructions make a friendly promise (§3.3.1.1, p. 24): enter the desired position, speed, and acceleration, and *"the motor will move at the desired speed until it reaches the desired position."*

So now every command carries three numbers:

- **Position** — where to go. Its meaning depends on Multi or Single Mode (§§2–3).
- **Speed** — the travel speed, in ERPM. Remember from Ch 3 §2: divide by 189 to get the real shaft speed in rpm.
- **Acceleration** — how quickly the motor may change its speed, in ERPM/s² (Ch 3 §4). The same value is used for speeding up at the start and slowing down at the end.

Now, why do we call this a **trapezoidal profile**? Imagine drawing a graph of the motor's speed against time during one move. The motor starts from rest and speeds up along a straight slope (your acceleration). Then it travels at a constant speed (your speed) — a flat line. Finally, it slows down along another straight slope and stops exactly on the target. Slope up, flat top, slope down: the graph looks like a **trapezoid**. That is the whole meaning of the name.

One interesting special case: if the trip is very short, the motor does not have enough distance to reach your cruise speed. It is still speeding up when it must already start slowing down. In that case the flat top disappears, and the graph becomes a **triangle** instead. The motor simply never reaches the speed you asked for — and nothing is wrong. You can see both shapes for yourself on the Real-time Data speed plot: a long trip draws the trapezoid, a short trip draws the triangle.

What is happening inside the motor? Exactly the same control chain as Chapter 9 §4 — position loop on top, velocity loop below it, current loop at the bottom. This mode only adds one new piece on top: a **profile generator**. Instead of giving the control loops the final target all at once, it feeds them a target that moves gradually, at the speed and acceleration you chose. The loops then chase this moving target, just as before. That is the entire trick — and it is why everything you learned in Chapters 6–9 still applies here without any change.

In CubeMarsTool, this mode lives in the **Trap Control** panel ("Trap" is short for trapezoid), with a Multi Mode / Single Mode selector and the three input fields. Over CAN, it is Control Mode ID 6, with the packet name `CAN_PACKET_SET_POS_SPD` (§5).

## 2. Multi-Turn: ±36,000°, Full Turns Are Counted

In **Multi Mode**, the position number is measured from the motor's zero (origin) position, and it can go far beyond one revolution: from **−36,000° to +36,000°**. Since one full turn is 360°, this range means **±100 full turns** (manual §3.3.1.1, p. 24: "±100 turns, i.e., −36,000°–36,000°"). This is the same position coordinate that Chapter 9's P field used, tracked by the same multi-turn position counting (Ch 9 §2).

The key idea is simple: **Multi Mode counts complete revolutions.** Think about the targets 90°, 450°, and 810°. If you only look at the shaft, all three leave it pointing in the same direction. But to Multi Mode, they are three *different* places:

- 90° means: 90° from the origin;
- 450° means: one full turn plus 90° (because 450 = 360 + 90);
- 810° means: two full turns plus 90° (because 810 = 720 + 90).

So if you command 720° from the origin, the shaft makes exactly two forward revolutions. Command −720°, and it makes two revolutions in reverse. And just like Chapter 9's targets, these are **absolute** positions, not "move by this much" instructions: if you command 720° twice, the motor moves once, and then does nothing — it is already there.

When is Multi Mode useful? Whenever your machine needs to know *which turn* it is on: lead screws, winches, cable drums — any mechanism where the turns add up into distance.

## 3. Single-Turn: 0–359°, One Revolution Only

In **Single Mode**, the position is just an angle inside one revolution: from **0° to 359°** (manual §3.3.1.2, p. 25: "the position is only one turn, i.e., 0°–359°"). The best picture is a compass dial:

```text
                  0°
                  ↑
                  |
          270° ←--+--→ 90°
                  |
                  ↓
                 180°
```

Targets like 720° or 1,080° have no meaning here — there is no "second lap" on a compass. You simply tell the motor which direction the shaft should point, and it moves there with your chosen speed and acceleration. Single Mode fits dials, turntables, valves, camera heads — anything where only the *facing direction* matters, and the number of laps does not.

## 4. Multi vs Single: The Difference

Both modes move the motor in the same smooth, profiled way. The whole difference is what the position number represents:

| Feature | Multi Mode | Single Mode |
|---|---|---|
| Position range | −36,000° to +36,000° | 0° to 359° |
| Counts full revolutions? | Yes (±100 turns) | No |
| Are 90° and 450° different targets? | Yes — one full turn apart | No — 450° is not a valid target here |
| Speed specified? | Yes (ERPM) | Yes (ERPM) |
| Acceleration specified? | Yes (ERPM/s²) | Yes (ERPM/s²) |
| Typical purpose | Motion that must count turns | Pointing within one revolution |

> **Key point:** Multi Mode's position is like an *odometer* — it keeps adding up turns. Single Mode's position is like a *dial* — only the direction within one turn exists. Everything else — the profile, the control loops, the holding — is identical.

## 5. Units and Protocol Scalings

All three command values use units you already met in Chapter 3. Here we only add the CAN message that carries them. Over CAN, this mode is **Control Mode ID 6**, packet `CAN_PACKET_SET_POS_SPD` (enum, p. 32), with an 8-byte data payload (manual §4.1.7, p. 37):

- **Position** — bytes Data[0..3], an int32, where **1 count = 0.0001°**. So the range −360,000,000 to +360,000,000 counts represents −36,000° to +36,000°. This is the same "multiply degrees by 10,000" rule as Chapter 9 §1.
- **Speed** — bytes Data[4..5], an int16, where **1 count = 10 ERPM**. So −32,768 to 32,767 counts represents −327,680 to +327,680 ERPM.
- **Acceleration** — bytes Data[6..7], an int16, where **1 count = 10 ERPM/s²**. So 0 to 32,767 counts represents 0 to 327,670 ERPM/s² (Ch 3 §4).

The manual's example function takes normal engineering units and applies these scalings for you — degrees are multiplied by 10,000, and speed and acceleration are divided by 10 (because each count is worth 10 units):

```c
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA) {
    int32_t send_index = 0;
    int16_t send_index1 = 4;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    buffer_append_int16(buffer, spd / 10.0, &send_index1);
    buffer_append_int16(buffer, RPA / 10.0, &send_index1);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}
```

Two things to keep in mind when reading these numbers. First, the ±327,680 ERPM speed range is only what the *message format* can express — it is much bigger than the GUI's ±50,000 ERPM limit, the Velocity-Loop CAN limit of ±100,000 ERPM, and far bigger than the motor's real top speed of about 570 rpm. A message range is never a promise of motor performance (Ch 2 §9, Ch 3 §6). Second, everything you type is *electrical* speed: **divide by 189 for shaft rpm**, and read acceleration as "ERPM gained per second" — for example, 5,000 ERPM/s² adds about 26 shaft rpm every second (Ch 3 §§2, 4). *(Note: earlier drafts of this series wrote the acceleration unit as "ERPM/s". The correct unit is ERPM/s², matching the manual's definition of 10 ERPM/s² per count.)*

## 6. Current, Load, and Holding: The Motor Handles It

Notice what you are *not* commanding here: current or torque. You only give $P_d$, $V_d$, $A_d$. But the motor can only move by producing torque, and torque always needs current (Ch 2 §6). So who takes care of the current? The controller does, automatically. Inside, the profile generator's moving target drives the position and velocity loops, and those loops decide the current — exactly as in Ch 9 §4. The manual's Simplified Diagram for the Position-Speed Loop (§4.1.7, p. 37) shows this same stack with the profile generator in front. The current needed changes from moment to moment: almost nothing while cruising with no load, a bit more during the speed-up and slow-down ramps (accelerating mass costs torque), and more again whenever a load pushes back.

**What about working under load?** Both modes stay fully closed-loop. Attach a mechanism, and the controller simply raises the current to whatever the commanded motion needs — the same "the load decides the current" behavior you saw in Ch 8 §4. But the budget is limited: torque is capped by the current limits and the actuator's ratings (Ch 1 §2). If the load needs more than the motor can give, the motor falls behind the profile — it pushes with everything it has, but cannot keep up with the moving target.

**What about after arrival?** The mode holds the position, exactly as Ch 9 §5 explained: the loop keeps running, any push creates an error, and the error creates a correcting torque. The hold remembers its target, and it is active electronics, not a mechanical lock — cut the power and the hold is gone (Ch 7 §6). And here is our two-line reminder of the big lesson from Ch 7 §5: **a shaft that is holding still can still carry a lot of current, and current always makes heat — zero speed ≠ zero current ≠ zero heat.** Ch 7 §5 has the full physics; watch the temperature telemetry during any loaded hold and you will see it — and this mode behaves exactly the same way once the trapezoid ends.

## 7. A Small Trap in the Manual (for Serial-Port Programmers)

If you control this mode through the **serial port** instead of CAN, be careful with one part of the manual. In the serial command list (§4.3.2, p. 43), the comments on two commands appear to be **swapped**: `COMM_SET_POS_MULTI = 61` is labelled "single circle motion mode", while `COMM_SET_POS_SINGLE = 62` is labelled "multiple circles motion mode, range ±100". The names say one thing; the comments say the opposite. Our advice: trust the command *names* and the "±100 turns" range description, not the comment placement — and before you build firmware on it, test each command ID on the bench to confirm what it really does.

## 8. Short Comparisons with the Neighbor Modes

**vs. Position Loop Mode (Ch 9).** Same absolute targets, same hold — but opposite travel style: Ch 9 always moves at full effort; here, the trip's speed and acceleration are part of your command. Demo 1 below repeats Ch 9's demo target — the same 90° step — as a smooth glide, exactly for this comparison. The full head-to-head is in Ch 12.

**vs. Velocity Loop Mode (Ch 8).** Ch 8 cruises at a target speed forever; this mode borrows that cruise as the middle of a trip that ends — the flat top of the trapezoid *is* a short velocity-mode moment with a destination attached. See Ch 12 for the full comparison.

**vs. MIT Position Control (Ch 11).** Both fix Ch 9's "always full speed" problem, but in different ways: this mode lets you shape the *path* (explicit speed and acceleration, fixed internal gains); MIT lets you shape the *response* (per-command stiffness Kp and damping Kd, no explicit profile). See Ch 11 §7 and Ch 12.

## Demos

Both demos follow Ch 4 §6's ground rules: motor mounted on the bench and calibrated per Ch 4, output shaft **unloaded**, 48 V supply with a conservative current limit, CubeMarsTool connected with Real-time Data open and the Stop button located. You will enter commands in the **Trap Control** panel — choose Multi Mode or Single Mode as stated, then fill Position (°), Speed (ERPM), and Acceleration (ERPM/s²). The manual's preconditions apply (§3.3.1.1–3.3.1.2): input power stable, connectors properly connected, upper computer successfully connected. Stick a small tape flag on the shaft so the rotation is easy to see, check the current position before each command (targets are absolute — Ch 9 §1's habit), and finish every session by returning to 0° and pressing Stop, never leaving a live hold alone. The speeds below are deliberately gentle. If you change any value, convert it with Ch 3 §2's ÷189 rule *before* you press Enter.

### Demo 1 — Multi-Turn with a Gentle Profile

The star of the chapter: first the very same 90° target as Ch 9's demo — this time as a designed glide — and then two full revolutions of smooth, profiled travel.

**Starting condition:** calibrated per Ch 4, unloaded, Multi Mode selected, shaft at 0° (zero the origin per Ch 9 §2 so the numbers below read literally), Real-time Data plotting position and speed.

**Commands and expected observations:**

1. Command **Position 90°, Speed 5,000 ERPM, Acceleration 30,000 ERPM/s²** — the exact command shown in Figure 10.1 and Video 10.1. The shaft makes a brisk but smooth quarter turn, easing in and easing out, and the whole move is over in well under a second. Now play Video 9.1 next to Video 10.1: the same 90° target, a sprint versus a glide.
2. Command **Position 720°, Speed 5,000 ERPM, Acceleration 5,000 ERPM/s²**. The shaft eases into motion over about one second, cruises through most of two full revolutions at about 26 rpm (that is $5{,}000 \div 189$ — Ch 3 §2), and eases to a stop exactly on the 720° mark. The whole trip takes about 5–6 seconds, with no snap at either end.
3. Command **Position 0°, Speed 5,000, Acceleration 5,000**. The same glide, in reverse, back to the origin.
4. Now do this once, on purpose: **Position 720°, Speed 1,000 ERPM, Acceleration 1,000 ERPM/s²**. The shaft crawls at about 5.3 rpm — one revolution every ~11 seconds, about 23 seconds for the trip. Nothing is broken. 1,000 ERPM really *is* this slow — that is the point of this step: it burns the ÷189 rule into your memory forever. When your patience runs out, return to 0° at 5,000 ERPM.

| Quantity | Do you command it? | What you observe |
|---|:---:|---|
| Position | **yes** — 90°, 720°, 0°, 720° | lands on each target and holds |
| Speed & acceleration | **yes** — in every command | ramps and cruise speeds that you chose — brisk in step 1, leisurely in step 4 |
| Current | no | small bursts during the ramps; near zero while cruising or holding unloaded |

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image11.png" 
          alt="Servo Trap Multi-Mode Control : Position = 90, Speed = 5000, Acceleration = 30000">
      <figcaption>
        Figure 10.1: Servo Trap Multi-Mode Control : Position = 90, Speed = 5000, Acceleration = 30000. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video9.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 10.1: Servo Trap Multi-Mode Control : Position = 90, Speed = 5000, Acceleration = 30000</i>
      </p>
</figure>
</div>

### Demo 2 — Single-Turn Positioning

The compass dial of §3, visited quadrant by quadrant.

**Starting condition:** as Demo 1, but with **Single Mode** selected; shaft settled, Real-time Data showing position.

**Commands and expected observations:** keep **Speed 5,000 ERPM and Acceleration 30,000 ERPM/s²** the same throughout — the same values as in Figure 10.2 and Video 10.2 — and command **Position 90° → 180° → 270° → 0°**, one after another. Each command produces a short, smooth move (well under 2 seconds) landing on the next quadrant; the shaft visits the four compass points and holds each one. Pay attention to which rotation direction the controller chooses on each leg — especially the last step, 270° → 0°. And notice that at no point can you ask for a "second lap": in Single Mode, position means direction, and nothing more.


<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image12.png" 
          alt="Servo Trap Single-Mode Control : Position = 90, Speed = 5000, Acceleration = 30000">
      <figcaption>
        Figure 10.2: Servo Trap Single-Mode Control : Position = 90, Speed = 5000, Acceleration = 30000. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video10.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 10.2: Servo Trap Single-Mode Control : Position = 90, Speed = 5000, Acceleration = 30000</i>
      </p>
</figure>
</div>


## When to Use Position–Velocity Mode — and When Not

Use this mode whenever you need to reach **an angle along a journey you must control**: indexing tables, dispensers, door and lid mechanisms, lead screws and winches (Multi Mode), dials and turntables (Single Mode) — in short, anything attached to the shaft that would not enjoy being sprinted at full power. In practice, that is almost everything attached. The moment a real mechanism is on the shaft, this should be your default position mode; plain Position Mode's unshaped, full-effort trips (Ch 9 §1) are best kept for unloaded bench work and for mechanisms that can genuinely survive full effort.

Skip it in a few cases. If you do not need a profile and maximum speed is acceptable, Ch 9 is one field to fill instead of three. If the motion never ends — a conveyor, a spinning wheel — you want a speed target, not a position target: that is Velocity Mode (Ch 8). And if you want to choose the *stiffness and damping* of the motion and the hold, rather than its speed, or to stream trajectories at a high rate, that is MIT control (Ch 11). The full decision framework is Ch 12.

## Where These Ideas Go Next

| Established here | Continues in |
|---|---|
| The trapezoidal profile as a rate-limited moving target on top of the Ch 9 cascade (§1) | Ch 11 (MIT replaces the profile with per-command gains), Ch 12's mode hierarchy |
| Multi-turn vs single-turn position meanings (§§2–4) | Ch 12's master table and scenario (e): indexing with profiled moves |
| The Position–Velocity CAN frame and its three scalings (§5) | Ch 3 §6's master table (already recorded there); Ch 12 cross-cutting cautions |
| The profile is not tracked when the torque budget runs out (§6) | Ch 12 limitations column |
| The serial enum label swap (§7) | — (verify on the bench before using in firmware) |

## Key Takeaways

- Position–Velocity Loop Mode (Servo ID 6, `CAN_PACKET_SET_POS_SPD`) takes **position + speed + acceleration** in every command, and the controller performs a **trapezoidal profile**: ramp up, cruise, ramp down — Ch 9's destination with the journey handed to you (§1). Short trips become triangles that never reach the commanded speed, and that is normal.
- **Multi Mode**: an odometer-style position, ±36,000° = ±100 turns; 90°, 450°, and 810° are three different places. **Single Mode**: a dial-style position, 0–359°; laps do not exist (§§2–4).
- Message scalings: position int32 × 10,000 (0.0001° per count), speed int16 at 10 ERPM per count (±327,680 ERPM range), acceleration int16 at 10 ERPM/s² per count (0–327,670 ERPM/s²) — these are message ranges, not motor ratings (§5, Ch 3 §§2/4/6).
- Everything you type is electrical speed: **divide by 189 for shaft rpm**, and plan speeds before commanding — 1,000 ERPM is a ~5.3 rpm crawl; use about 5,000–20,000 ERPM to see clear motion (Ch 3 §2, Demo 1).
- Current and torque are handled entirely by the controller: the load, the ramps, and the hold decide the current bill, never your three numbers (§6; Ch 6 §4, Ch 8 §4, Ch 7 §5, Ch 9 §5).
- Serial-port users: the manual's comments on command IDs 61 and 62 appear swapped — trust the command names and test on the bench (§7).

## Safety Notes

- A smooth profile makes moves gentler, **not automatically safe**: a high speed with a high acceleration gets close to Ch 9's sprint. For your first commands: unloaded shaft, modest speeds (around 5,000 ERPM), targets near the current position, and hands and cables clear *before* pressing Enter.
- **Targets are absolute** — the length of the trip is the distance to the target, not the size of the number. Check the current position first, especially in Multi Mode, where a "small" target can be several laps away (Ch 9 §1's habit).
- **Convert before commanding** (Ch 3 §2): a value that crawls when read as ERPM is fast when read as rpm, and "fixing" a slow demo by pushing the number toward the ±327,680 ERPM message limit is exactly how shafts suddenly jump to hundreds of rpm. Message ranges are never ratings (Ch 2 §9).
- **Arrival starts a live hold**, with all of Ch 9's consequences: do not leave it unattended, end sessions at 0° plus Stop, and remember: power off = hold off (Ch 7 §6).
- **Holding a load, and heavy profiles, heat the motor** (Ch 7 §5): keep the temperature field visible during long sessions, and stay inside the 12 A continuous / 28 A peak ratings (Ch 1 §2), no matter what the message format can encode.
- A load that is too strong does not pause the mode — the controller falls behind the profile while pushing at full available effort, which is a maximum-current heating condition (§6). If the mechanism jams: press Stop first, investigate second.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §3.3.1.1 "Multi-Position-Velocity Loop Mode" (p. 24: Multi Mode GUI operation; position ±100 turns = −36,000°–36,000°; "the motor will move at the desired speed until it reaches the desired position"); §3.3.1.2 "Single-Position-Velocity-Loop Mode" (p. 25: Single Mode GUI operation; position one turn, 0°–359°); §4.1 "Servo Mode Control Modes and Description" (p. 31: "Position-Velocity Loop Mode: A specified position, speed, and acceleration are given to the motor"; Control Mode ID 6 among the seven mode IDs; extended-frame ID layout); the CAN packet enum (p. 32: `CAN_PACKET_SET_POS_SPD`); §4.1.7 "Position-Velocity Loop Mode" (p. 37: Simplified Diagram for the Position-Speed Loop; data-transmission definition — position int32 ±360,000,000 ↔ ±36,000°, speed int16 −32,768…32,767 ↔ −327,680…327,680 ERPM, acceleration int16 0…32,767 ↔ 0…327,670 with 1 unit = 10 ERPM/s²; the `comm_can_set_pos_spd` example routine); §4.3.2 "Serial Port Message Protocol" (p. 43: `COMM_SET_POS_MULTI = 61` / `COMM_SET_POS_SINGLE = 62` and their apparently swapped comments, cited in §7).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — 21 pole pairs, 9:1 reduction, and the 12 A / 28 A current and 9 N·m / 22 N·m torque ratings, cited via this series' reference table (Ch 1 §2) and the ÷189 conversion (Ch 3 §2).
3. **CubeMarsTool upper-computer software** — Trap Control panel shown in Figures 10.1–10.2; © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.

---

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 11 — MIT (Force) Control Mode →](11_MIT_Force_Control_Mode.md)
