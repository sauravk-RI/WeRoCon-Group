# Chapter 9 — Position Loop Mode

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 10 — Position–Velocity Loop Mode →](10_Position_Velocity_Mode.md)

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In Chapter 8, the motor learned to control its own **speed**. In this chapter, it learns to control the last thing in the chain: **where the shaft is pointing**.

**Position Loop Mode** — Servo Control Mode ID 4 — is the mode that most real machines actually want. Think about a robot arm. Nobody asks the arm to "run 3 amps of current" or "spin at 5,000 ERPM". We ask it to *"move to 45° and stay there."* That is exactly what this mode does. You type one number — a target angle in degrees — and the controller works out everything else on its own: how to move, how much torque to use, and how much current to send. And after the shaft arrives, the controller keeps working, holding the shaft at that angle against anything that tries to push it away.

Before we go any further, there is one fact about this mode that is more important than everything else, so let's say it right away: **in plain Position Mode, the motor always moves at its maximum speed and maximum acceleration.** There is no "gentle" setting in this mode. Every trip is a full-power sprint. The warning box in §1 and the careful design of the demo below come from this one sentence.

Here is what this chapter covers, in order:

- what Position Loop Mode is, its command range, its CAN message, and the maximum-speed warning (§1);
- how the motor knows where its shaft is — the encoder, multi-turn counting, and where 0° comes from (§2);
- the position error, and the feedback loop built on it (§3);
- the cascade: how one angle command turns into velocity, torque, and current — plus what the manual's own diagram tells us about how it is really built inside (§4);
- what happens after the shaft arrives: holding (§5);
- where this mode is useful, and where it is not (§6), with short pointers to the neighbouring modes (§7);

followed by a bench demo, advice on when (and when not) to use the mode, safety notes, and sources. Everything here assumes your motor is connected and calibrated, and that you follow the safety ground rules from Ch 4 §6.

## 1. What Position Loop Mode Is

The manual describes this mode in two places, and both quietly say the same alarming thing. The Servo-mode overview (§4.1, p. 31) says: *"Position Mode: A specified position is given to the motor, and the motor will move to the specified position (speed and acceleration are default to the maximum value)."* The GUI instructions (§3.3.1.4, p. 26) say: in the Servo Control interface, *"enter the desired position P, and the motor will reach the desired position with maximum speed and acceleration."*

Read those brackets again. "Maximum speed and acceleration" is not a small detail. It changes how you must handle this mode, so here it is in a box:

> ### ⚠️ MAXIMUM-SPEED WARNING
>
> **In Position Loop Mode, the motor travels to every target at maximum speed and maximum acceleration.** This is not a setting you can turn down inside this mode — it is simply how the mode works (manual §3.3.1.4, p. 26; §4.1, p. 31). If you command a target that is 3,600° away, the motor will sprint ten full turns at full power, and then brake just as hard at the end. Please remember these four things before your first command:
>
> - **Never send your first command with anything attached to the shaft** — no lever, no mechanism, no load. A full-power ten-turn sprint can break fixtures, throw attached parts across the room, and injure hands. Do all your learning with a bare, unloaded shaft.
> - **Start with targets that are close by.** The demos below begin with 90° steps on purpose: the full-speed part of the trip is over almost before it starts.
> - **Check where the shaft is before you command.** The length of the trip is the distance to your *target*, not the size of the number you type. If the shaft is at 3,000° and you type 0°, that "small" command is a 3,000° sprint.
> - **If you want to choose the speed yourself, this is the wrong mode.** That is what Position–Velocity Mode is for (Ch 10) — same destination, but you shape the journey. This chapter's demo video exists exactly so you can compare it with Ch 10's smooth version later.

Now the details of the command itself. The command is a **target angle of the output shaft, in degrees**. It can be anywhere in the multi-turn range **±36,000°, which is ±100 full revolutions** (Ch 3 §1). One very important point: the command is an **absolute** target, not a "move by this much" instruction. It names a *place*, not a *distance*. If you command 90° twice in a row, the motor moves once and then does nothing the second time — because the second command asks for a place the shaft is already at.

On the CAN bus, the command travels as **Control Mode ID 4** with the packet name `CAN_PACKET_SET_POS` (manual p. 32). The angle is sent as an int32 number where **1 count = 0.0001°**. So the range −360,000,000 … +360,000,000 counts encodes −36,000° … +36,000° (manual §4.1.5, p. 36). The manual's example function simply multiplies your degrees by 10,000 before sending:

```c
void comm_can_set_pos(uint8_t controller_id, float pos) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}
```

In CubeMarsTool, the command is simply the **P field** in the Servo Control interface, typed in plain degrees.

Because the target is absolute, the length (and violence) of the trip depends on where the shaft *currently is* — not on how big your number looks. A small table makes this clear:

| Shaft is currently at | You command P = | Length of the trip | At maximum power, that means… |
|---:|---:|---:|---|
| 0° | 90° | 90° | a quick snap — over in a blink |
| 0° | 720° | 2 full turns | a real full-speed sprint |
| 3,000° | 0° | about 8.3 turns | a *long* sprint hiding behind a small number |
| 90° | 90° | 0° | nothing — the shaft is already there |

The last row teaches a habit worth forming early: sending the same target again does not mean "do it again". It means "stay where you are". There are no relative moves in this mode. If you want one, read the current position from the telemetry, add your step to it, and command the sum.

One last small caution, carried over from Ch 3 §6: the motor's automatic telemetry message reports position as an int16 number that only covers ±3,200° — much narrower than the ±36,000° you can command. So for long multi-turn positions, trust the full position field in the GUI, not the periodic telemetry frame alone.

## 2. How the Motor Knows Where It Is

Position control only works because the motor can constantly measure its own shaft angle. The AK80-9 has a built-in **encoder** — a sensor that reports the shaft's angle to the controller, thousands of times per second. (It is the same sensor whose readings, when differentiated, gave Ch 8 its speed measurement.) Two practical things about it matter for this chapter.

**First: the position count is multi-turn.** The firmware does not only know the angle *within* one revolution. It also *counts revolutions*. So the measured position runs smoothly from 0° up through 360°, 720°, 3,600° and beyond (and the same in the negative direction), across the whole ±36,000° range. This accumulated number is the position your P command points at. It is also why you must actually check "where is the shaft right now?" — in the Real-time Data position field — before commanding, as the warning box says.

**Second: zero degrees is wherever you tell it to be.** The multi-turn count is measured from an origin (a reference point), and the protocol lets you set it. Set Origin Mode (Control Mode ID 5, `CAN_PACKET_SET_ORIGIN_HERE`, manual §4.1.6, p. 36) declares that the shaft's *current* position is now 0° — either temporarily (forgotten when power is removed) or permanently (saved). In practice: turn the shaft to where you want your reference, set the origin, and from then on every P command is measured from that spot. The demos below assume you have done exactly this at their start.

That is all the sensing knowledge this chapter needs. The encoder's role in calibration was covered in Ch 4 §5, and its role in speed measurement in Ch 8 §1.

## 3. Position Error and the Feedback Loop

Just like Ch 8's speed loop, this mode's entire "thinking" comes down to one subtraction. Call the commanded target $P_d$ (d for *desired*) and the measured multi-turn position $P$. The **position error** is:

$$
e_P = P_d - P .
$$

In plain words: *how far, and in which direction, is the shaft from where I want it?* The sign and the size of this one number decide everything:

| Position error | What it means | What the controller does |
|---|---|---|
| $e_P > 0$ | shaft has not reached the target yet | drive forward — harder when the error is big |
| $e_P = 0$ | shaft is exactly on target | hold — apply only the torque needed to stay |
| $e_P < 0$ | shaft has gone past the target | drive backward, toward the target |

Let's walk through one real example — the demo's command. The shaft rests at $P = 0°$ and you command $P_d = 90°$. Here are four snapshots of the loop's arithmetic during the move. Notice that the *size* of the error matters as much as its sign, because the controller scales its effort to it (the same idea as Ch 8 §3):

| $P_d$ | measured $P$ | $e_P$ | How the controller reads it |
|---:|---:|---:|---|
| 90° | 0° | +90° | far from target — full-power drive forward (this is the warning box in action) |
| 90° | 62° | +28° | getting closer — effort eases off, braking begins |
| 90° | 88° | +2° | nearly there — small correcting push, gentle landing |
| 90° | 93° | −3° | went past — drive reverses, back toward the target |

So the shaft accelerates, the encoder keeps reporting fresh positions, the error shrinks, and the controller's push shrinks with it — braking the spinning rotor so the shaft lands *on* 90°, not through it. And if it does overshoot a little, the error simply becomes negative and the correction reverses. That is really all "stopping at the answer" means in a feedback loop. Also notice: the loop never finishes. "Arrival" only means the error is currently zero (§5 continues this story).

Here is the whole loop as a picture:

```text
P_d ──►(compare)──► e_P ──► position controller ──► torque request ──► iq_ref
          ▲                                                              │
          │                                                    FOC current loop (Ch 2 §5, Ch 6)
          │                                                              │
          └────── measured P ◄────── encoder ◄──────────── motor ◄───────┘
```

Two ideas transfer directly from Ch 8 §2, and each needs only one line here. First, the command is a **standing target**, not a one-time action — if you send a new target while the motor is still moving toward the old one, the loop simply starts chasing the new number. Second, the telemetry position field is the *truth* and the P field is the *wish* — during a move they disagree, and that disagreement (the error) is exactly what the whole mode runs on.

## 4. The Cascade: From an Angle to a Current

This section is the reward for reading Chapters 6 and 8 first. Here is the puzzle: a motor cannot "produce position". It cannot even produce speed. As Ch 6 §4 established once for this whole series, a motor produces only one thing — **torque**. Torque creates acceleration, acceleration changes speed, and speed, acting over time, changes position. So a position controller can only do its job by working *down* that chain: decide what motion would shrink the error, request the torque that creates that motion, and turn that torque into winding current.

```text
what you command:      position                                (this chapter)
what that requires:    a velocity toward the target            (Ch 8's quantity)
what THAT requires:    a torque to create/adjust the velocity  (Ch 6's quantity)
what THAT requires:    an Iq current                           (Ch 2 §6's constants)
what physics then does: current → torque → acceleration → velocity → position
```

Read that ladder from the bottom up and you can see the whole series in one picture. Duty Cycle Mode commanded voltage (Ch 5). Current Loop Mode put *your* hand on the current command (Ch 6). Velocity Mode replaced your hand with an automatic regulator (Ch 8). And Position Mode now places a supervisor above that supervisor. Each level automates the one below it. Nothing from Chapters 2–8 is thrown away — every behaviour you tested at the bench in the earlier chapters is still running, in miniature, inside every position move.

There is a neat way to line up the whole Servo family: just ask one question — *who writes the current request (`iq_ref`) that the FOC loop serves?*

| Mode | You command | `iq_ref` is written by | What you get |
|---|---|---|---|
| Duty (Ch 5) | a voltage effort | nobody — there is no current target | whatever physics makes of the voltage |
| Current Loop (Ch 6) | $I_q$ itself | **you**, by hand | a torque; speed and position just happen |
| Velocity (Ch 8) | a speed | an automatic speed regulator | a defended speed; position just happens |
| **Position (this chapter)** | **an angle** | **the position stage (see below)** | **a reached and defended angle** |

Going down each row, your job shrinks by one step — and the controller's job grows by the same step.

Two practical consequences are worth about five lines each before we look at the real diagram:

- **The required torque changes constantly during a move.** Far from the target: a large accelerating torque. Getting close: a shrinking torque, and then a *reversing* torque, to brake the spinning rotor so it lands on the number instead of flying past it. You never see this negotiation — you typed one angle — but it is all there in the current telemetry during the demo.
- **So the required current changes constantly too.** Torque needs current ($\tau \rightarrow I_q$ through Ch 2 §6, as always). Electrically, a position move is a quick rise-and-fall of $I_q$. After arrival, the current drops to whatever the *hold* needs (§5) — almost nothing when unloaded, quite a lot under load.

Now for one honest and genuinely interesting detail, because the manual shows it and it deserves to be seen. The **Simplified Control Diagram for Position Loop** (manual §4.1.5, p. 35) shows how the mode is actually built. Translated into English, it looks like this:

```text
set angle ──►(−)──►[ position-loop Kp ]──►(+)──► [ torque-limit ] ──► iq_ref ──► FOC current loop ──► motor
              ▲                            ▲       protection                     (id_ref = 0)          │
              │      ┌►[ d/dt ]►[ Kd ]─────┘                                                            │
              └──────┴───────────── measured position ◄──── encoder ◄───────────────────────────────────┘
```

Compare this with Ch 8 §1's velocity diagram and notice something surprising: the position loop is **not** drawn as a loop wrapped around Ch 8's speed regulator as an inner box. Instead, the position stage is drawn as something called a **PD regulator** that writes `iq_ref` directly. "PD" means it has two parts: a **P**roportional part (the gain **Kp**), which pushes harder the bigger the position error is, and a **D**erivative part (the gain **Kd**), which acts on how fast the position is changing — in other words, on the measured velocity — and works like a damper, calming the motion down. The two parts are added together, limited by a torque-protection block, and sent straight to the same FOC current loop that every other mode uses.

So is the "cascade" idea from the ladder above wrong? No — it is completely right as a way of *thinking*: velocity information really does control the move. But in this firmware, the velocity enters as the **Kd damping term**, not as a separate nested speed loop. And if this Kp-plus-Kd shape looks familiar, it should: *torque = Kp × (position error) + damping* is exactly the shape of MIT mode's famous equation — except that here, the gains are fixed inside the firmware, while MIT mode lets you send your own Kp and Kd with every command. In other words, Servo Position Mode is like **a stiff MIT position hold where someone else already chose the gains for you**. Chapter 11 will cash in this preview fully.

## 5. Arrival and Holding

The move ends. The mode does not. After the error reaches zero, the loop keeps running at full rate, and everything special about "holding" comes from simply re-reading §3's table at $e_P \approx 0$. Any disturbance that moves the shaft creates a fresh error, and a fresh error creates a correcting torque — automatically, immediately, for as long as the mode is active. Push the shaft by hand from 90° to 89° and you have just handed the controller a +1° error; it drives the shaft back. That "spring-back" — which you can feel for yourself in the demo's step 4 — is the behaviour this section owns.

Everything *else* about holding was explained once, properly, in Ch 7 §5, so here it gets only its allowed two lines: **a holding shaft at zero speed still carries whatever current its holding torque needs, and it pays the $I^2R$ heating bill the whole time — zero speed ≠ zero current ≠ zero heat.** The full physics, the "heat grows with current squared" table, and the safety habits are all in Ch 7 §5; you can verify the same thing in this mode's telemetry during any loaded hold.

The position-mode version of the story has exactly two cases:

| Holding at the target… | External torque on the shaft | Holding torque needed | Current and heat |
|---|---|---|---|
| unloaded (the demo, step 3) | almost none | almost none | almost zero — the hold is nearly free |
| under load (e.g. gravity pulling a lever) | continuous (e.g. gravity pulling a lever) | continuous, matching the load | steady $I_q = \tau / 0.5701$, steady $I^2R$ heat |

Same zero speed in both rows. The difference is the *load* — never the motion.

Three position-specific points complete the picture. First, the hold is **not a mechanical lock** — it is active electronic control, and it disappears the instant power does (the general story is Ch 7 §6). Second, the hold has a **memory of its target**, which a Brake-Mode hold does not have: force the shaft away and let go, and Position Mode drives it back to $P_d$, while Brake Mode happily keeps the new angle (Ch 7 §6 stages exactly this contrast). Third, the holding strength is **finite**: if an external torque is bigger than what the actuator can produce within its torque, current, and heat limits (Ch 1 §2), the shaft *will* be pushed away. The controller fights at full available power the whole way down — which also means that a load which keeps overpowering the hold is a maximum-current heating situation, not a harmless stall.

## 6. Applications and Limitations

**Where this mode fits.** Any task whose real specification is an angle: robot-arm and humanoid joints, pan-tilt camera mounts, grippers, automated valves, focus and aperture drives, indexing fixtures, lab positioning stages, pick-and-place axes. The interface is the simplest in the Servo family — one absolute number — and in return you get the full closed-loop package: automatic torque under load, disturbance rejection, and an endless hold at the destination.

**Where it does not fit.** Four boundaries, all met above. The motion profile is not yours: every trip is a maximum-power sprint (§1), so if the *journey* matters — smooth starts, controlled speeds, synchronised axes — you want Position–Velocity Mode (Ch 10) or MIT control (Ch 11). The hold is not a lock: a power cut drops the load (§5, Ch 7 §6). The hold is not free: holding under load costs current and heat continuously (Ch 7 §5). And the holding strength is finite: choose your actuator for the *worst-case* external torque, not the average (§5). The full "which mode should I choose?" framework — including "position hold vs brake hold, when to use each" — is Chapter 12.

## 7. Nearest Neighbours: Four Pointers

**vs. Velocity Loop Mode (Ch 8).** Velocity Mode answers "how fast?" and never stops on its own; Position Mode answers "where?" and stopping at the answer *is* the job — with Ch 8's error-to-effort machinery working one level down inside it (§4). Full comparison in Ch 12.

**vs. Current Brake Mode (Ch 7).** Both can keep a shaft still, but Brake Mode only asks *"is it moving?"* and holds wherever it happens to stand, while Position Mode asks *"is it where it should be?"* and drives any displacement back to the target (§5). Which hold to use when is Ch 12, scenario (d).

**vs. Position–Velocity Mode (Ch 10, next).** Same idea of a destination, opposite philosophy about the trip: this mode always sprints at maximum power (§1); Ch 10 lets you command the speed and acceleration and shapes a smooth trapezoid profile. The demo's Video 9.1 exists exactly to be replayed against Ch 10's Video 10.1 — the same 90° target, sprint versus glide. Full contrast in Ch 12.

**vs. MIT Position Control (Ch 11).** The manual's own diagram shows this mode as a fixed-gain PD position regulator (§4); MIT position control is the same law with Kp and Kd sent in every command — stiffness and damping become *yours* to choose, move by move. The full Servo-vs-MIT picture is Ch 11 and Ch 12.

## Demo — A 90° Step at Full Speed

The demo follows Ch 4 §6's ground rules: motor bolted to the bench and calibrated per Ch 4, output shaft **unloaded**, 48 V supply with a conservative current limit, CubeMarsTool connected with the Real-time Data panel open and the Stop button located before anything else. The manual's preconditions for the mode apply throughout (§3.3.1.4): stable input power, connectors properly connected, upper computer successfully connected. Position commands go into the Servo Control interface's **P field**, in degrees, in the multi-turn coordinate from §2 — so before your first command, note the current position in Real-time Data (best of all: park the shaft and set the origin as in §2, so the numbers below read literally). Fit a tape flag or pointer to the shaft so angles are visible, both to your eyes and on camera.

Two mode-specific cautions, both direct consequences of §1's warning box. First: **every move in this demo is a maximum-speed, maximum-acceleration move** — even a 90° step is a snap — so make sure hands, cables, and loose objects are clear of the shaft *before* pressing Enter, not after. Second: because the mode holds forever after arriving, **end the demo by returning the shaft to P = 0° and pressing Stop** (or leaving the mode) rather than walking away from a live hold.

**Starting condition:** calibrated per Ch 4, unloaded, shaft flagged and parked at 0° with the origin set as in §2, Real-time Data plotting position and current.

**Commands and expected observations:**

1. Command **P = 90°**. The shaft snaps a quarter turn almost instantly — §1's maximum-speed move, over in a blink. Figure 9.1 and Video 9.1 show exactly this command and its result. In the telemetry, the position steps to 90° and the current shows a brief spike-and-reverse as the controller launches the rotor and then brakes it onto the target (§4).
2. Command **P = 90° again**. Nothing happens. The target is absolute — the shaft is already at the one place you named (§1). There are no relative moves in this mode.
3. Look at the current field during the hold: with nothing attached, it reads near zero. An unloaded hold is nearly free (§5).
4. Gently nudge the flag a few degrees off the target and let go: the shaft springs straight back to 90°. You have just handed the loop a fresh error, and it corrected it (§3, §5). Nudge only by the flag, gently and briefly — the controller answers with its full available torque.
5. Command **P = 0°**, let the shaft snap home, and press Stop to end the session — never walk away from a live hold.

| Quantity | Commanded by you? | What you observe |
|---|:---:|---|
| Position | **yes** — 90°, then 0° | an instant full-speed snap onto each absolute target |
| Speed & acceleration | no — always maximum | §1's warning box, live |
| Current | no | a brief spike per move; near zero during the unloaded hold |
| The hold | automatic | pushes back and re-centres when nudged (step 4) |

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image10.png" 
          alt="Servo Control Panel for Position Command of 90 Degrees">
      <figcaption>
        Figure 9.1: Servo Control Panel for Position Command of 90 Degrees <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video8.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 9.1: Servo Action for Position Command of 90 Degrees</i>
      </p>
</figure>
</div>

## When to Use Position Loop Mode — and When Not

Use it whenever the requirement is **an angle, reached and kept, with the simplest possible interface**: joints, pointing mechanisms, valves, grippers, indexers — one absolute number in, closed-loop arrival plus an endless disturbance-rejecting hold out. It is the natural first tool whenever "where" is the whole specification and the journey does not matter.

Skip it when its boundaries bite. If the *motion itself* matters — speed limits, smooth ramps, anything attached that would not enjoy being sprinted — the maximum-power profile (§1) rules it out: Ch 10 owns the shaped trip. If the hold must survive a power cut, no electronic hold can (§5): specify a mechanical brake. If long loaded holds are most of the job, budget for the heating cost (Ch 7 §5) or redesign the mechanism. And if you need to choose the stiffness and damping of arrival and holding yourself, per command, that is MIT position control (Ch 11). The systematic decision framework is Ch 12.

## Where These Ideas Go Next

| Established here | Continues in |
|---|---|
| The completed cascade: position above velocity above current (§4) | Ch 10 (adds the motion profile on top), Ch 12's mode hierarchy |
| The maximum-speed default and its safety consequences (§1) | Ch 10 §1's payoff (the profiled alternative), Ch 12's cross-cutting cautions |
| The fixed-gain PD structure of the position stage (§4) | Ch 11 (MIT exposes the same Kp/Kd as per-command parameters), Ch 12 Servo-vs-MIT |
| A hold with target memory vs Brake's hold-in-place (§5) | Ch 12 scenario (d): park-and-hold, which mode when |
| Absolute multi-turn targets, ×10,000 int32 scaling (§1–§2) | Ch 10 §5 (the Position–Velocity frame reuses the scaling), Ch 3 §6's master table |

## Key Takeaways

- Position Loop Mode (Servo ID 4, `CAN_PACKET_SET_POS`, int32 = degrees × 10,000) commands an **absolute target angle** in the ±36,000° multi-turn coordinate — and travels to it at **maximum speed and acceleration, always** (§1). For shaped motion, see Ch 10; for your own gains, Ch 11.
- The loop runs on one subtraction, $e_P = P_d - P$: the sign picks the direction, the size scales the effort, and the shrinking error brakes the landing onto the target (§3).
- A position command becomes motion only through the full chain — error → torque request → $I_q$ → torque → acceleration → velocity → position — with the required torque and current re-decided continuously during every move (§4).
- The manual's own diagram shows the inside as a **fixed-gain PD regulator writing `iq_ref` directly** (Kp on the position error, Kd on the measured velocity): a cascade in concept, MIT-with-built-in-gains in structure (§4).
- Arrival ends nothing: the hold is the same loop at $e_P \approx 0$, turning any disturbance into a correcting torque, with a memory of the target (§5). Zero speed ≠ zero current ≠ zero heat — Ch 7 §5 owns the physics; watch the telemetry during any loaded hold to see this mode paying the bill.
- The hold is active and finite: power off = hold off, and a load beyond the actuator's torque wins the fight — at full current (§5).

## Safety Notes

- **Every move is a maximum-speed, maximum-acceleration move** (§1). For your first commands: unloaded shaft, nearby targets, hands and cables clear *before* commanding, and the current position checked so the trip length is what you think it is.
- **Never attach or adjust a load during a hold or between rapid commands.** Fix loads only with the motor at rest, and move loaded mechanisms in small steps, because the trip cannot be softened.
- **A live hold is a live actuator.** Do not leave a powered hold unattended; end sessions by returning to P = 0° and pressing Stop. Removing power removes the hold — never let an electronic hold be the only thing between a load and the floor (§5).
- **Pushing the shaft against the hold makes the controller respond with its maximum available torque** (§5): displace it gently, by the flag or lever only, and briefly.
- **Holding under load is a heating condition** (Ch 7 §5): keep the Real-time Data temperature field visible during any long hold, plan the mechanism so holds happen at low-gravity-torque poses where possible, and stay inside the 12 A continuous / 28 A peak ratings no matter what the ±60 A telemetry scale can display (Ch 2 §9).
- The int16 telemetry position range (±3,200°) is narrower than the ±36,000° command range (Ch 3 §6) — check long multi-turn positions in the GUI's full position field, not the periodic telemetry frame, before commanding the next move.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §3.3.1.4 "Position Loop Mode" (p. 26: GUI operation — "enter the desired position P, and the motor will reach the desired position with maximum speed and acceleration"); §4.1 "Servo Mode Control Modes and Description" (p. 31: "Position Mode: A specified position is given to the motor, and the motor will move to the specified position (speed and acceleration are default to the maximum value)"; Control Mode ID 4 among the seven mode IDs; extended-frame ID layout); the CAN packet enum (p. 32: `CAN_PACKET_SET_POS`, `CAN_PACKET_SET_ORIGIN_HERE`); §4.1.5 "Position Loop Mode" (p. 35–36: the Simplified Control Diagram for Position Loop — position-loop Kp plus a differentiator-fed Kd, summed through torque-limit protection into `iq_ref` and the FOC current loop with `id_ref = 0`; the data-transmission definition — int32 payload, −360,000,000 … +360,000,000 representing −36,000° … +36,000°; the `comm_can_set_pos` example routine with its ×10,000.0 scaling); §4.1.6 "Setting Origin Mode" (p. 36: temporary vs permanent origin, `comm_can_set_origin` routine), cited for §2's origin discussion.
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — the 9 N·m rated / 22 N·m peak torque and 12 A / 28 A ratings cited in §5–§6 and the safety notes, and the 0.5701 N·m/A output torque coefficient used in §5's current estimate, via this series' reference table (Ch 1 §2) and Ch 2 §6.
3. **CubeMarsTool upper-computer software** — Servo Control P field and Real-time Data plots shown in Figure 9.1; © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.

---

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 10 — Position–Velocity Loop Mode →](10_Position_Velocity_Mode.md)
