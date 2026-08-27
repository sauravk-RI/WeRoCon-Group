# Chapter 12 — Choosing the Right Control Mode

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

You now know ten ways to control the AK80-9: seven Servo commands and three MIT patterns. This chapter helps you pick the right one for a real job. Nothing here is new — every fact was already explained in Chapters 1–11. This chapter just puts everything side by side so you can compare and choose.

> **Which mode should I use?**

Here is what this chapter covers:

- a one-page table of every mode, plus common mix-ups (§1);
- six short worked examples (§2);
- Servo vs MIT, and how to choose (§3);
- nine short comparisons between modes people often mix up (§4);
- five cautions that apply to every mode (§5);
- how our own lab uses these modes for prosthetic legs and exoskeletons (§6).

There is no hands-on demo here — every point links back to its home chapter.

## 1. The Master Table


| Mode (CAN ID) | You send | It controls | Good for | Watch out for | Type |
|---|---|---|---|---|---|
| **Duty Cycle** (0, Ch 5) | a voltage fraction, −0.95 to +0.95 | nothing — open loop | quick tests | current is highest at start-up and stalls later, not at speed (Ch 5 §4) | SISO |
| **Current Loop** (1, Ch 6) | a current in amps (torque ≈ 0.57 × current, Ch 2 §6) | only the current, which sets the torque | grippers with a force limit, tensioning — <span style="background-color:#ffe58a;">also the way to control torque from the Raspberry Pi, since Servo has no separate torque mode</span> (§3.1) | no speed or position limit — an unloaded shaft can spin fast (Ch 6 §4) | SISO |
| **Current Brake** (2, Ch 7) | a braking amount in amps (always positive) | pushes back against motion, whichever way the shaft is moving | quick stops, or holding a shaft still for a short time | no memory of where it started — pushed to a new spot, it just holds the new spot; holding makes heat (Ch 7 §5) | SISO |
| **Velocity Loop** (3, Ch 8) | a target speed in ERPM (÷189 = shaft rpm, Ch 3 §2) | speed, using a built-in PI controller | conveyors, wheels, fans — anything that should spin at a steady rpm | never stops by itself (Ch 8 §6) | SISO |
| **Position Loop** (4, Ch 9) | one target angle, in degrees | position, and holds it after arrival | joints, valves, pointers — when only the end position matters | every move happens at full speed and full acceleration (Ch 9 §1) — check where the shaft is before sending a new target | SISO |
| **Set Origin** (5, Ch 4/9) | nothing to control — marks "this position = 0°" | — | setting the zero point that position commands measure from (Ch 9 §2) | not a control mode; a temporary origin is lost on power loss | — |
| **Position–Velocity** (6, Ch 10) | position + speed + acceleration, together | goes to commanded position with the selected velocity and accelaration (Ch 10 §1) | index tables, dispensers — anywhere a gentler, controlled move matters | you still can't choose how firm or soft the hold feels | MISO |
| **MIT Position** (8, Ch 11 §7) | target position $P_d$, stiffness $K_p$, damping $K_d$ | a spring-like hold: torque $= K_p \times$ position error $- K_d \times$ velocity | joints that need an adjustable, springy feel | under load it settles a little short of target unless you add feedforward torque; <span style="background-color:#ffe58a;">you can raise the $K_p$ to a high value like 80-100 to hold the shaft in the desired position against some external torque - but only once the shaft has arrived — a high $K_p$ sent from far away causes a sudden, violent move</span> (§3.1, Ch 11 Demo 3) | MISO |
| **MIT Velocity** (8, Ch 11 §8) | target speed $V_d$ and damping $K_d$ | speed: torque $= K_d \times$ (target speed − speed) | speed control | runs a little slow under load, with no automatic correction (Ch 11 §8) | MISO |
| **MIT Torque** (8, Ch 11 §9) | a signed torque directly, in N·m, up to ±18 | only the torque — the motor converts it to current for you | robotics code that already thinks in torque; gravity compensation | doesn't control speed at all — an unloaded shaft will accelerate (Ch 6 §4) | MISO |

A few notes on the table:

- Every mode ends the same way: it sends a current to the same core control loop inside the motor (Ch 2 §5). The modes only differ in who computes that current, and from what.
- Every range shown here (like ±60 A or ±18 N·m) is what the *message* allows, not what the motor can safely handle — see §5.
- **SISO** means you send one number and control one thing. **MISO** means several numbers work together to control one thing. Position–Velocity and MIT are MISO. <span style="background-color:#ffe58a;">In MIT, you can send position alone, speed alone, torque alone, or mix them — if you mix position and speed, keep the damping gain ($K_d$) smaller than the stiffness gain ($K_p$), or the shaft will fight itself instead of settling smoothly near the target.</span>

A few common mix-ups, explained simply:

- **Brake vs Velocity:** Velocity Mode pushes to *keep* the shaft moving. Brake Mode pushes to *stop* it. They react to a load in opposite ways.
- **Brake vs a Position hold:** Brake just resists movement — pushed away, it stays at the new spot. A Position hold remembers the target and pulls back to it. Scenario (d) below shows how to pick.
- **Brake vs a high-stiffness MIT hold:** <span style="background-color:#ffe58a;">A MIT hold with a high $K_p$ resists a push almost like Brake does, but it also remembers its target, like a Position hold</span> (§3.1).
- **Servo vs MIT:** this choice is really about how much control you want over the motor's *feel* — that's §3.

## 2. Six Worked Examples

**(a) A robot-arm joint fighting gravity → MIT.** Servo Position can reach the angle, but always at full speed, with a firmness you can't change. MIT lets you set the stiffness ($K_p$), the damping ($K_d$), and add a steady push (feedforward torque) to cancel gravity, so the arm doesn't dip under its own weight. Keep $K_p$ low while the arm is still moving, and only raise it once it has arrived — otherwise the move becomes sudden and jerky (§3.1). A robotic knee works the same way: soft while swinging, firm while carrying weight (§6).

**(b) A conveyor belt or drive wheel → Velocity Loop.** A conveyor needs a steady rpm no matter the load. Velocity Loop's built-in controller keeps adjusting the current to hold that speed, and removes any leftover speed error over time — something MIT Velocity's simpler rule can't fully do (Ch 11 §8). Send 189 × the rpm you want.

**(c) Tensioning, or a gripper with a force limit → Current Loop.** Here you want a fixed force and don't care about speed or position — the current *is* the force limit (torque ≈ 0.57 × current). MIT Torque works too, if your code already speaks torque and MIT frames.

**(d) Park-and-hold against a load → Current Brake, or a Position hold.** Brake holds against a push but doesn't return to a target afterward — good for "just stay still, wherever that is." A Position hold remembers and returns to the exact spot — better when the joint must stay at one exact angle. A MIT hold with a high $K_p$ can do the same job as a Position hold, with the option to soften later (§3.1). None of these survive a power cut — use a mechanical brake if a falling load is a real risk.

**(e) An indexing table that needs smooth moves → Position–Velocity.** Plain Position Mode always moves at full speed — not good if something fragile is attached. Position–Velocity lets you set the speed and acceleration of the move, so it's smoother. Use Multi Mode if turns add up (a lead screw); use Single Mode if only the final angle matters (a dial).

**(f) Characterizing a motor, or open-loop testing → Duty Cycle.** To see the motor's raw behavior, use Duty Cycle — no internal loop changes what you're measuring. For most other custom control loops, Current Loop is easier, because it handles the electrical side for you.

## 3. Servo vs MIT: Two Ways of Working

**Servo control is simple.** You give one target — a duty, a current, a speed, an angle — and the motor's own controller does the rest, with fixed settings you can't see or change. This is enough when only one thing matters, and you don't care about *how* the motor gets there: the conveyor of (b), the index table of (e), the gripper of (c).

**MIT control asks more of you, but gives more back.** You also choose *how* the motor should respond: how stiff, how damped, how much extra push. You need a computer sending five values, often many times a second — this is the least forgiving mode in the series, since a typing mistake in a gain can make the motor sprint or shake (Ch 11's Safety Notes). In return, you can change the joint's feel from one command to the next.

A simple way to choose: if your requirement includes words like stiffness, damping, or springiness, use MIT — nothing else can do that. If one well-behaved quantity is the whole job, and there's no computer streaming commands, use a Servo mode — it's simpler.

**Rigid or springy — the feel of the hold.** Every Servo hold — Position, Position–Velocity — is firm: once the shaft arrives, it won't budge, and the motor uses as much torque as it needs to keep it exactly there (Ch 9 §5). MIT is springy by default: it gives a little under a push and returns, like a real leg joint. But <span style="background-color:#ffe58a;">you can make a MIT hold feel just as firm as Servo Position — raise $K_p$ to something high, like 80–100</span> (Box 6 has more detail). <span style="background-color:#ffe58a;">The important rule: only raise $K_p$ once the shaft has already arrived and needs to hold against a push. If you send a high $K_p$ while the shaft is still far from the target, it will snap toward the target violently instead of moving there smoothly.</span>

**Which joint needs which feel.** Different joints on the same machine can need opposite answers. <span style="background-color:#ffe58a;">A joint that must not buckle under load — an exoskeleton hip holding the leg upright — wants Servo Position's firm hold, or MIT with $K_p$ set high. A joint that must give and rebound — a prosthetic or exoskeleton knee, swinging and absorbing small shocks — wants the springy, adjustable feel that only MIT gives</span>, with a moderate $K_p$ that changes through the walking cycle. §6.2 shows this in our own lab's work.

### 3.1 A Practical Note: MIT Mode vs. Impedance Control on the Pi

Two things worth knowing before writing control code for this motor on a Raspberry Pi (Chapters 13–14).

**First:** <span style="background-color:#ffe58a;">the Servo command set has no separate "torque mode." Current Loop *is* how you control torque</span>, because torque ≈ 0.57 × current (Ch 2 §6). Whenever this chapter, or your own code, needs torque control through the Servo commands, Current Loop (ID 1) is the answer.

**Second:** <span style="background-color:#ffe58a;">in our own testing, we found bugs in the motor's own MIT mode (Control ID 8) when driving it directly from the Raspberry Pi</span> — sending $P_d$, $V_d$, $K_p$, $K_d$, $\tau_{\mathrm{ff}}$ and letting the motor's firmware compute the torque itself (Ch 14 §1.1). <span style="background-color:#ffe58a;">So instead, our own code computes the same spring-damper equation in Python, on the Pi, and sends the result as an ordinary Current Loop command</span>:

$$\tau = \tau_{\mathrm{ff}} + K_p(P_d - P) + K_d(V_d - V)$$

This is computed fresh every control cycle from the motor's own telemetry, converted to current (current = torque ÷ 0.57), and sent as a normal Current Loop frame (ID 1). Chapter 14 §4.2 shows the working code; Chapter 15's knee–ankle controller is built the same way.

Is this the same as "MIT mode"? <span style="background-color:#ffe58a;">Almost — it's the same equation, just running in a different place.</span> This equation is called *impedance control* in general — a well-known idea from control theory (Hogan, 1985): instead of a fixed target, you command a spring-damper relationship, centered on a moving target. "MIT mode" is one specific version of it — the same equation, running inside the motor's own firmware, using the five-value frame the MIT Biomimetic Robotics Lab designed for their Mini Cheetah project (Ch 11 §1). So wherever this chapter says "MIT Position" or "MIT/impedance control" about our own hardware, it means this Pi-side version, not literally Control ID 8.

## 4. Nine Head-to-Head Comparisons

> **1 — Duty Cycle vs Current Loop** *(Ch 5 vs Ch 6)*
> Duty Cycle fixes the voltage. As the motor speeds up, back-EMF eats into that voltage, so current — and torque — drop, and nothing corrects this. Current Loop fixes the current instead: if it starts to drop, the controller raises the voltage to hold it steady.
> Simple way to remember: Duty lets physics decide the torque; Current Loop holds the torque steady.
> Use Duty to see the motor's raw behavior, or to run your own outer control loop. Use Current Loop when you want torque as a controlled value.

> **2 — Current Loop vs Current Brake** *(Ch 6 vs Ch 7)*
> Current Loop takes a signed value (−60 to +60 A) — you choose the size and the direction. Current Brake takes only a positive value (0 to 60 A) — you choose how hard to resist, and the motor figures out which direction to push against.
> Current Loop can push the shaft through zero and reverse it. Brake stops at zero speed and holds there.
> Note: Brake's number is not officially documented as current, so torque estimates from it are rough guesses only (Ch 7 §1).

> **3 — Current Loop vs Velocity Loop** *(Ch 6 vs Ch 8)*
> Current Loop controls force, not speed — the shaft's speed just settles wherever the load allows. Velocity Loop adds a controller on top: it watches the speed and keeps adjusting the current to hold a target speed steady, even if the load changes.
> Remember: a small current in Current Loop can still mean a fast, unloaded shaft — don't assume a small number means slow motion.

> **4 — Velocity Loop vs Position Loop** *(Ch 8 vs Ch 9)*
> Velocity Loop never stops by itself — it just keeps spinning at the target speed. Position Loop is built to stop exactly at a target angle, and holds there afterward.
> Use Velocity for motion that doesn't end (a wheel, a fan). Use Position for motion that must end at a specific spot.

> **5 — Position Loop vs Position–Velocity** *(Ch 9 vs Ch 10)*
> Both move to an absolute angle and hold there afterward. Position Loop always moves at full speed and full acceleration — you can't change that. Position–Velocity lets you set the speed and acceleration of the move.
> Rule of thumb: if something real (not just a bare shaft) is attached, use Position–Velocity by default.

> **6 — Servo Position vs MIT Position** *(Ch 9 vs Ch 11 §7)*
> Both work the same way inside: a position error, plus some damping, produces torque. Servo Position uses fixed values you can't see. MIT Position lets you set the stiffness ($K_p$) and damping ($K_d$) yourself, every time.
> At a low $K_p$ (around 5), MIT feels like a soft spring. At a high $K_p$ (around 100), it feels just as firm as Servo Position.
> Trade-off: MIT only covers about ±2 turns (Servo covers ±100 turns), and a loaded hold settles a little short of the target unless you add feedforward torque. You must also set the damping ($K_d$) yourself — without it, the shaft overshoots and swings back and forth around the target.
> Because you can dial the stiffness up or down, MIT Position can act like Servo Position — but Servo Position cannot act like a soft MIT hold.

> **7 — Servo Velocity vs MIT Velocity** *(Ch 8 vs Ch 11 §8)*
> Servo Velocity uses a built-in PI controller — it slowly removes any leftover speed error, even under load. MIT Velocity uses a simpler rule you control yourself (just the $K_d$ gain) — under load, it settles a little below target and stays there.
> Use Servo Velocity for steady speed control on its own. Use MIT Velocity when you're already streaming MIT commands and want to set how strongly it reacts yourself.

> **8 — Current Loop vs MIT Torque** *(Ch 6 vs Ch 11 §9)*
> Both control torque, just in different units. Current Loop: you send amps and convert to torque yourself (torque ≈ 0.57 × current). MIT Torque: you send newton-metres directly, and the motor converts it to current for you.
> The behavior is the same either way — torque changes speed, but does not set a specific speed.
> On this lab's Raspberry Pi code, MIT-style torque commands are actually sent as Current Loop values, computed on the Pi (§3.1).

> **9 — Current Brake vs MIT (Torque)** *(Ch 7 vs Ch 11)*
> MIT Torque needs you to set the direction yourself — you must know which way the shaft is moving, push against it, and turn the command off once it stops, or it will push the shaft the other way. Current Brake does this automatically: it reads the direction from the encoder and always resists it, then holds once the shaft stops.
> MIT can also hold like Brake, but with a stiffness you choose and a memory of the target — a high-$K_p$ MIT hold behaves a lot like Brake, but remembers where it should be (§3.1).

## 5. Five Cautions That Follow You Everywhere

1. **A protocol range is not a motor rating.** The message may allow ±60 A, ±18 N·m, and so on — but the motor's real limits are 12 A continuous / 28 A peak, 9 / 22 N·m, 570 rpm no-load (Ch 1 §2, Ch 2 §9). Same for MIT's gain ranges (Ch 11 §10).
2. **Holding still still uses current, and current makes heat.** Brake, Position, Position–Velocity, and MIT all pay this cost while holding — even at zero speed (Ch 7 §5).
3. **Divide ERPM by 189 to get shaft rpm** (21 pole pairs × 9:1 gearbox, Ch 3 §2). 1,000 ERPM is only about 5 rpm — a common mistake is typing a huge number to "fix" a slow demo.
4. **Plain Position Mode always moves at full speed and acceleration.** This can't be changed — it's the mode's definition (Ch 9 §1). Use Position–Velocity or MIT for a gentler move.
5. **A hold is not a mechanical lock.** It stops working the instant power is lost, and it can only push so hard before a strong enough load overpowers it (Ch 7 §6, Ch 9 §5).
6. *Documentation trap:* the manual's example MIT code (`pack_cmd`) has a bug — it always sends position and speed as zero. Fix this before using it (Ch 11 §13).
7. *Documentation trap:* the manual's names for `COMM_SET_POS_MULTI` (61) and `COMM_SET_POS_SINGLE` (62) look swapped. Trust the ID names, and test on the bench first (Ch 10 §7).

## 6. The AK80-9 in Our Lab: Prosthetic Legs and Exoskeletons

This series was written at the **Wearable Robotics & Control (WeRoCon) Laboratory, IIT Jodhpur**, where we build powered lower-limb prostheses and exoskeletons, with clinical partners including AIIMS Jodhpur. Here's why this motor, and which of its modes our own research actually uses.

### 6.1 Why this motor?

A wearable robot needs to be light, strong in torque, easy to move by hand (backdrivable), and able to control torque accurately and quickly.

The AK80-9 is a **quasi-direct-drive (QDD)** actuator: a motor that produces high torque for its size, combined with a small gear ratio (9:1, Ch 1 §6) instead of a large one like 100:1. A small gear ratio means less friction, so the joint stays easy to move by hand, and it lets us estimate output torque directly from motor current (torque ≈ 0.57 × current, Ch 2 §6) — no separate torque sensor needed.

This actuator is widely used in this field: it powers exoskeletons like M-BLUE (University of Michigan) and a lower-limb exoskeleton from Georgia Tech, and the same approach — high torque, low gear ratio — is behind several powered prosthetic knee designs. It is light (490 g), has the motor, gearbox, encoder, and driver built into one unit, and speaks both Servo and MIT protocols over CAN.

### 6.2 Which modes for which lab tasks?

| Lab task | Mode | Why |
|---|---|---|
| First checks on a new motor | Duty Cycle, then Current Loop | no internal loop changes what you're measuring — scenario (f) |
| Gait assistance — a torque profile timed to the walking cycle | MIT Torque, streamed — in practice, Current Loop computed on the Pi (§3.1) | the requirement is torque over time, not a position |
| Exoskeleton hip during standing — must not buckle under body weight | Servo Position, or MIT with a high $K_p$ | the joint should hold firmly, like scenario (a) |
| Knee during walking — firm in stance, springy in swing | MIT Position with changing $K_p$/$K_d$ — run as Pi-side impedance control over Current Loop (§3.1) | the stiffness needs to change through the walking cycle |
| Holding a joint still for fitting, or between bench tests | Current Brake Mode / <span style="background-color:#ffe58a;">Impedance Control Mode (High $K_p$)</span> | simple "stay put" — never the only thing holding a load, though |
| Bench tests: repeated, smooth moves | Position–Velocity | smooth, repeatable moves at a speed you set |
| Test rigs needing a steady rpm | Velocity Loop | holds speed steady even as the load changes |

Walking is really a stiffness-and-damping problem, not just a position or speed problem — the joint's stiffness needs to change through the gait cycle. That's exactly what MIT/impedance control gives us, run through the Pi as explained in §3.1.

## Key Takeaways

- Servo modes (Duty, Current, Brake, Velocity, Position) each take one number and control one thing (SISO). Position–Velocity and MIT take several numbers together (MISO) — MIT is the most flexible, letting you mix position, speed, and torque in one command (§1).
- Pick a mode by asking what your application actually needs.
- Servo Position always holds firmly. MIT holds springily by default, but a high $K_p$ — set only once the shaft has arrived, never during a move — makes it just as firm. This is why a rigid hip and a springy knee can both run on this same motor's MIT interface (§3).
- The Servo command set has no separate torque mode — Current Loop is torque control. On this lab's own hardware, MIT-style control also runs through Current Loop, computed on the Raspberry Pi, because of firmware issues found in the motor's own MIT mode (§3.1).
- All ten modes end up sending a current to the same core control loop inside the motor (Ch 2 §5) — they only differ in who computes that current, and how.
- The AK80-9's light weight, ease of moving by hand, and torque-from-current design are why it fits prosthetic legs and exoskeletons, and MIT/impedance control is the mode our gait controllers ultimately use (§6).

## Sources / References

1. **This tutorial series, Chapters 1–11** — this chapter is a synthesis: every mode behavior, demo result, equation, and caution above is established in its home chapter, principally: Ch 1 §§1–2, §6 (the integrated actuator, ratings table, 9:1 gearbox); Ch 2 §§5–6, §§9–11 (FOC, the two torque constants, encodings vs ratings); Ch 3 §§2–6 (units and the ÷189 conversion); Ch 4 §§1–2, §6 (mode map, CAN Control Mode IDs, ground rules); Ch 5 §§1–7 (Duty); Ch 6 §§1–9 (Current Loop); Ch 7 §§1–10 (Brake, holding, heat); Ch 8 §§1–8 (Velocity); Ch 9 §§1–7 (Position); Ch 10 §§1–8 (Position–Velocity); Ch 11 §§1–14 (MIT).
2. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — the mode-overview sections behind §1's table: §4.1 "Servo Mode Control Modes and Description" (p. 31); §4.2 "Force Control Mode (MIT) Communication Protocol" (p. 37–39); §3.3.1–3.3.2 (p. 24–29, per-mode GUI operation and ranges).
3. **CubeMars AK80-9 V3.0 KV100 product specifications and applications**, cubemars.com (accessed August 2026) — ratings cited in §1 and §5; manufacturer application guidance naming exoskeletons, legged robots, and robot limb joints, and the Georgia Tech AI-driven lower-limb exoskeleton (published in *Science Advances*) powered by AK80-9 actuators, cited in §6.1.
4. **T. Elery, S. Rezazadeh, C. Nesler, and R. D. Gregg, "Design and Validation of a Powered Knee–Ankle Prosthesis With High-Torque, Low-Impedance Actuators,"** *IEEE Transactions on Robotics*, 2020 — the high-torque, low-reduction actuation approach for powered prosthetic legs, cited in §6.1.
5. **C. Nesler, G. Thomas, N. Divekar, E. J. Rouse, and R. D. Gregg, "Enhancing Voluntary Motion With Modular, Backdrivable, Powered Hip and Knee Orthoses"** (M-BLUE), *IEEE Robotics and Automation Letters*, 2022 — a partial-assist lower-limb exoskeleton built on the same AK80-9 actuator, cited in §6.1.
6. **S. Yu et al., "Quasi-Direct Drive Actuation for a Lightweight Hip Exoskeleton With High Backdrivability and High Bandwidth,"** *IEEE/ASME Transactions on Mechatronics*, 2020 — the QDD approach for wearable robots, cited in §6.1.
7. **Wearable Robotics & Control (WeRoCon) Laboratory, IIT Jodhpur** — laboratory research context for §6: home.iitj.ac.in/~sauravk (accessed August 2026).
