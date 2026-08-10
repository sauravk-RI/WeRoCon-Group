# Chapter 12 — Choosing the Right Control Mode

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026.
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

Welcome to the final chapter! Eleven chapters ago, the AK80-9 was just a sealed metal cylinder with three connectors. Today, for you, it is ten different tools: seven Servo commands and three MIT patterns. You have tried each one on the bench, and you know what each one promises. Only one question is left — and it is the question every real project begins with:

> **Which mode should I use?**

This chapter answers that question. And here is some good news: there is nothing new to learn here. Every fact below was already explained somewhere in Chs 1–11. What this chapter adds is the *arrangement* — all the modes on one page, side by side, so you can compare them and choose. In the mode chapters, we always kept comparisons short and said "the full comparison is in Ch 12." Well, you have arrived. In order, we cover:

- the one-page master table: every mode, what you command, what the controller does for you (§1);
- a decision flowchart: *what do you actually need to control?* (§2);
- six worked examples that go from a real application to the right mode, with the reasoning shown (§3);
- Servo vs MIT: two different philosophies, and when to prefer each one (§4);
- nine short "head-to-head" boxes — one for each pair of modes that people often confuse (§5);
- the five cautions that follow you into *every* mode (§6);
- and finally, the section this whole series was built for: how these modes are used in **our own lab's research on powered prosthetic legs and lower-limb exoskeletons**, and why we chose this particular motor for it (§7).

There is no demo in this chapter. Every behavior mentioned here was already established in its home chapter — on the bench or in its telemetry discussion — and the pointers will take you straight back to the evidence.

## 1. The Master Table

Before the table, one simple idea that makes the whole table easy to read. Think of the modes as a **ladder**:

```text
        (top)   Position–Velocity  ← controller does the most for you
                Position
                Velocity
                Current (torque)
        (bottom) Duty Cycle        ← controller does the least for you
```

On the bottom step, Duty Cycle Mode, you hand the driver a raw electrical effort and the controller regulates nothing. Each step up, the controller takes over one more job: first it holds the current for you, then the speed, then the position, then even the shape of the trip. This is the "cascade" idea from Ch 9 §4 — each loop is built on top of the loop below it. MIT mode then steps *sideways* off the ladder: instead of picking one step, it hands you the whole control law — target, gains, and extra torque — in every single command (Ch 11 §1).

Now the table. Read it top to bottom as "you do more work → the controller does more work," with the three MIT patterns at the end as the flexible alternative:

| Mode (CAN ID) | You command | The controller regulates | Typical use | Main limitation | Be careful about |
|---|---|---|---|---|---|
| **Duty Cycle** (0, Ch 5) | a signed voltage effort, ±0.95 | nothing — open loop | first spins, testing, running your own external controller | speed and torque drift when the load changes, and nothing corrects them (Ch 5 §6) | current is *highest at low speed* — at starts and stalls (Ch 5 §4) |
| **Current Loop** (1, Ch 6) | a signed current $I_q$ in amps → torque ≈ 0.5701 × $I_q$ N·m (Ch 2 §6) | only the current | grippers with a force limit, tensioning, a torque input for your own outer loop | no speed or position target at all — motion just "happens" (Ch 6 §4) | a small current still reaches a high speed when unloaded; a stall makes heat (Ch 6 §5) |
| **Current Brake** (2, Ch 7) | an unsigned braking effort in amps | opposition to any motion | commanded stops, park-and-hold, slowing a load that wants to run away | no target and no memory: if pushed away, it stays away (Ch 7 §6) | holding = continuous $I^2R$ heat at zero speed (Ch 7 §5) |
| **Velocity Loop** (3, Ch 8) | a target speed in ERPM (÷189 = shaft rpm, Ch 3 §2) | the speed, using an internal PI loop (Ch 8 §1) | conveyors, wheels, fans — any "keep this rpm" job | acceleration is always the maximum; it never stops by itself | a jammed shaft or an impossible target = full effort forever (Ch 8 §6) |
| **Position Loop** (4, Ch 9) | one absolute angle in degrees, ±36,000° | the position, and the hold after arrival | joints, valves, pointers — when only "where" matters | **every trip is at maximum speed and acceleration** (Ch 9 §1) | check where the shaft is *before* commanding — the trip is the distance, not the number |
| **Set Origin** (5, Ch 4/9) | — (a utility: "this position = 0°") | — | setting the zero point that all position commands measure from (Ch 9 §2) | not a control mode | a temporary origin is lost when power is lost |
| **Position–Velocity** (6, Ch 10) | position + speed + acceleration | the position, moved along a smooth trapezoid profile | index tables, dispensers, lead screws (Multi), dials (Single) | still no choice of stiffness or damping per move | a smooth profile is not automatically a safe one; too big a load = full effort (Ch 10 §6) |
| **MIT Position** (8, Ch 11 §7) | $P_d$, $K_p$, $K_d$ in rad (with $V_d = 0$) | a spring–damper law: $\tau = K_p e_P - K_d V$ | compliant joints, where *you* choose stiffness and damping per command | range is only ±12.56 rad ≈ ±2 turns (Ch 11 §10) | a loaded hold sags by $\tau_L/K_p$; with $K_d = 0$ it oscillates (Ch 11 Demo 3) |
| **MIT Velocity** (8, Ch 11 §8) | $V_d$ and $K_d$ in rad/s (with $K_p = 0$) | speed, through $\tau = K_d(V_d - V)$ | speed control inside a streamed MIT scheme | runs slightly slow under load (error $\tau_L/K_d$) — no integral term | $K_d$ is *your* choice every command; Servo Velocity has no such offset |
| **MIT Torque** (8, Ch 11 §9) | a signed torque $\tau_{\mathrm{ff}}$ in N·m, ±18 | only the torque (the driver converts N·m to current for you) | direct torque in robotics units; gravity compensation | does not control speed — same story as Ch 6 §4, new units | ±18 N·m is what the *message* can say, not what the motor can safely do |

Two notes for reading the table. First, every mode above ends in the same place: an `iq_ref` handed to the same FOC current loop from Ch 2 §5. The modes only differ in *who computes that current, and from what* (Ch 9 §4 asked exactly this question with its own table). Second, every number range in this table is a message-encoding range, not a promise about the motor — that is the rule from Ch 2 §9, and it returns in §6.


## 2. The Decision Flowchart

When choosing a mode, do not ask "what should the motor do?" That question is too vague. Ask instead: **which quantity does my application actually specify?** Look at your requirement and find the number in it — is it a force? a speed? an angle? Then follow the chart:

```text
What do you actually need to control?
│
├─ A FORCE or a TORQUE (speed and position are free to float)
│   ├─ you think in amps, commanded from a Servo-style host ..... CURRENT LOOP (Ch 6)
│   ├─ you think in N·m, or you stream MIT frames ............... MIT TORQUE (Ch 11 §9)
│   └─ the real goal is "resist motion / stay put" .............. CURRENT BRAKE (Ch 7)
│
├─ A SPEED, held steady even when the load changes
│   ├─ one number, set-and-forget, no offset under load ......... VELOCITY LOOP (Ch 8)
│   └─ you want to choose the aggressiveness per command ........ MIT VELOCITY (Ch 11 §8)
│
├─ A POSITION, reached and then held
│   ├─ the journey does not matter (unloaded, robust setup) ..... POSITION LOOP (Ch 9)
│   ├─ the journey matters: you must set speed & acceleration ... POSITION–VELOCITY (Ch 10)
│   │     └─ turns add up? → Multi ·· only the facing matters? → Single (Ch 10 §§2–3)
│   └─ the stiffness/damping of the hold must be yours .......... MIT POSITION (Ch 11 §7)
│
├─ COMPLIANCE — the *feel* of the joint is part of the job
│   └─ spring–damper behavior, trajectories + feedforward ....... MIT, all five fields (Ch 11 §12)
│
└─ NOTHING — you want raw, unregulated drive
    └─ testing, characterization, your own control loop ......... DUTY CYCLE (Ch 5)
```

Three small forks that the chart squeezes together, spelled out. **Brake vs Velocity:** both involve motion, but they want opposite things. Velocity Mode *defends* motion — if the load slows the shaft, the controller pushes harder. Brake Mode *opposes* motion — any movement, in any direction, gets a counter-torque (Ch 7 §10, Ch 8 §8). **Brake vs Position for holding:** Brake asks *"is it moving?"*; Position asks *"is it where it should be?"* Scenario (d) below shows how to choose. **Servo vs MIT:** this fork appears at every level of the chart, because the real question there is not the quantity but the *philosophy* — that is §4.

## 3. Six Worked Examples

Rules choose modes badly; real requirements choose them well. Here are six applications. For each, we start from the requirement and reason our way to the mode. (You will notice several of them sound close to what we do in our own lab — §7 makes that connection complete.)

**(a) A robot-arm joint fighting gravity → MIT ($K_p$, $K_d$, $\tau_{\mathrm{ff}}$).** The requirement here is not just "reach this angle." The joint must reach the angle *gently*, carry its own weight, and be safe to touch. Servo Position can reach the angle, but it sprints there at full effort (Ch 9 §1) and holds with fixed gains you cannot see or change. Position–Velocity fixes the trip but not the hold. MIT Position gives you the stiffness and damping as numbers in your command — and $\tau_{\mathrm{ff}}$ pays the gravity bill directly, so the pose has no sag. Ch 11 §7 works exactly this situation in miniature: a $K_p = 20$ hold against a 1 N·m load droops by ~2.9° — until a 1 N·m feedforward arrives and the droop disappears. For a *moving* arm, stream all five fields: the trajectory as $P_d(t)$ and $V_d(t)$, the predicted dynamics as $\tau_{\mathrm{ff}}(t)$, and the gains set to the "feel" you want (Ch 11 §12). This is also how a powered knee behaves during walking — hold that thought for §7.

**(b) A conveyor belt or a drive wheel → Velocity Loop.** The requirement contains an rpm and the words "no matter the load." That is exactly the contract of Ch 8. Remember why nothing simpler works: Ch 6 §4.3 showed that a fixed current can *produce* a steady belt speed — but nobody is controlling that speed, so the first heavy box slows the belt permanently. Velocity Loop's internal PI controller rewrites the current the moment the load changes (Ch 8 §4 — the dip-and-recover story), and its integral action means there is no leftover speed error under load — something MIT Velocity's simpler law cannot promise (Ch 11 §8). One signed number, converted first: 189 × the shaft rpm you want (Ch 3 §2).

**(c) Tensioning, or a gripper with a force limit → Current Loop.** Here the requirement is a *force*, and the speed is explicitly allowed to float: film pays out at whatever rate the process pulls it; gripper jaws stop wherever the object is. Command $I_q = \tau^*/0.5701$ (Ch 6 §2), and the current limit *is* the force limit — at any position, at any speed, including zero. The "stalled gripper" state is Ch 6 §5's behavior working as a feature, as long as you respect the heat. Choose MIT Torque instead only if your controller already speaks N·m or streams MIT frames (§5, Box 8) — the physics is identical either way.

**(d) Park-and-hold against a load → Current Brake — or a Position hold. Here is the boundary.** Both can keep a loaded shaft still, and both pay the same $I^2R$ heating bill while doing it (Ch 7 §5). The deciding question is: *what should happen after a disturbance?* If the answer is "resist it, and wherever the shaft ends up is fine" — a winch pausing, a robot held on a ramp, "wait here while the rest of the machine works" — then Brake Mode does the job with one unsigned number and nothing to configure (Ch 7 §9). If the answer is "come back to the exact pose" — an indexed fixture, a joint that must stay *at* its angle — then you need target memory, and only a position hold has it. Put Ch 9's spring-back next to Ch 7's push-and-stay: that pair of behaviors is the whole difference (Ch 7 §6, Ch 9 §5). And remember: neither hold survives a power cut. A load that must never fall needs a mechanical brake.

**(e) An indexing table that needs smooth moves → Position–Velocity.** The requirement is an angle *plus* rules about the journey — because the mechanism on the shaft does not like being sprinted, and in practice almost nothing attached to a shaft does (Ch 10's rule of thumb). Plain Position Mode is ruled out by its own definition: maximum speed and acceleration, always (Ch 9 §1). Position–Velocity keeps the same "go to this absolute angle" meaning but lets you write the cruise speed and the ramps into every command (Ch 10 §1 — watch the speed plot on a long and a short trip, trapezoid then triangle, and you will see your profile really is yours). Multi Mode if the table's turns add up; Single Mode if only the facing matters (Ch 10 §4). MIT could also do this, but it costs five fields per move for flexibility this job does not need.

**(f) Characterizing a motor, or open-loop testing → Duty Cycle.** This time the requirement is the *absence* of control: you are measuring how the motor and mechanism respond, and any internal loop would reshape the very response you are trying to measure (Ch 5's "when to use," case 4). Step through a few duty values, log the current, speed, and bus voltage, and the back-EMF story writes itself in the telemetry (Ch 5 §4 and its demo). The same logic makes duty a natural output for a controller you build yourself — although for most external loops, commanding current is easier and safer, because the inner loop then handles the electrical dynamics for you (Ch 5's case 3, Ch 6's case 3).

## 4. Servo vs MIT: Two Philosophies

Every MIT comparison in §5 comes down to one distinction, so let us make it clear once. Ch 11 §14 named the two philosophies; here is how to *choose* between them.

**Servo control is delegation.** You name one target — a duty, a current, a speed, an angle — and the firmware's own controllers, with their own hidden gains, decide how hard to chase it. The interface is one to three numbers. You can *set it and forget it*, because the regulation keeps running by itself between your commands. This is the right philosophy when one well-regulated quantity is the whole job and you do not care about the *style* of the pursuit: the conveyor of scenario (b), the index table of (e), the gripper of (c).

**MIT control is specification.** Every frame states not only *where* and *how fast*, but also *how stiff*, *how damped*, and *how much extra torque* — the response itself becomes part of the command (Ch 11 §1). The price is that MIT expects an intelligent master: a computer composing five coupled fields, ideally streaming them quickly, and it is the least forgiving interface in this series — one typo in a gain field is a command to sprint or to oscillate (Ch 11's Safety Notes). The reward is that the joint's *mechanical personality* is yours, per command: a stiff servo in one frame, a soft compliant spring in the next, the same hardware throughout (Ch 11 §12). No Servo setting can offer that at any price.

A practical selector, in two questions. *Does the response matter?* If words like stiffness, damping, compliance, or "model-based feedforward" appear in your requirement, choose MIT — nothing else can express them. *Is there a master?* MIT assumes a computer streaming frames; Servo modes assume a target that can stand alone. A conveyor run from a PLC wants Servo Velocity; a walking robot's leg run from a 1 kHz controller wants MIT. In between, pick the simplest interface that can state your requirement — delegation is cheaper than specification whenever delegation is enough.

## 5. Nine Head-to-Head Boxes

Each box below settles one classic confusion between two modes. Each is short on purpose; the pointers carry the depth.

> **Box 1 — Duty Cycle ↔ Current Loop** *(Ch 5 ↔ Ch 6)*
> Same physics, opposite fixed point. Duty **pins the voltage**: as speed rises, back-EMF eats the voltage headroom, so the current — and the torque — get squeezed down, and the controller does nothing about it (Ch 5 §4). Current Loop **pins the current**: the loop notices $I_q$ sagging and spends *more* voltage to defend it, until the 48 V bus has nothing left to give (Ch 6 §4.2).
> A one-line memory aid: *duty lets physics set the torque; current loop pins it.*
> The low-speed consequence: in Duty mode, current at standstill is an uncontrolled inrush — the highest current the mode ever draws (Ch 5 §4 Stage 5). In Current Loop, a stalled shaft carries exactly the current you commanded — safe by design, but thermally expensive if you forget about it (Ch 6 §5).
> Choose Duty to *observe* the electrical chain, or to drive it from your own controller. Choose Current Loop to *use* torque as a controlled quantity.

> **Box 2 — Current Loop ↔ Current Brake** *(Ch 6 ↔ Ch 7)*
> The command ranges tell the whole story: signed −60…+60 A versus unsigned 0…60 A. Current Loop says *"I want this torque"* — you choose the size **and the direction**, and the motor will accelerate, brake, or reverse exactly as your sign says (Ch 6 §3). Current Brake says *"oppose motion, this hard"* — you choose only the effort; the firmware reads the encoder, sees which way the shaft moves, and pushes against it, every time (Ch 7 §3).
> A useful habit for any motor driver you meet: when a command range is one-sided, the controller is telling you it has kept the direction decision for itself (Ch 7 §3).
> Also: Brake stops the story at zero speed and switches to holding. Current Loop's signed command pushes *through* zero into reversal (Ch 6 Demo 2 vs Ch 7 §2).
> Small caution: the brake command is not documented as $I_q$, so torque estimates via 0.5701 N·m/A are estimates, not promises (Ch 7 §1).

> **Box 3 — Current Loop ↔ Velocity Loop** *(Ch 6 ↔ Ch 8)*
> Two different questions: *"how much torque?"* versus *"how fast?"* In Current Loop, you write the current command by hand, and the speed is whatever the load equation produces — steady only if the torques happen to balance, and defended by nobody (Ch 6 §4.3). Velocity Loop is the same machine **with a supervisor**: a PI controller measures the speed, computes the error, and continuously rewrites the very current command you used to write yourself (Ch 8 §1).
> The visible difference is one friction pad. Under Current Loop, the shaft slows and *stays* slow — the loop defends the current perfectly and the speed not at all. Under Velocity Loop, the shaft dips and *recovers*, with the current rising by itself to pay for your friction. Ch 6 §4.3 and Ch 8 §4 tell the two halves of that story.
> And remember: a small command is not a slow command in Current Loop (0.5 A → hundreds of rpm unloaded, Ch 6 §4.2). In Velocity Loop, a low target genuinely means a low speed.

> **Box 4 — Velocity Loop ↔ Position Loop** *(Ch 8 ↔ Ch 9)*
> *"How fast should I move?"* versus *"where should I stop?"* Velocity Mode has no destination: unless you send a new command, it spins at the target forever — stopping is simply not in its contract. Position Mode's whole job *is* stopping at the answer: the shrinking position error brakes the landing onto the target, and arrival begins a hold that rejects disturbances indefinitely (Ch 9 §3, §5).
> Structurally, Position wraps Velocity, which wraps Current — the completed cascade (Ch 9 §4). Nothing from Ch 8 is thrown away; a velocity toward the target is created *internally* on the way to every angle.
> Boundary cases: motion that never ends (conveying, spinning) is Velocity, even if you happen to have a position sensor. Motion that must end *somewhere specific* is Position, even if it spends most of its time moving.

> **Box 5 — Position Loop ↔ Position–Velocity** *(Ch 9 ↔ Ch 10)*
> Same destinations — absolute targets in the same ±36,000° coordinate, same hold after arrival — but opposite journeys. In Position Mode, the controller chooses the motion, and its choice is always **maximum speed and maximum acceleration** (Ch 9 §1's warning box). In Position–Velocity, the journey is part of the command: target + cruise speed + acceleration, shaped into a trapezoid (or a triangle, when the trip is too short to reach cruise speed; Ch 10 §1).
> Under the surface, Ch 10 adds exactly one block: a profile generator that feeds the Ch 9 cascade a slowly moving setpoint instead of the final target all at once (Ch 10 §1).
> The selection rule from Ch 10: the moment a real mechanism is attached to the shaft, Position–Velocity should be your *default* position mode. Plain Position Mode — one field instead of three — is for unloaded bench work and mechanisms that can truly take full effort. Ch 9's demo and Ch 10's Demo 1 run the very same 90° target so you can see the contrast: sprint versus glide.

> **Box 6 — Servo Position ↔ MIT Position** *(Ch 9 ↔ Ch 11 §7)*
> The manual's own diagrams give away the secret: Servo Position Mode is *already* a PD controller writing `iq_ref` — with its gains locked inside the firmware (Ch 9 §4). MIT Position is the same law, $\tau = K_p e_P - K_d V$, but with $K_p$ and $K_d$ shipped **inside every command**. Stiffness and damping stop being the firmware's business and become yours, per move (Ch 11 §7).
> What you gain: the same actuator, holding the same target, feels like a rubber band at $K_p = 5$ and like a rigid stop at $K_p = 100$ (Ch 11 Demo 3) — compliance as a command.
> What you trade: MIT position covers only ±12.56 rad ≈ ±2 turns, against Servo's ±100 turns (Ch 11 §10); a pure-spring hold sags by $\tau_L/K_p$ under load, fixable with $\tau_{\mathrm{ff}}$ (Ch 11 §7); and the tuning is now your responsibility — including always sending a nonzero $K_d$, or the step response rings (Ch 11 Demo 3).
> This is §4's philosophy at the position level: a simpler interface versus per-command authority.

> **Box 7 — Servo Velocity ↔ MIT Velocity** *(Ch 8 ↔ Ch 11 §8)*
> Both hold a speed by turning velocity error into torque. The difference is *whose law it is*. Servo Velocity: an internal **PI** controller — you send only the target, the firmware owns the gains, and the integral term slowly removes any leftover speed error, even under load (Ch 8 §1).
> MIT Velocity: the law is out in the open, and it is proportional only — $\tau = K_d(V_d - V)$ — with $K_d$ as *your* dial, per command, setting how aggressively error becomes torque (Ch 11 Demo 2).
> The cost of the simpler law: under a load $\tau_L$, the shaft settles slightly *below* target, with a standing error of $\tau_L/K_d$, because the error must stay nonzero to keep producing torque. You can cancel a known load with $\tau_{\mathrm{ff}}$, or accept the offset (Ch 11 §8).
> Rule of thumb: set-and-forget speed under changing load → Servo Velocity. Speed control *inside* a streamed MIT scheme, with per-command aggressiveness → MIT Velocity.

> **Box 8 — Current Loop ↔ MIT Torque** *(Ch 6 ↔ Ch 11 §9)*
> The same physical act — commanding the motor's torque — in two different units, over two different protocols. Current Loop: you command **amps** (Servo ID 1) and do the conversion yourself: $I_q^* = \tau^*/0.5701$, so a 2 N·m request becomes a 3.51 A command (Ch 6 §2). MIT Torque: you command **newton-metres** directly (ID 8 with $K_p = K_d = 0$, range ±18 N·m) — the only place this actuator accepts N·m over the wire — and the driver does the conversion for you, through the same FOC machinery and the same constants (Ch 11 §9, Ch 2 §11).
> Downstream, the behavior is identical, because the physics is: torque causes acceleration, never a particular speed — Ch 6 §4's story carries over word for word, with $\tau_{\mathrm{ff}}$ in place of $0.5701\,I_q$.
> Choose by ecosystem: a Servo-speaking host that thinks in amps → Current Loop. Robotics units, or torque as one term of a fuller MIT command → MIT Torque. And neither one is the GUI's `T` field — that is a braking shortcut over Brake Mode (Ch 6 §8).

> **Box 9 — Current Brake ↔ MIT (Torque)** *(Ch 7 ↔ Ch 11)*
> The classic beginner question: *"why not just use MIT Torque to brake?"* Answer: **who signs the torque.** MIT Torque is manual direction control — you specify $\tau_{\mathrm{ff}}$ *including its sign*. To brake a moving shaft you must know its direction, oppose it yourself, and remove the command at standstill — otherwise the shaft reverses (Ch 11 §9, and Ch 6 §3's brake-then-reverse story). Brake Mode is automatic: one unsigned number, the firmware reads the measured velocity, opposes it, and stops the story at zero speed, smoothly switching to holding (Ch 7 §§2–4).
> MIT *can* also hold — as a spring–damper via $K_p$/$K_d$, with target memory and a stiffness you choose (Ch 11 §7). That is richer than Brake's hold-wherever-it-stops, but it costs the full MIT interface and its ±2-turn window.
> Rule: "stop it / keep it still, simply" → Brake. "Push with this signed torque," or a hold whose *feel* is part of the job → MIT.

## 6. Five Cautions That Follow You Everywhere

Whichever mode you choose, these five rules — plus two documentation traps — come with you. Each one was fully explained once, in its home chapter; here they are as a pre-flight checklist.

1. **A protocol range is not a motor rating.** ±60 A, ±100,000 ERPM, ±327,680 ERPM, ±65 rad/s, ±18 N·m — these are what the *messages* can express. The motor's real limits are Ch 1 §2's table: 12 A continuous / 28 A peak, 9 / 22 N·m, 570 rpm no-load. Stated once for the series in Ch 2 §9; the MIT version — "range ≠ recommendation," so $K_p \le 500$ does not mean start at 500 — is Ch 11 §10.
2. **Holding makes heat.** Zero speed ≠ zero current ≠ zero heat: a motor holding a load dissipates $I^2R$ continuously, with no motion to carry the heat away — and the heat grows with the *square* of the current. Every mode that holds — Brake, Position, Position–Velocity, MIT — pays this bill in exactly the same way. Ch 7 §5 owns the physics; watch the temperature telemetry during any hold and you will see it for yourself.
3. **Divide ERPM by 189.** Every Servo speed and acceleration you type is *electrical*: divide by 189 (21 pole pairs × 9:1 gearbox) to get shaft rpm, and convert *before* commanding. 1,000 ERPM is a ~5.3 rpm crawl — and "fixing" a slow demo by typing a huge number toward an encoding limit is how shafts jump to hundreds of rpm. Ch 3 §2 owns the conversion.
4. **Plain Position Mode always moves at maximum speed and acceleration.** It is the mode's definition, not a setting you can soften (Ch 9 §1's warning box). Nearby targets, unloaded first contact, and check the current position before every command. Wanting a gentler trip means wanting Ch 10 or Ch 11 — not a different number in the same field.
5. **An active hold is not a lock.** Brake, Position, and MIT holds are electronics: power off = hold off, holding strength is finite, and a load stronger than the motor wins the tug-of-war at full current. Never let an electronic hold be the only thing between a load and the floor (Ch 7 §6, Ch 9 §5).
6. *Documentation trap 1:* the manual's MIT `pack_cmd` example clamps the number 0 instead of the variables `p_des`/`v_des` — as printed, it always transmits $P_d = V_d = 0$. Fix both lines before implementing the MIT protocol (Ch 11 §13).
7. *Documentation trap 2:* the manual's serial-port enum comments for `COMM_SET_POS_MULTI` (61) and `COMM_SET_POS_SINGLE` (62) appear to be swapped — trust the identifier names, and verify on the bench before trusting firmware to them (Ch 10 §7).

## 7. The AK80-9 in Our Lab: Prosthetic Legs and Exoskeletons

This series was written at the **Wearable Robotics & Control (WeRoCon) Laboratory, IIT Jodhpur**, where we design and build intelligent lower-limb prostheses and powered exoskeletons for people with mobility impairments — work carried out together with clinical partners such as AIIMS Jodhpur. So the honest final question of this series is not "which mode for a conveyor?" but: *why is this exact motor sitting on our bench, and which of its modes will our own research actually use?* Everything you have learned in Chs 1–11 comes together in the answer.

### 7.1 Why this motor? The quasi-direct-drive idea

A wearable robot has a hard set of demands that most industrial actuators fail. It must be **light**, because every gram is carried on a human leg. It must be **strong in torque**, because human joints work in newton-metres, not in rpm. It must be **backdrivable** — meaning a person can move the joint by hand, against only a small resistance — because the wearer's own leg must never feel trapped inside the machine. And it must offer **accurate, fast torque control**, because assistance is delivered as carefully timed torque, in step with the wearer's gait.

The AK80-9 belongs to a family of actuators built exactly for these demands, called **quasi-direct-drive (QDD)** actuators: a torque-dense, "pancake"-shaped motor combined with a deliberately *small* gear ratio — here, just 9:1 (Ch 1 §6). Compare that with a conventional robot joint, which may use a 100:1 or larger gearbox. That big gearbox multiplies torque, yes — but it also multiplies friction and reflected inertia, so the joint becomes stiff, noisy, and nearly impossible to backdrive by hand. The QDD approach accepts a smaller torque multiplication in exchange for a joint that stays *transparent*: the AK80-9 can be turned by hand with well under a newton-metre of static resistance, and published exoskeleton studies report only a few newton-metres of resistance even during fast, walking-like motion. A small gear ratio also means the motor "feels" the outside world clearly, so output torque can be estimated well from the motor current alone — $\tau_{\mathrm{out}} \approx 0.5701\,I_q$, Ch 2 §6 — without adding a heavy, expensive torque sensor at the joint. This is why the current-based torque control you practiced in Ch 6, and the MIT control of Ch 11, are not just conveniences: for wearable robots, they *replace hardware*.

You do not have to take our word for this choice. The AK80-9 is one of the most widely used actuators in lower-limb wearable-robotics research worldwide: the modular backdrivable hip, knee, and ankle exoskeleton M-BLUE (University of Michigan) is built around this exact actuator, and a recent AI-driven lower-limb exoskeleton from Georgia Tech, published in *Science Advances*, is fully powered by AK80-9 units. The same *actuation philosophy* — a high-torque motor with a low-ratio transmission — is behind leading powered knee–ankle prosthesis designs, which report benefits our own prototypes care about deeply: a knee that swings freely, compliance with the ground at foot contact, quiet operation, and even energy regeneration during the braking phases of gait. The manufacturer itself markets this actuator "especially" for exoskeletons and robot leg joints. In short: for a lab building prosthetic legs and lower-limb exoskeletons, the AK80-9 is close to the community's default choice — light (490 g), integrated (motor + gearbox + encoder + driver in one module, Ch 1 §1), torque-honest, backdrivable, and speaking both Servo and MIT protocols over a simple CAN bus.

### 7.2 Which modes for which lab tasks?

Now map our own workflow onto this chapter's decision framework. Notice how every stage of building a wearable robot lands on a mode you already know:

| Lab task | Mode | Why (in this chapter's language) |
|---|---|---|
| First checks of a new actuator; friction and inertia characterization | **Duty Cycle** (Ch 5), then **Current Loop** (Ch 6) | scenario (f): you want *no* internal loop shaping the response you are measuring |
| "Transparent" / zero-torque mode — the exoskeleton follows the wearer without helping or resisting | **MIT Torque** with $\tau_{\mathrm{ff}} \approx 0$ (or small compensation), or **Current Loop** near 0 A | the joint must be a pure, tiny torque source; the QDD's low friction is what makes ≈0 commanded torque feel like ≈0 resisted torque |
| Gait assistance: a torque profile delivered in time with the gait cycle (e.g., help at push-off) | **MIT Torque**, streamed — $\tau_{\mathrm{ff}}(t)$ follows the gait phase | the requirement is a *torque versus time*, not a position; Box 8's "torque in robotics units," commanded at high rate |
| A prosthetic knee during walking: stiff in stance (it must carry body weight), soft in swing (it must swing like a pendulum) | **MIT Position** with gait-phase-dependent $K_p$, $K_d$ | this is impedance control — scenario (a) at a human joint; the knee's stiffness and damping are re-commanded every gait phase, which only MIT can express (Ch 11 §7, §12) |
| Holding a joint still during donning/doffing, fitting, or between trials on the bench | **Current Brake** (Ch 7) — with caution 5 in mind | scenario (d): "stay put, wherever you are," one number, automatic engagement — but never as the only thing supporting a limb or a load |
| Bench rigs: repeatable joint sweeps for testing, sensor calibration, data collection | **Position–Velocity** (Ch 10) | smooth, repeatable trapezoid moves at speeds *you* chose — scenario (e) on a test stand |
| Treadmill-like or dynamometer-style test fixtures | **Velocity Loop** (Ch 8) | scenario (b): hold an rpm while the load (the leg being tested) varies |

Read the middle rows again, because they contain the reason this series ends with MIT mode as its flagship. Human walking is not a position problem or a speed problem — it is an **impedance** problem: at every moment, the joint must present the right *stiffness*, the right *damping*, and the right *bias torque*, and all three change with the phase of gait. That is, word for word, the MIT command frame: $\{P_d, V_d, K_p, K_d, \tau_{\mathrm{ff}}\}$, re-sent at high rate (Ch 11 §12). A gait controller in our lab is, at its lowest level, a program that tracks which point of the gait cycle the wearer is in and streams the matching five numbers to this motor. Every chapter of this series — the torque constants of Ch 2, the units of Ch 3, the heat rules of Ch 7, the cascade of Chs 6–9, and the master equation of Ch 11 — was preparing you to write, tune, and debug exactly that program.

### 7.3 One extra safety layer: there is a person in the loop

Everything in Ch 4 §6 and this chapter's §6 applies double when the actuator is attached to a human body. Three lab rules to carry forward. First, **all mode experiments happen on the bench first** — a control idea meets a wearer only after it has been fully proven on a fixture, and human trials in this lab happen only under the supervisor's approval and the applicable ethics protocols. Second, **know the power-loss behavior of every mode you use** (caution 5): a wearable joint must be mechanically safe — able to move or be moved — when the electronics stop, which is one more reason the backdrivable QDD design was chosen. Third, **respect the thermal budget** (caution 2): stance-phase support and long holds are exactly the "zero speed, real current" condition of Ch 7 §5, and a warm actuator strapped to a leg is both a comfort problem and a safety problem — watch the temperature telemetry in every wearable experiment.

## Key Takeaways

- The modes form a **ladder of delegation** — duty → current → velocity → position → profiled position — where each step up hands one more job to an internal loop; MIT steps sideways and hands you the whole control law in every frame (§1).
- Choose a mode by asking **which quantity your requirement actually specifies**: a force → Current Loop or MIT Torque; a maintained speed → Velocity Loop or MIT Velocity; an angle → Position, Position–Velocity, or MIT Position; "oppose motion" → Brake; a *feel* (compliance) → MIT with all five fields; nothing → Duty (§2).
- The recurring boundary questions have concrete answers: brake-hold vs position-hold depends on whether a disturbance should be *resisted* or *reversed* (scenario d); plain vs profiled position depends on whether anything fragile is attached (Box 5); Servo vs MIT depends on whether the response is part of the spec, and whether a master is streaming commands (§4).
- All ten tools drive the same machine: every mode ends by writing an `iq_ref` for the same FOC current loop, converted through the same two torque constants, obeying the same physics — torque causes acceleration, back-EMF caps speed, holding costs heat (Chs 2, 5–7).
- For our lab's research, the AK80-9's quasi-direct-drive design — light, torque-dense, backdrivable, torque-honest through its current — is why it sits at the joints of our prosthetic-leg and exoskeleton prototypes, and MIT impedance control is the mode our gait controllers ultimately speak (§7).
- The five cautions of §6 — encodings are not ratings, holding heats, ÷189, Position Mode sprints, holds are not locks — travel with you into every mode, every bench, and every prototype.

## Safety Notes

- This chapter adds no new hands-on procedures. If its comparisons send you back to the bench, re-enter each mode through its own chapter's demos, starting conditions, and safety notes — and Ch 4 §6's ground rules apply to every powered minute, in every mode.
- Mode *transitions* deserve care: command zero (0 A, S = 0, B = 0, or Stop) and let the shaft settle before switching modes. A leftover command from the old mode plus a first command in the new one is how surprises happen.
- When selecting a mode for a real application, check §6's cautions before any argument about elegance: worst-case load versus the *ratings* (not the encodings), the thermal cost of holding, and what happens at power loss.
- For wearable applications specifically: bench-prove everything first; know each mode's power-loss behavior before any human is attached; monitor temperature throughout; and involve the wearer and supervisor per §7.3 and the lab's protocols.

## Sources / References

1. **This tutorial series, Chapters 1–11** — this capstone is a synthesis: every mode behavior, demo result, equation, and caution above is established in its home chapter, principally: Ch 1 §§1–2, §6 (the integrated actuator, ratings table, 9:1 gearbox); Ch 2 §§5–6, §§9–11 (FOC, the two torque constants, encodings vs ratings, the full chain); Ch 3 §§2–6 (units and the ÷189 conversion); Ch 4 §§1–2, §6 (mode map, CAN Control Mode IDs, ground rules); Ch 5 §§1–7 (Duty); Ch 6 §§1–9 (Current Loop); Ch 7 §§1–10 (Brake, holding, heat); Ch 8 §§1–8 (Velocity); Ch 9 §§1–7 (Position and the cascade); Ch 10 §§1–8 (Position–Velocity); Ch 11 §§1–14 (MIT).
2. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — the mode-overview sections behind §1's table and §2's flowchart: §4.1 "Servo Mode Control Modes and Description" (p. 31: one-line definitions of all Servo modes, Control Mode IDs 0–6, and the maximum-speed/maximum-acceleration and motor-temperature notes quoted via Chs 5–10); §4.2 "Force Control Mode (MIT) Communication Protocol" (p. 37–39: single Control ID 8, the three MIT patterns, the master-equation diagram, and the AK80-9 parameter-range row); §3.3.1–3.3.2 (p. 24–29: per-mode GUI operation and ranges, cited via the mode chapters).
3. **CubeMars AK80-9 V3.0 KV100 product specifications and applications**, cubemars.com (accessed August 2026) — ratings cited in §1 and §6 via the series reference table (Ch 1 §2); manufacturer application guidance naming exoskeletons, legged robots, and robot limb joints, and the note on the Georgia Tech AI-driven lower-limb exoskeleton (published in *Science Advances*) powered by AK80-9 actuators, cited in §7.1.
4. **T. Elery, S. Rezazadeh, C. Nesler, and R. D. Gregg, "Design and Validation of a Powered Knee–Ankle Prosthesis With High-Torque, Low-Impedance Actuators,"** *IEEE Transactions on Robotics*, 2020 — the high-torque, low-reduction (quasi-direct-drive) actuation philosophy for powered prosthetic legs: backdrivability from small torques, free-swinging knee motion, ground compliance, low reflected inertia, quiet operation, energy regeneration, and accurate impedance/torque control without joint torque sensors; cited in §7.1.
5. **C. Nesler, G. Thomas, N. Divekar, E. J. Rouse, and R. D. Gregg, "Enhancing Voluntary Motion With Modular, Backdrivable, Powered Hip and Knee Orthoses"** (M-BLUE), *IEEE Robotics and Automation Letters*, 2022, and the follow-up modular backdrivable ankle exoskeleton work by the same group — partial-assist lower-limb exoskeleton modules built directly on the T-Motor AK80-9 quasi-direct-drive actuator, reporting sub-newton-metre static backdrive torque and low dynamic backdrive torque during walking-like motion; cited in §7.1.
6. **S. Yu et al., "Quasi-Direct Drive Actuation for a Lightweight Hip Exoskeleton With High Backdrivability and High Bandwidth,"** *IEEE/ASME Transactions on Mechatronics*, 2020 — the QDD paradigm for wearable robots (high-torque-density motor + low gear ratio) benchmarked against conventional and series-elastic actuation for torque capability, control bandwidth, backdrivability, and torque-tracking accuracy; cited in §7.1.
7. **Wearable Robotics & Control (WeRoCon) Laboratory, IIT Jodhpur** — laboratory research context for §7 (intelligent lower-limb prostheses, powered exoskeletons, and rehabilitation robotics, in collaboration with clinical partners including AIIMS Jodhpur): home.iitj.ac.in/~sauravk (accessed August 2026).