# Chapter 11 — MIT (Force) Control Mode

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In all the earlier chapters, each control mode gave you **one thing** to control: a duty ratio, a current, a speed, or a position. This chapter gives you five things at once. Do not worry — it is much less scary than it sounds, and by the end of this chapter it will feel natural.

**MIT (Force) Control Mode** — CubeMars also calls it **Force Control Mode** or **Motion Control Mode** — uses Control Mode ID 8. Every MIT command carries five values together: a desired position $P_d$, a desired velocity $V_d$, a stiffness gain $K_p$, a damping gain $K_d$, and a feedforward torque $\tau_{\mathrm{ff}}$. The driver combines these five values in one simple equation and turns them into a torque request. Depending on which of the five values you actually use, the same command can behave like a position controller, a velocity controller, a torque controller — or, most interestingly, like a **programmable spring and damper**. In other words, you do not just tell the motor *where* to go. You also tell it *how it should feel* while going there and while staying there: soft like a rubber band, or stiff like a solid rod. That is something no Servo mode can do.

This is the mode the whole series has been building toward. In Ch 9 §4 we already saw its skeleton hiding inside Servo Position Mode: a control law of the form "torque = gain × position error + damping," with the gains fixed inside the firmware. MIT mode is the same law, but now the gains travel *inside every command*. The stiffness and the damping are no longer the firmware's choice — they are **your** choice, for every single move. This is why MIT mode is the favorite mode for walking robots, robot arms, and any joint that must touch and interact with the real world gently instead of just overpowering it.

**One note about naming, given once, for the whole chapter.** CubeMars describes three ways of using MIT control, but sadly it names them inconsistently in different parts of the manual. The GUI section calls them Force Control Position / Velocity / Torque Mode (manual §3.3.2). The protocol section on p. 38 calls the very same three things "Position Loop Mode," "Velocity Loop Mode," and — for the torque one — "**Current Loop Mode**." (The CAN examples on p. 53 even label the MIT torque values as "IQ Current.") This is confusing, because those names sound exactly like the Servo modes from Chs 5–10, which are completely different things. So, to keep everything clear, this series always calls them **MIT Position**, **MIT Velocity**, and **MIT Torque**. Whenever the manual says Position/Velocity/Current Loop Mode under the Force Control heading, it means these three.

Here is what this chapter covers, in order:

- why MIT control exists: one command, one Control ID, one master equation (§1);
- the position term, and $K_p$ as a virtual spring (§2);
- the velocity term, and $K_d$ as a virtual damper (§3);
- feedforward torque (§4), and all three terms working together (§5);
- why the three "modes" are really just settings of one equation (§6);
- MIT Position control in practice: why $V_d = 0$, why $K_d$ matters so much, and what "holding" really means (§7);
- MIT Velocity control (§8) and MIT Torque control (§9);
- the AK80-9's parameter ranges — and why a range is not a recommendation (§10);
- units (§11), and the spring–damper mental picture, plus a small look beyond the three basic patterns (§12);
- the CAN protocol: how the five values are packed, and a real bug in the manual's example code (§13);
- and finally, MIT vs Servo: two different ways of thinking (§14);

followed by three bench demos — the "feel the gains with your own hands" experiments that are the heart of this mode — advice on when (and when not) to use it, safety notes, and sources. As always, everything assumes a connected, calibrated motor and the safety ground rules from Ch 4 §6.

## 1. Why MIT Control Exists: One Command, One Equation

In Servo control, choosing what to control means choosing a mode from a menu: Current Loop (ID 1), Brake (ID 2), Velocity (ID 3), Position (ID 4), Position–Velocity (ID 6). MIT control replaces that whole menu with a **single command frame** under **Control Mode ID 8** (manual §4.2, p. 37–38). This frame always carries the same five values. What the motor does is decided not by *which* frame you send, but by *what you put inside it*. The manual says this directly: "The MIT Control Mode has three control modes, all sharing a single Control ID number, with the specific control mode determined by the transmitted data" (§4.2, p. 37).

Inside the driver, the five values are combined into a torque request by the **master equation** of this chapter — and honestly, of the whole series:

$$
\boxed{\;\tau_{\mathrm{cmd}} = K_p\,(P_d - P) \;+\; K_d\,(V_d - V) \;+\; \tau_{\mathrm{ff}}\;}
$$

Here are all the symbols, in one table you can come back to at any time:

| Symbol | Meaning | Unit | Where it comes from |
|---|---|---|---|
| $P_d$, $V_d$ | desired position, desired velocity | rad, rad/s | your command |
| $P$, $V$ | actual position, actual velocity | rad, rad/s | the encoder, measured continuously |
| $K_p$ | position gain — the virtual spring (§2) | acts as N·m/rad | your command |
| $K_d$ | velocity gain — the virtual damper (§3) | acts as N·m/(rad/s) | your command |
| $\tau_{\mathrm{ff}}$ | feedforward torque (§4) | N·m | your command |
| $\tau_{\mathrm{cmd}}$ | the resulting torque request | N·m | the sum, computed every control cycle |

So the torque request is the sum of three parts: a **position-error torque**, a **velocity-error torque**, and a **feedforward torque**. We will look at each part one by one in §§2–4, and then put them together in §5.

Everything that happens *after* $\tau_{\mathrm{cmd}}$ is machinery you already know from Ch 2. The manual's own simplified diagram for Force Control Mode (p. 38) draws exactly this equation: the position error scaled by $K_p$, added to a torque reference and the $K_d$ velocity path, then passed through a torque-limit protection block into `iq_ref`, which feeds the FOC current loop with `id_ref = 0`. It is the same chain as Ch 2 §10, with the MIT sum sitting on top as the outermost stage:

```text
{Pd, Vd, Kp, Kd, τff} ──► MIT sum ──► τcmd ──► torque limit ──► iq_ref ──► FOC ──► phase currents
                                                                            ──► motor torque ──► 9:1 gearbox ──► output shaft
```

Notice something nice: MIT mode never asks you for a current. You speak in positions, speeds, and newton-metres, and the driver quietly does the torque-to-current conversion inside, using Ch 2 §6's constants (Ch 2 §11 already gave you a preview of this). On the bench, the mode lives in CubeMarsTool's **MIT Control** interface (manual §3.3.2): enter the CAN ID and the desired values, then click Start.

## 2. The Position Term: $K_p$ as a Virtual Spring

The first part of the equation is $\tau_P = K_p\,(P_d - P)$. Read it like this: measure how far the shaft is from where you want it (that is the position error, $e_P = P_d - P$), multiply that error by $K_p$, and push in the direction that makes the error smaller.

One small worked example is enough to carry the whole idea. Suppose $P_d = 1.0$ rad, the shaft is actually at $P = 0.8$ rad, and $K_p = 10$. The error is $0.2$ rad, so the position term contributes $\tau_P = 10 \times 0.2 = 2$ N·m of torque pulling the shaft toward the target. If you double $K_p$, the same displacement produces double the torque.

"Torque proportional to displacement" has a familiar name in mechanics: it is exactly how a **spring** behaves. So imagine the desired position and the actual shaft connected by an invisible torsion spring. When $P = P_d$, the spring is relaxed and $\tau_P = 0$ — the motor does nothing. Push the shaft away, and a restoring torque appears, proportional to how far you pushed, with $K_p$ playing the role of the spring constant. Since the error is in rad and the torque is in N·m, $K_p$ effectively has units of **N·m per rad** — it travels through the CAN packet as a plain number, but physically that is what it means (Ch 3 §6). A large $K_p$ is a stiff spring. A small $K_p$ is a soft spring. Demo 3 will let you feel this with your own hand: the *same* motor, holding the *same* target, feels like a rubber band at $K_p = 5$ and like a rigid wall at $K_p = 100$.

## 3. The Velocity Term: $K_d$ as a Virtual Damper

The second part is $\tau_D = K_d\,(V_d - V)$. It is built exactly like the first part, just one level up: instead of position error, it uses **velocity error**. Measure how far the actual speed is from the desired speed, multiply by $K_d$, and push in the direction that closes the gap.

Again, one example: $V_d = 5$ rad/s, the shaft is actually turning at $V = 3$ rad/s, and $K_d = 1.5$. The velocity error is 2 rad/s, so this term contributes $1.5 \times 2 = 3$ N·m of torque that speeds the shaft up. If the shaft were instead running *faster* than desired, the error would be negative, and the term would brake. $K_d$ effectively carries units of **N·m per (rad/s)** (Ch 3 §6).

Now, where does the name "damping" come from? It appears when you set $V_d = 0$, which MIT Position control almost always does (§7). Then the term becomes $\tau_D = -K_d V$: a torque that always **opposes whatever motion exists**, and opposes it harder the faster the shaft moves. That is exactly how a **damper** behaves — think of a door closer, or a shock absorber on a bicycle. It does not care where the door is; it only resists how fast the door is moving. So $K_p$ builds a virtual spring, and $K_d$ builds a virtual damper next to it:

```text
Desired Pd ────/\/\/\/\──── output shaft ────[ damper Kd ]──── (resists motion)
                spring Kp
```

Why a *position* controller needs a *damper* at all — and why $V_d = 0$ does **not** switch this term off — is an important story. We will tell it properly in §7.

## 4. Feedforward Torque $\tau_{\mathrm{ff}}$

The third part is different from the other two: it does not depend on any error at all. Suppose the shaft is exactly on target and not moving ($P = P_d$, $V = V_d = 0$). Both feedback terms are zero. But if $\tau_{\mathrm{ff}} = 3$ N·m, then $\tau_{\mathrm{cmd}} = 0 + 0 + 3 = 3$ N·m. The motor pushes anyway.

And that is exactly the point. The two feedback terms must *wait for an error to appear* before they can produce any torque. The feedforward term produces torque because **you already know, in advance, that torque will be needed**. That is what "feedforward" means: you feed the answer forward, before the error even happens.

The classic use is gravity. Imagine a joint holding a lever whose weight creates about 1 N·m of torque around the axis. If you give the controller $\tau_{\mathrm{ff}} = 1$ N·m, it supplies the counter-torque directly. Without it, the $K_p$ spring has to supply that torque instead — and a spring can only make torque by staying stretched, which means the joint sits slightly off its target the whole time (§7 puts a number on this). Finally, notice one special case: if you set $K_p = K_d = 0$, then $\tau_{\mathrm{ff}}$ is the *only* term left. That is MIT Torque control, which we will meet in §9.

## 5. All Three Terms Together

The three terms are not alternatives — they are added together, every control cycle. Here is one combined example, the heart of the chapter. Suppose a command carries $P_d = 1.0$ rad, $V_d = 2$ rad/s, $K_p = 10$, $K_d = 2$, $\tau_{\mathrm{ff}} = 1$ N·m, and at this moment the shaft is at $P = 0.8$ rad, moving at $V = 1$ rad/s. Then:

| Term | Calculation | Contribution |
|---|---|---:|
| position | $10 \times (1.0 - 0.8)$ | $2$ N·m |
| velocity | $2 \times (2 - 1)$ | $2$ N·m |
| feedforward | — | $1$ N·m |
| **total** $\tau_{\mathrm{cmd}}$ | | $\mathbf{5}$ **N·m** |

Five newton-metres. Internally, the FOC layer turns this into a current request of about $5 / 0.5701 \approx 8.8$ A of $I_q$ (Ch 2 §6) — and the whole calculation repeats continuously as $P$ and $V$ change.

So here is the big realization: MIT control is not fundamentally position control, *or* velocity control, *or* torque control. It is a **torque-producing control law**, and all of those behaviors — and mixtures of them — are just different settings of the same law.

## 6. Three "Modes," One Equation: Just Parameter Patterns

CubeMars documents three MIT modes, but they are not three separate protocols. All three use Control ID 8, and each one is simply a pattern of which terms you keep and which terms you switch off:

| Pattern | $P_d$ | $V_d$ | $K_p$ | $K_d$ | $\tau_{\mathrm{ff}}$ | The equation becomes |
|---|---|---|---|---|---|---|
| **MIT Position** | target | 0 (usually) | $> 0$ | $> 0$ | 0 (usually) | $\tau = K_p(P_d - P) - K_d V$ |
| **MIT Velocity** | not used | target | 0 | $> 0$ | 0 (usually) | $\tau = K_d(V_d - V)$ |
| **MIT Torque** | not used | not used | 0 | 0 | desired torque | $\tau = \tau_{\mathrm{ff}}$ |

The trick to read this table: **setting a gain to zero removes its whole term from the equation.** Whatever terms survive decide the behavior. The GUI's three MIT panels are just convenient forms that fill in this table's rows for you.

The next three sections walk through the three patterns one at a time. A small heads-up: on the bench, you should try them in the *reverse* order — torque first (only one term alive), then velocity (two terms), then position (the full law). The Demos section follows exactly this learning path.

## 7. MIT Position Control

Command a target $P_d$ with both gains alive and $V_d = 0$, and the law becomes $\tau_{\mathrm{cmd}} = K_p(P_d - P) - K_d V$. This is a **PD position controller written directly in torque** — precisely the fixed-gain structure that Ch 9 §4 discovered drawn inside Servo Position Mode, except that now $K_p$ and $K_d$ are fields in *your* command.

**Why $V_d = 0$ — and why that does not switch off the $K_d$ term.** This is the most common point of confusion when people meet this mode for the first time. The thought goes: *"If my desired velocity is zero, then surely the velocity term is always zero?"* No — and here is why. The term reads $K_d(0 - V) = -K_d V$, and $V$ is the **measured** velocity. While the shaft is traveling toward the target, $V$ is definitely not zero, so the term is definitely not zero either.

An everyday picture makes this feel natural. Imagine you are driving toward a red traffic light. The traffic light is your desired position. Your desired speed *at the light* is 0 km/h. But when you are still 20 metres away, you might be doing 40 km/h. Your *target* speed is zero — your *actual* speed clearly is not. So what do you do? You press the brake, gradually, so that you arrive at the light with no speed left. That gradual braking is exactly what the $-K_d V$ term does for the motor.

Let us watch the two terms trade places during one real move: $P_d = 1.57$ rad (a quarter turn) starting from rest, with $K_p = 50$ and $K_d = 2$:

| Stage | $P$ (rad) | $V$ (rad/s) | Spring $K_p(P_d - P)$ | Damper $-K_d V$ | Who is winning |
|---|---:|---:|---:|---:|---|
| start | 0 | 0 | $+78.5$ N·m → clipped by the torque limit | 0 | spring alone — launch! |
| middle of the move | 0.60 | 3 | $+48.5$ N·m (before clipping) | $-6$ N·m | spring pulls, damper leans back |
| near the target | 1.55 | 2 | $+1$ N·m | $-4$ N·m | **damper** — braking *before* arrival |
| at the target, still moving | 1.57 | 1 | 0 | $-2$ N·m | damper alone — pure brake |
| settled | 1.57 | 0 | 0 | 0 | nobody — torque is zero, journey over |

(A side note on those huge spring numbers at launch: the actuator cannot and will not produce 78 N·m. The torque-limit block in the p. 38 diagram clips $\tau_{\mathrm{cmd}}$ before it reaches FOC. This is one more reason why a large-$K_p$ step is *aggressive* but not *impossible*.) Read the table top to bottom: early in the move, the position error is big, so the spring dominates and the shaft accelerates. Near the target, the error is small but the speed is high, so the damper takes over and removes the kinetic energy *before* the target instead of after it. At the target with some speed still left, the damper is the only term working, and it acts as a pure brake until the shaft rests. So $V_d = 0$ really means **"arrive at rest"**, and $K_d$ is the mechanism that makes it come true. This is also why $K_d$ is called the *damping* gain and not the velocity gain: with $V_d = 0$, it never demands that the shaft freeze instantly — it simply taxes motion, continuously, until the motion is gone.

**Why $K_d$ is not optional.** Now imagine $K_d = 0$. The law becomes a pure spring, $\tau = K_p(P_d - P)$, driving a rotor that has inertia — which is exactly a mass on a spring with no damper. What does such a system do? The shaft accelerates into the target, cannot stop in time, **overshoots**, gets pulled back by the spring, overshoots the other way, and keeps **oscillating** back and forth. The oscillation dies out only as fast as friction slowly eats the energy. Add even a modest $K_d$, and the damper removes kinetic energy continuously, so the very same step lands cleanly with no overshoot. Demo 3's Part B films both cases side by side (Videos 11.3 and 11.4) — the signature pair of clips of this chapter.

**Holding: a spring, not a lock.** Once the shaft has settled at $P = P_d$ with $V = 0$, every term is zero. So an *unloaded* MIT hold commands no torque at all — the motor just sits there, ready. Now push the shaft by hand: an error appears, and the spring pushes back. Displace a $K_p = 100$ hold by 0.05 rad, and the restoring torque is $100 \times 0.05 = 5$ N·m. But please carry three permanent truths from the earlier chapters with you, in short form. First, this is an *active* hold, not a mechanical lock — if the power goes off, the hold goes off, and if a load is stronger than the actuator's torque limit, the shaft will be pushed away no matter what (Ch 7 §6, Ch 9 §5). Second, a *loaded* hold means continuous current, and continuous current means continuous heat, even at zero speed — the series' cornerstone "zero speed ≠ zero current ≠ zero heat" lives in Ch 7 §5 and applies here in full force. Third, one property is special to this mode: if the hold is fighting a constant load $\tau_L$ using $K_p$ alone, it does not settle *at* the target but slightly *beside* it, displaced by a steady error of $e_P = \tau_L / K_p$. Why? Because a spring can only make torque by staying stretched. With $K_p = 20$ against a 1 N·m load, the sag is $0.05$ rad ≈ 2.9° — small, but clearly visible. You can shrink the sag by raising $K_p$ (a stiffer spring stretches less), or you can remove it completely by supplying the load torque as $\tau_{\mathrm{ff}}$ (§4).

## 8. MIT Velocity Control

Set $K_p = 0$, command a $V_d$, and keep $K_d > 0$. The law collapses to $\tau_{\mathrm{cmd}} = K_d(V_d - V)$. Below the target speed, the torque is positive and the shaft accelerates. Above it, the torque is negative and the shaft brakes. The shaft settles at whatever speed makes the torque balance the load. For example, with $V_d = 6$ rad/s and $K_d = 2$: at $V = 4$ the motor drives with $2 \times 2 = 4$ N·m; if something spins it up to $V = 7$, it brakes with $-2$ N·m.

Here, $K_d$'s job is to set **how aggressively** speed errors are corrected. It is the exchange rate from velocity error to torque — and you choose it, per command. That is the whole difference from Servo Velocity Mode (Ch 8): there, a PI controller with its gains lives *inside* the firmware, and you only send the target. It is flexibility versus simplicity, exactly as Ch 8 §8 described it.

One honest limitation is worth knowing. A purely proportional law behaves like the loaded position hold from §7: under a constant load, the shaft settles with a small standing speed error of $\tau_L / K_d$, running slightly *below* the commanded speed — because the term must stay nonzero to keep producing torque against the load. Servo Velocity's integral action removes such offsets automatically; MIT Velocity trades that away in exchange for per-command control. (Or you can cancel a known load yourself with $\tau_{\mathrm{ff}}$.)

## 9. MIT Torque Control

Set both gains to zero, and the equation becomes just one term long: $\tau_{\mathrm{cmd}} = \tau_{\mathrm{ff}}$. The feedforward field *is* the command now — a **signed torque in newton-metres**, and this is the only place in the whole actuator where you can send N·m over the wire (Ch 6 §8). Positive and negative values push in opposite directions. That makes this the signed cousin of Brake Mode, whose command was an unsigned magnitude with the firmware choosing the direction (Ch 7 §3).

Two things this mode is **not**, and both matter. First, it is not speed control. With no feedback term alive, nothing regulates the velocity at all. The commanded torque simply accelerates whatever inertia it meets — so a small $\tau_{\mathrm{ff}}$ on an unloaded shaft will happily wind up to a high speed, exactly like a small $I_q$ did in Ch 6. The full story — net torque, acceleration, and what finally limits the speed — was told once for the whole series in Ch 6 §4, and it transfers here word for word, with $\tau_{\mathrm{ff}}$ taking the place of $0.5701\,I_q$. Second, it is not current-free. The newton-metres you command still become $I_q$ inside the driver, through the same FOC machinery as every other mode (Ch 2 §6, §11). You have changed the *unit of the conversation*, not the physics. And that unit is exactly the practical difference from Ch 6's Current Loop: there you command amps and do the torque conversion yourself; here you command torque and the driver owns the conversion.

## 10. Parameter Ranges for the AK80-9 — and Why a Range Is Not a Recommendation

The manual's Parameter Ranges table (§4.2, p. 39) gives the MIT command ranges for each motor model. For the **AK80-9 (KV100)**:

| Parameter | Range |
|---|---:|
| Position $P_d$ | $-12.56$ … $+12.56$ rad |
| Velocity $V_d$ | $-65$ … $+65$ rad/s |
| Torque $\tau_{\mathrm{ff}}$ | $-18$ … $+18$ N·m |
| $K_p$ | $0$ … $500$ |
| $K_d$ | $0$ … $5$ |

The position range deserves a few lines of simple arithmetic. One revolution is $2\pi \approx 6.283$ rad, and $12.56 \approx 2 \times 2\pi$. So the MIT position range covers about **±2 revolutions** — a deliberately joint-sized window, tiny compared to the Servo multi-turn range of ±100 turns (Ch 10 §2, Ch 3 §1). A Servo target of 720° sits right *at* the MIT range limit.

And now the standing warning, which is this mode's version of the rule from Ch 2 §9: **these are command-encoding ranges, not recommendations.** $K_p \le 500$ does *not* mean you should start at 500 — Demo 3 uses 5 and 100, and 100 already feels rigid. $\pm 18$ N·m is what the 12-bit torque field can *express*; the motor's actual ratings remain 9 N·m continuous / 22 N·m peak. And ±65 rad/s converts to about ±621 rpm — *above* the motor's 570 rpm no-load speed (Ch 3 §3) — so the message can literally ask for speeds the motor cannot deliver. Please burn this into memory: **a value being encodable never means it is operable.**

## 11. Units

MIT mode speaks the robotics dialect: position in **rad**, velocity in **rad/s**, torque in **N·m at the output shaft**. No degrees, no ERPM, no amps. Every conversion you will need — rad ↔ degrees, rad/s ↔ rpm ↔ ERPM, and the handy "1 N·m ≈ 1 kg hanging 10 cm from the axis" intuition — lives in Ch 3, especially §1, §3, and §5. The effective units of the gains, N·m/rad and N·m/(rad/s), were explained in §2–§3 above. One good habit to build right now: before pressing Start, convert your command in your head. An MIT velocity of 3 rad/s is about 28.6 rpm (Ch 3 §3) — knowing that in advance means the motion you *see* will match the motion you *expected*.

## 12. The Virtual Spring–Damper Actuator

Let us now assemble §§2–4 into the mental picture this mode truly deserves. MIT control turns the AK80-9 into a **programmable spring–damper actuator with an extra torque source**:

```text
                    PROGRAMMABLE JOINT
        Pd ────/\/\/\/\──┐
                spring Kp │
                          ├──►(+)──► τcmd ──► FOC ──► shaft
        Vd ──[ damper Kd ]┘   ▲
                              │
                        τff (direct torque)
```

Change the five fields, and the *mechanical personality* of the joint changes: a stiff servo, a soft compliant joint, a heavily damped landing, a free-spinning torque source — same hardware, a different character in every frame. This is the property the Servo family cannot offer at any setting, and it is why this interface carries the name of the MIT Cheetah robot family that made it famous.

And the three patterns of §6 are not the end of the story. Nothing forbids using *all five fields at once*. Picture a robot computer streaming a time-varying trajectory: at every instant it sends the desired position $P_d(t)$, the desired velocity $V_d(t)$, and a feedforward torque $\tau_{\mathrm{ff}}(t)$ that its physics model predicts will be needed (gravity, acceleration loads) — while $K_p$ and $K_d$ quietly correct whatever small errors remain around that moving reference. Sent at a high rate over CAN, this is trajectory tracking with feedforward — the daily bread of walking robots. And if you choose the gains not for accuracy but for the *feel of the interaction*, you arrive at **impedance control**, where the joint's apparent stiffness and damping are themselves the things being controlled. This series stops at that doorway — but now you know the doorway exists.

## 13. The MIT CAN Protocol: Packing, the Manual's Example Code, and Its Bugs

This section is for the moment you leave the GUI behind and start sending MIT frames from your own microcontroller. It also contains an important warning about the manual's printed example code.

**The frame.** MIT commands use an extended CAN frame: the identifier carries Control Mode ID 8 in bits [28:8] and the driver ID in bits [7:0], with 8 data bytes (manual p. 38). For a motor with CAN ID 0x68, every MIT command therefore carries the extended identifier `0x00000868` — the same construction as the Servo frames of Chs 5–10, just with 8 as the mode number. The five values are bit-packed tightly: position gets 16 bits, and each of the other four gets 12 bits, which adds up to exactly 64:

| Byte | Content |
|---|---|
| DATA[0] | $K_p$ high 8 bits |
| DATA[1] | $K_p$ low 4 bits ∣ $K_d$ high 4 bits |
| DATA[2] | $K_d$ low 8 bits |
| DATA[3] | Position high 8 bits |
| DATA[4] | Position low 8 bits |
| DATA[5] | Velocity high 8 bits |
| DATA[6] | Velocity low 4 bits ∣ Torque high 4 bits |
| DATA[7] | Torque low 8 bits |

(One misprint to be aware of: the manual's p. 38 table labels DATA[6]'s upper half as "High 8 bits of Speed value." The packing code on p. 41 — `buffer[6] = ((v_int&0xF)<<4)|(t_int>>8)` — shows it is really the *low 4 bits* of velocity, as written in the table above.)

**From floats to field integers.** Each physical value is converted into its unsigned field by the manual's `float_to_uint` function (p. 41). The recipe is simple: clamp $x$ into its range $[x_{\min}, x_{\max}]$, then stretch $(x - x_{\min})$ across the $2^{\mathrm{bits}}$ counts of the field. Notice a slightly surprising consequence: zero lands in the *middle* of the range — so an all-zero *physical* command is **not** an all-zero *payload*. Two more practical consequences. Resolution: 16 bits across ±12.56 rad gives about 0.0004 rad per count; 12 bits across ±18 N·m gives about 0.0088 N·m per count. And **range-dependence**: the packed bytes mean nothing by themselves — the receiver decodes them using the same min/max constants the sender encoded with. So always use *your* motor's row from the p. 39 table, on both ends.

The manual's own worked examples (§4.4.1, p. 52–53) show that last point very sharply. The MIT Position example decodes correctly against the universal ±12.56 rad span (`BD 70` → 6 rad; $K_p$ = 2 and $K_d$ = 2 also check out against 0–500 and 0–5). But the MIT Velocity example — "Kd set to 2, speed set to 6 rad/s" → `00 06 66 7F FF 8F 57 FF` — packs the velocity as `0x8F5`, and that decodes to 6 rad/s only if the velocity range were **±50 rad/s** — a range that matches *no column* of the p. 39 table. (Decoded with the AK80-9's ±65 range, it would read about 7.8 rad/s.) The lesson: treat the printed payloads as illustrations of the packing scheme, never as byte-strings to copy and replay on your own motor.

**The `pack_cmd` clamping bug.** The manual's send-code example (p. 40–41) contains a genuine mistake that you must not copy. Its first two clamping lines read:

```c
p_des = fminf(fmaxf(P_MIN, 0), P_MAX);
v_des = fminf(fmaxf(V_MIN, 0), V_MAX);
```

Look closely: both lines clamp the **literal number 0** instead of the value that was passed in. Compare with the very next line, which does it correctly with the variable: `kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);`. As printed, `pack_cmd` therefore throws away whatever position and velocity you give it and always transmits $P_d = 0$ and $V_d = 0$. The intended, correct form is:

```c
p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
```

(There is even evidence inside the manual itself that this is a printing mistake and not intended behavior: the p. 53 position example carries a correctly packed, nonzero $P_d$ = 6 rad — a payload the printed code could never produce.) So: if you implement the MIT protocol from the manual, fix those two lines first, and in general, verify example code against the actual behavior of your firmware version instead of trusting it blindly. This is one of the two manual quirks flagged for this series; the other one — the swapped serial-enum comments — lives in Ch 10 §7.

**Feedback from the motor.** The manual documents motor feedback as the timed Servo-side CAN upload of §4.3.1 (p. 42): position, speed, current, temperature, and error code, at a configurable rate of 1–500 Hz. It does *not* print a separate MIT-specific reply format in §4.2. Some firmwares in the wider MIT-protocol family send a specially packed reply — if your application needs that, test for its presence and format on your own firmware version before relying on it.

## 14. MIT vs Servo: Two Ways of Thinking

Here is the whole philosophical difference in a few plain sentences. The Servo family works by **delegation**: you name one target — a current, a speed, an angle — and the firmware's internal controllers decide how strongly to chase it. MIT control works by **specification**: every frame states not only *where* and *how fast*, but also **how stiff, how damped, and with how much extra torque** — the response itself becomes part of the command. Servo modes are "set and forget," perfect when one well-regulated quantity is the entire job. MIT mode expects a smarter master that keeps streaming frames — and in return, it gives you per-command control over the joint's mechanical character. Which philosophy fits which application — with every head-to-head mode comparison — is the entire business of Ch 12.

> **The final mental model, in one box.** MIT mode is a torque-producing control law, $\tau_{\mathrm{cmd}} = K_p(P_d - P) + K_d(V_d - V) + \tau_{\mathrm{ff}}$, running on top of the same FOC engine as every other mode. $K_p$ is a spring you specify; $K_d$ is a damper you specify; $\tau_{\mathrm{ff}}$ is a torque you inject directly. Keep all three terms: a compliant, trajectory-tracking joint. Keep two: a position servo with stiffness and damping of your choosing. Keep one: a velocity regulator, or a pure torque source. One equation, one Control ID — every behavior in this series, *configured* rather than *selected*.

## Demos

All three demos follow Ch 4 §6's ground rules: motor bench-mounted and calibrated per Ch 4, output shaft **unloaded**, 48 V supply with a conservative current limit, CubeMarsTool connected with the Real-time Data panel open and the Stop button located before you begin. Commands are entered in the **MIT Control** interface (manual §3.3.2) with the motor's CAN ID set. Fields you are not using in a given demo must be typed as **0**, on purpose — remember §6: in this mode, a zeroed gain removes its whole term from the equation, and every demo below is built on exactly that trick. Fit a tape flag or a light lever to the shaft so the motion is visible on camera. All angles and speeds are output-shaft values in rad and rad/s (§11).

The order of the demos is the learning path from §6: torque first (only one live term), then velocity (two terms), then position (the full PD law) — each experiment isolates one piece of the master equation before the next one adds to it. One mode-specific caution: an MIT command takes effect the moment you press Start, gains included. So **always type and double-check $K_p$ and $K_d$ before touching the position field**, and always start from the small values printed here — never from §10's range limits.

### Demo 1 — Pure Torque Control

The equation cut down to a single term: $K_p = 0$, $K_d = 0$, so $\tau_{\mathrm{cmd}} = \tau_{\mathrm{ff}}$ (§9).

**Starting condition:** calibrated per Ch 4, unloaded, shaft flagged, motor firmly bolted down, Real-time Data showing speed and current.

**Commands and expected observations:**

1. Set **$K_p = 0$, $K_d = 0$, $V_d = 0$**, and start small: **$\tau_{\mathrm{ff}} = 1$ N·m**. The unloaded shaft accelerates smoothly away — torque causes acceleration, exactly as in Ch 6 §4, only now the command is written in newton-metres. Return the command to 0 and let the shaft coast down.
2. The recorded run (Figure 11.1, Video 11.1) uses **$\tau_{\mathrm{ff}} = 14$ N·m** — a much stronger push, above the 9 N·m continuous rating and internally about $14 / 0.5701 \approx 24.6$ A of $I_q$ (Ch 2 §6). Treat it as a short burst only. If you repeat it: shaft unloaded, motor firmly bolted, hands and cables completely clear, and bring the command back to 0 within a second or two.
3. Repeat step 2 with **$\tau_{\mathrm{ff}} = -14$ N·m**: the same strength, opposite direction. The command is signed — and this field is the actuator's only signed N·m interface (§9).

| Quantity | Commanded? | What you observe |
|---|:---:|---|
| Torque | **yes** — ±14 N·m via $\tau_{\mathrm{ff}}$ | steady push, whether spinning or stalled |
| Speed | no | climbs freely when unloaded; 0 when stalled — never regulated |
| Position | no | wherever the motion leaves it |
| Current | no (internal) | the FOC price of 14 N·m (Ch 2 §6) |

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image13.png" 
          alt="MIT Mode Torque Control : Position = 0, Velocity = 0, Feedforward Torque = 14">
      <figcaption>
        Figure 11.1: MIT Mode Torque Control : Position = 0, Velocity = 0, Feedforward Torque = 14. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video11.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 11.1: MIT Mode Torque Control : Position = 0, Velocity = 0, Feedforward Torque = 14</i>
      </p>
</figure>
</div>

### Demo 2 — Pure Velocity Control

Two live terms now: $K_p = 0$, so $\tau_{\mathrm{cmd}} = K_d(V_d - V)$ (§8).

**Starting condition:** as Demo 1, shaft at rest.

**Commands and expected observations:**

1. Set **$K_p = 0$, $\tau_{\mathrm{ff}} = 0$, $V_d = 3$ rad/s, $K_d = 1$**. Start. The shaft spins up to a steady ≈ 3 rad/s ≈ 28.6 rpm (Ch 3 §3) — a calm, easy-to-watch rotation. This is exactly the run in Figure 11.2 and Video 11.2.
2. Now disturb it: drag the flag gently with your fingers to slow the shaft down. Feel how the torque against your fingers *grows* the more you slow it — drag it 2 rad/s below the target, and the term supplies $1 \times 2 = 2$ N·m of recovery torque. Let go, and the shaft returns to 3 rad/s. Error in, torque out, continuously.
3. Repeat step 2 after restarting with **$K_d = 2$**: the same drag is now resisted twice as hard, and the recovery is quicker. $K_d$ is the aggressiveness dial of this pattern — and it is *your* dial, in every command, whereas Ch 8's Servo loop kept its gains locked inside the firmware.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image14.png" 
          alt="MIT Mode Velocity Control : Position = 0, Velocity = 3, Feedforward Torque = 0, Kp = 0, Kd = 1">
      <figcaption>
        Figure 11.2: MIT Mode Velocity Control : Position = 0, Velocity = 3, Feedforward Torque = 0, Kp = 0, Kd = 1. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video12.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 11.2: MIT Mode Velocity Control : Position = 0, Velocity = 3, Feedforward Torque = 0, Kp = 0, Kd = 1</i>
      </p>
</figure>
</div>

### Demo 3 — Position Control: Feeling $K_p$ and $K_d$

The full PD law at last: $\tau_{\mathrm{cmd}} = K_p(P_d - P) - K_d V$ (§7). This demo has two parts — first you *feel* what $K_p$ does, then you *watch* what $K_d$ does.

**Starting condition:** as Demo 1, unloaded, shaft settled at $P = 0$ (zero the position, or account for the offset), Real-time Data plotting position.

**Part A — $K_p$, felt by hand:**

1. Set **$K_p = 5$, $K_d = 1$, $P_d = 0$, $V_d = 0$, $\tau_{\mathrm{ff}} = 0$**. Start. Nothing visibly happens — on target, every term is zero (§7). Now wind the shaft away from the target by hand. It gives way like a soft torsion spring — you can turn it a large fraction of a radian with two fingers (at 0.2 rad, the restoring torque is only $5 \times 0.2 = 1$ N·m) — and it pulls back gently to zero when you release it.
2. Stop, set **$K_p = 100$**, Start, and try again. The same shaft now feels rigid: the same 0.2 rad displacement would demand 20 N·m, so your hand runs out of strength within a few degrees. Same target, same hardware — twenty times the stiffness, because you typed a different number.

One honest note: the sensation in Part A — the graded spring resistance growing under your palm, soft versus rigid — **cannot be filmed**. It must be felt at the bench, with your own hand. That is why the videos below belong to Part B.

**Part B — $K_d$, watched on camera and on the plot:**

3. Set **$K_p = 5$, $K_d = 0$, $V_d = 0$, $\tau_{\mathrm{ff}} = 0$**, and command a step to **$P_d = 1.57$ rad** (a quarter turn). The shaft snaps toward the target, flies past it, gets pulled back, passes it again — **visible overshoot and ringing** around 1.57 rad, dying out only as friction slowly drains the energy. A pure spring driving an inertia, with no damper (§7). Keep well clear of the shaft even at this soft $K_p = 5$ — and never try an undamped step at high gains, where the same launch becomes genuinely aggressive.
4. Stop, return to $P_d = 0$, let it settle. Now set **$K_d = 1$** with the same $K_p = 5$ and command the same step to **1.57 rad**: one clean, confident move that brakes *into* the target and lands without a single overshoot — the $-K_d V$ term removing the kinetic energy before arrival (§7).
5. Look at the Real-time position plot across both takes: a decaying sine wave ringing around 1.57 in the first; a crisp, well-damped landing in the second. That pair of traces is a textbook PD-tuning lesson — produced by changing one number.

> **Note — what the two gains do, in one breath.** As you increase $K_p$, the spring behavior of the shaft increases: it holds its target more stiffly, resists your hand more strongly, and sags less under any load (the sag is $\tau_L / K_p$, §7). $K_d$ acts on the velocity: it damps the motion, and it decides how cleanly and accurately the shaft lands on its final position — too little $K_d$ and the shaft overshoots and oscillates around the target (step 3); enough $K_d$ and it settles onto the target precisely, in one smooth motion (step 4).

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video13.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 11.3: MIT Mode Position Control : Position = 1.57, Velocity = 0, Feedforward Torque = 0, Kp = 5, Kd = 0</i>
      </p>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video14.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 11.4: MIT Mode Position Control : Position = 1.57, Velocity = 0, Feedforward Torque = 0, Kp = 5, Kd = 1</i>
      </p>
</figure>
</div>


## When to Use MIT Mode — and When Not

Use it when the joint's *response* is part of what you need to specify: compliant robot joints and legged locomotion; arms and mechanisms that touch people, parts, or an unpredictable world; gravity- or dynamics-compensated poses via $\tau_{\mathrm{ff}}$ (§4, §7); high-rate trajectory streaming with feedback cleanup (§12); direct torque commands in N·m (§9); and any application where the stiffness and damping must change from move to move rather than being fixed in firmware.

Skip it when delegation is simply the better deal. If one well-regulated quantity is the whole job — a conveyor's speed, an indexing table's profiled moves, a set-and-forget hold — the Servo modes do it with one field and no external master in the loop (Chs 5–10), and Servo Velocity's integral action holds speed under load without §8's standing error. Remember also that MIT position covers only ±2 revolutions (§10) — long accumulating travels belong to the Servo multi-turn coordinate (Ch 10). And be honest about the risk: five coupled fields mean five ways to command something aggressive by a typing mistake; this is the least forgiving interface in the series (see Safety Notes). The systematic decision framework, with every MIT-vs-Servo head-to-head comparison, is Ch 12.

## Where These Ideas Go Next

| Established here | Continues in |
|---|---|
| The master equation and the three patterns (§1, §6) | Ch 12's master table treats the three MIT patterns as first-class modes |
| Per-command $K_p$/$K_d$ vs internal Servo gains (§7–§9, §14) | Ch 12 §4 Servo-vs-MIT philosophy; head-to-head boxes for Position, Velocity, Current/Torque |
| Steady-state sag $\tau_L/K_p$ and its feedforward cure (§4, §7) | Ch 12 scenario (a): a robot-arm joint with gravity |
| MIT Torque = signed N·m cousin of Brake's unsigned magnitude (§9) | Ch 12's comparison of the current-flavored commands (with Ch 7 §3) |
| Range ≠ recommendation, MIT edition (§10) | Ch 12 §6's cross-cutting cautions recap |
| The pack_cmd clamping bug and range-dependent payloads (§13) | anyone implementing the protocol; flagged in Ch 12 §6's cautions |
| The spring–damper joint and the doorway to impedance control (§12) | beyond this series — the robotics literature it points toward |

## Key Takeaways

- MIT (Force) Control, Control Mode ID 8, packs $\{P_d, V_d, K_p, K_d, \tau_{\mathrm{ff}}\}$ into one frame and computes $\tau_{\mathrm{cmd}} = K_p(P_d - P) + K_d(V_d - V) + \tau_{\mathrm{ff}}$ — one equation feeding the same FOC engine as every other mode.
- **MIT Position / MIT Velocity / MIT Torque are parameter patterns, not separate protocols**: set a gain to zero and its term disappears; the manual's p. 38 names for them (Position/Velocity/Current Loop Mode) mean the same three things.
- $K_p$ is a virtual spring (N·m/rad); $K_d$ is a virtual damper (N·m/(rad/s)); $\tau_{\mathrm{ff}}$ is torque you give in advance, without waiting for an error.
- In MIT Position control, $V_d = 0$ means "arrive at rest," and the $-K_d V$ term is the brake that makes it true — without it, a spring plus inertia oscillates (Demo 3, Part B). Holding is an active spring, not a lock: it sags by $\tau_L/K_p$ under load (curable with $\tau_{\mathrm{ff}}$), fails with power loss, and heats while it holds (Ch 7 §5–§6).
- MIT Torque is the actuator's only N·m command and its only *signed* direct torque interface — and it controls torque, never speed (Ch 6 §4).
- AK80-9 ranges: ±12.56 rad (≈ ±2 revolutions), ±65 rad/s, ±18 N·m, $K_p$ 0–500, $K_d$ 0–5 (manual p. 39) — **these are encoding ranges, not operating advice**; the real ratings remain 9/22 N·m and 570 rpm.
- The manual's `pack_cmd` example clamps the literal 0 instead of `p_des`/`v_des` — fix those two lines before use — and its example payloads were packed with module-specific (in one case, unidentifiable) ranges: never replay them byte-for-byte (§13).

## Safety Notes

- **Check the gain fields before the position field, every single time.** A large $K_p$ with a distant $P_d$ is a command to sprint; an undamped one is a command to oscillate (Demo 3, Part B). Start from this chapter's demo values, never from §10's limits, and increase gains in small steps.
- **A zeroed $K_d$ removes the brakes.** Never command position steps with $K_d = 0$ except deliberately, unloaded, and well clear of the shaft, as in Demo 3 Part B.
- $\tau_{\mathrm{ff}}$ acts immediately and unconditionally — it is live torque, not a setting. Confirm its sign and size before pressing Start, especially with anything attached to the shaft.
- Every loaded MIT hold is a heating condition (Ch 7 §5): watch the temperature field, keep hold durations short, and never leave a powered hold suspending a load unattended — power off means hold off (§7).
- Stall or disturb the shaft only by hand, through a cloth or a lever, and only at the small torques used in these demos (Ch 4 §6). At high gains, the restoring torque against your hand can reach the actuator's full strength.
- The ±18 N·m, ±65 rad/s, and $K_p = 500$ command limits go beyond what the motor can safely or physically sustain (§10) — encodable is not operable.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §4.2 "Force Control Mode (MIT) Communication Protocol" (p. 37–41: single Control Mode ID 8 shared by the three MIT patterns; extended-frame ID layout; the Data Transmission Definition table, p. 38, including the DATA[6] label misprint noted in §13; the simplified Force Control diagram, p. 38, showing the $K_p$/$K_d$/$T_{\mathrm{ref}}$ sum feeding `iq_ref` through torque-limit protection with `id_ref = 0`; the p. 38 "Position/Velocity/Current Loop Mode" naming recorded in this chapter's terminology note; the Parameter Ranges table, p. 39, AK80-9 column: ±12.56 rad, ±65 rad/s, ±18 N·m, $K_p$ 0–500, $K_d$ 0–5 — verified against the printed table image; the Force Control Mode Send Code Example, p. 39–41: `SERVO_Can_Send_Msg`, `pack_cmd` — including the clamping error documented in §13 — and `float_to_uint`); §3.3.2 "Force Control Mode (Mit Control)" (p. 28–29: GUI operation of Force Control Position / Velocity / Torque Mode); §4.3.1 "CAN Upload Message Protocol" (p. 42: the timed telemetry upload cited for §13's feedback note); §4.4.1 "CAN Port Control Command Examples" (p. 52–53: the MIT Velocity / Position / Torque example payloads analyzed in §13).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — rated/peak torque 9 / 22 N·m, rated/peak current 12 / 28 A, no-load speed 570 rpm, and the torque constants 0.095 N·m/A (motor side) / 0.5701 N·m/A (output side) used for this chapter's current estimates, via the series reference table (Ch 1 §2) and their definitive treatment in Ch 2 §6 and §9.