# Chapter 6 — Current Loop Mode: Controlling Torque

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 7 — Current Brake Mode →](07_Current_Brake_Mode.md)

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In Ch 5, you commanded a raw voltage level, and you saw what the motor does when nothing is regulated. In this chapter, the controller starts working *for* you. **Current Loop Mode** (Servo Control Mode ID 1) is the innermost control loop of the AK80-9 — the first rung of the cascade diagram you saw in Ch 5's introduction — and it is the first mode where the controller actively holds a value that you asked for.

The value you command is $I_q$: the motor's torque-producing current. And here is the key point. Since the output-shaft torque is approximately $0.5701 \times I_q$ (Ch 2 §6), commanding current is really the same as commanding **torque** — the turning force of the motor. So, for all practical purposes, this is the Servo family's *torque mode*.

This mode also surprises almost every new user, and the reason is simple: **the command is in amps, and amps *feel* like they should mean speed.** They do not. Command $I_q^* = 1$ A on an unloaded motor, and the shaft does not turn slowly. It speeds up and up, reaching a couple of hundred rpm — all from a turning force of barely half a newton-metre. Understanding *why* this happens is the main job of this chapter. The mental model you will build here — *current commands torque, and torque causes acceleration, not a particular speed* — is used again and again in every later chapter.

Here is the plan, step by step:

- what Current Loop Mode is, and why commanding $I_q$ means commanding torque (§1);
- how to convert a desired torque into an $I_q$ command, with a worked 2 N·m example (§2);
- positive and negative commands, and why torque direction is not the same as rotation direction (§3);
- the chapter's core mental model, told once for the whole series: what a constant current actually makes the motor *do* — with no load, with a load, and everything in between (§4);
- what happens when the shaft is blocked and cannot turn (§5);
- two short reminders with pointers: the commanded $I_q$ is not the supply current (§6), and the ±60 A protocol range is not a rating (§7);
- the mystery of the GUI's `T` control: where the Servo "torque mode" really lives at the protocol level (§8);
- and three short pointers to the neighboring modes (§9);

followed by two bench demos, advice on when (and when not) to use this mode, safety notes, and sources. As always, we assume a connected, calibrated motor and Ch 4 §6's safety ground rules.

## 1. What Current Loop Mode Is

The manual defines the mode in one line (§4.1, p. 31): *"A specified Iq current is given to the motor, and since the motor output torque = iq·KT, it can be used as a torque loop."* On the GUI side, using it is one step (§3.3.1.5, p. 26): in the Servo Control interface, *"enter the desired current I, and the motor will operate at the desired current."*

Let's unpack this definition slowly, because three important facts are hiding inside it.

**Fact 1: the commanded quantity is $I_q$ — the FOC q-axis current.** It is not the DC supply current, and it is not one of the three phase currents. It is the special torque-producing component that Ch 2 §5 showed us how to extract from the phase currents using the Clarke and Park transformations. So when you type 2 into the I field, you are telling the controller: *please keep the motor's torque-producing current component at 2 A.*

**Fact 2: this is a closed loop — the first one you have met.** In Duty Cycle Mode, your command went straight to the inverter, and nothing checked the result. Here, something *does* check. The controller continuously measures the real phase currents, calculates the real $I_q$ from them, compares it with your command, and adjusts the inverter's voltage until the measured value matches the commanded value. This is Ch 2 §5's FOC loop — but now it is serving a target that *you* chose:

```text
   Iq* (your command)
        │
        ▼
  current controller ◄── measured Iq (from the phase currents, Ch 2 §5)
        │
        ▼ adjusts voltage / modulation
     inverter → windings → torque → gearbox → output shaft
```

Notice what is regulated and what is not. Everything *electrical* is now managed for you. But everything *mechanical* — speed, position, direction of travel — is still a free outcome, just like in Ch 5. This half-and-half situation is what defines the mode.

**Fact 3: a regulated current means a regulated torque.** The two constants that connect current to torque are Ch 2 §6's — we simply use them here, without repeating the full explanation:

$$
\tau_m = 0.095\,I_q \quad\text{(motor side)}, \qquad
\tau_{\mathrm{out}} \approx 0.5701\,I_q \quad\text{(output shaft)} .
$$

For everyday work, the output-side formula is the one you will use. It is also why CubeMars ends its definition with "it can be used as a torque loop": if a mode holds $I_q$ steady, it also holds the output torque steady at about $0.5701\,I_q$ newton-metres. You can think of Current Loop Mode as *a torque mode that happens to speak in amps*.

One more thing this mode does **not** contain: any speed or position goal. When you command 1 A, there is no hidden target rpm anywhere inside the controller — no loop measuring speed, no setpoint to hold. Whatever motion happens next comes purely from physics, and that is §4's story.

## 2. From Desired Torque to an $I_q$ Command

In real projects, you usually do not start from a current. You start from a torque requirement — "this joint needs about 2 N·m" — and work backwards. To do that, just turn the output-side formula around:

$$
I_q^* = \frac{\tau_{\mathrm{out}}^*}{0.5701} .
$$

**Worked example — asking for about 2 N·m at the output shaft.** The required command is $I_q^* = 2 / 0.5701 \approx 3.51$ A. Enter 3.51 in the GUI's I field (Servo Control interface, General Control area — Ch 4 §3), or send it over CAN (shown below). The actuator will then produce roughly 2 N·m at the output shaft for as long as the command stands — at any speed the mechanics settle into, in either rotation direction, moving or completely stopped.

Here are a few useful values to keep in your head, all computed with $\tau_{\mathrm{out}} \approx 0.5701\,I_q$:

| $I_q^*$ | $\tau_{\mathrm{out}}$ (approx.) | Remark |
|---:|---:|---|
| 0.5 A | 0.29 N·m | a very gentle push |
| 1 A | 0.57 N·m | a gentle push — the working value of Demos 1–2 |
| 2 A | 1.14 N·m | the braking value used in Ch 7's demo |
| 3.51 A | 2.00 N·m | the worked example above |
| 5 A | 2.85 N·m | §4's load-beating example |
| 10 A | 5.70 N·m | getting close to the 12 A continuous limit |

One short warning, explained fully in Ch 2 §6.6: this formula is an engineering *model*, not a torque sensor. Friction, temperature, and calibration all pull the real delivered torque a little away from the calculated value. So treat your results as good estimates, not exact measurements.

**Over CAN**, the command uses Control Mode ID 1 with a 4-byte payload: a signed int32 equal to **current × 1000**. That means the values −60,000…+60,000 represent −60…+60 A (manual §4.1.2, p. 33). The manual's own send routine is really just that one line of scaling:

```c
void comm_can_set_current(uint8_t controller_id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}
```

So our 3.51 A example travels through the wire as the number 3,510. The current row of Ch 3 §6's master units table records this scaling next to every other unit the motor speaks. And what that wide ±60 A range does and does not mean — we will get to that in §7.

## 3. Signed Commands: Torque Direction Is Not Rotation Direction

The $I_q$ command is **signed** — it can be positive or negative across the full ±60 A protocol range — and the sign means *torque direction*:

| Command | Requested torque | Meaning |
|---:|---:|---|
| $I_q^* = +2$ A | $\approx +1.14$ N·m | full torque, positive direction |
| $I_q^* = -2$ A | $\approx -1.14$ N·m | same strength, opposite direction |
| $I_q^* = 0$ A | $\approx 0$ N·m | no torque at all — the shaft is free to coast |

Which physical rotation direction counts as "positive"? That depends on how your motor's direction convention is configured. Check it once: give a small positive command on the free shaft, note which way it turns, and that answer holds for every mode from now on (Ch 2 §7).

Now for the part that deserves thirty seconds of careful thought: **the sign tells you the direction of the torque — not the direction the shaft is turning.** The two agree only when the torque and the motion happen to point the same way.

Imagine the shaft is already spinning in the positive direction, and you command $I_q^* = -1$ A. The motor now pushes *against* the existing motion. So the shaft does not instantly start rotating negatively — instead, it **slows down**. The negative torque acts first as a brake, bringing the speed down through zero. Only then, if the command is still there, does the same torque start speeding the shaft up in the negative direction. One command, two phases:

```text
  shaft spinning +      Iq* = −1 A      ω falls … 0 …      ω grows negative
  ───────────────►   torque points −    (braking phase)    (reversal phase)
```

This is not a rare corner case — it is how *every* direction change works in this mode, and you will watch it happen in Demo 2. It also gives you a preview of a family difference: in Current Loop Mode, *you* choose the torque direction with a sign. In Current Brake Mode, the command has no sign at all — it is just a magnitude, and the firmware chooses whichever direction opposes the motion (Ch 7 §3).

## 4. The Core Mental Model: Current Commands Torque, Torque Causes Acceleration

Here is the question the whole chapter turns on: *you command a constant current — what does the motor actually do?*

The tempting answers are: "it spins at a speed proportional to the current," or "once the current and the load are both constant, the speed becomes constant too." Both are wrong. The correct answer fits in one sentence:

> **A constant $I_q$ produces an approximately constant torque — and torque decides the *acceleration* of the shaft. It never, by itself, decides a particular speed.**

Everything in this section is that one sentence, applied to three different situations. And the single equation that runs it all is Newton's second law, written for rotation:

$$
J\alpha \;=\; \tau_{\mathrm{motor}} - \tau_{\mathrm{load}} - \tau_{\mathrm{loss}} \;=\; \tau_{\mathrm{net}} .
$$

Don't worry if this looks dense — each symbol is simple. $J$ is the inertia of the rotating system (how "heavy" it is to spin). $\alpha = d\omega/dt$ is the angular acceleration (how quickly the speed is changing). $\tau_{\mathrm{motor}} \approx 0.5701\,I_q$ is the torque you commanded. $\tau_{\mathrm{load}}$ is any external torque pushing back, and $\tau_{\mathrm{loss}}$ is friction and other internal losses. The equation says: *the shaft accelerates according to the torque that is left over after the opposition is subtracted.*

### 4.1 Net torque decides everything: three cases

The motor's torque never acts alone. Motion follows the **net** torque, and there are exactly three possibilities:

| Torque balance | $\tau_{\mathrm{net}}$ | $\alpha$ | What the speed does |
|---|:---:|:---:|---|
| $\tau_{\mathrm{motor}} > \tau_{\mathrm{load}} + \tau_{\mathrm{loss}}$ | > 0 | > 0 | rises — and keeps rising while the extra torque lasts |
| $\tau_{\mathrm{motor}} = \tau_{\mathrm{load}} + \tau_{\mathrm{loss}}$ | 0 | 0 | stays at *whatever value it currently has* |
| $\tau_{\mathrm{motor}} < \tau_{\mathrm{load}} + \tau_{\mathrm{loss}}$ | < 0 | < 0 | falls — and may pass through zero and reverse |

Please read the middle row twice, because it contains the mode's most important subtlety. When the torques balance, the speed is *constant* — but it is constant at whatever value the system happened to reach on its own. No part of the controller chose that number, and no part of the controller will defend it.

Some concrete numbers make the first and third rows feel real. Suppose a mechanism pushes back with a steady 2 N·m. Command $I_q^* = 1$ A, and the motor offers only about 0.57 N·m — the load wins, so the mechanism either refuses to move, or slows down if it was already moving. Now command $I_q^* = 5$ A, and the motor offers about 2.85 N·m — there is now roughly 0.85 N·m of extra torque, and the mechanism accelerates in the commanded direction. Notice carefully what the bigger current bought you: **more torque, and therefore more ability to accelerate against the load.** At no point did it say anything about what speed the mechanism should run at.

### 4.2 Unloaded: why a small current means high speed — and what finally stops it

Now remove the load completely — this is the bench setup of Demo 1. Command a deliberately small current, $I_q^* = 1$ A. The motor produces about 0.57 N·m, and the only thing opposing it is the motor's own internal friction. That puts us in the first row of §4.1's table with almost no opposition: $\tau_{\mathrm{net}} > 0$, so $\alpha > 0$ — and as long as that stays true, the speed simply keeps climbing. Give it a few seconds, and a current too small to even warm the windings has the shaft spinning at a couple of hundred rpm.

Nothing is broken. Nothing is even unusual. **A small current is a small torque, and a small torque applied continuously to an almost-free rotor adds up, second after second, into a large speed.**

So what finally stops it? To see clearly, first imagine the *idealized* case, because it shows us exactly what reality must break. In a fantasy motor — zero losses, unlimited voltage, a controller that can hold the commanded current at any speed — constant $I_q$ would mean constant $\tau$, so constant $\alpha = \tau/J$, so

$$
\omega(t) = \omega_0 + \frac{\tau}{J}\,t ,
$$

a speed growing forever, for as long as the command stands. The real AK80-9 breaks this fantasy in two places.

**The first is the electrical ceiling.** Ch 5 §3–4 established the key fact, once for the whole series: as the motor spins faster, its back-EMF $K_e\omega$ grows with it, using up the voltage that is available to push current through the windings. What is *new* in this mode is how the controller reacts. In Duty Cycle Mode, the drive voltage was fixed by your command, so the rising back-EMF simply squeezed the current down. Current Loop Mode does the opposite: the loop notices the real $I_q$ starting to drop below the command, and **raises its voltage to compensate** — spending more and more of the DC bus to defend your current against the growing back-EMF. (In steady state, $V_{\mathrm{required}} \approx I_q R + K_e\omega$ — the Ch 5 §3 equation, now read as a *requirement*.)

```text
motor accelerates → back-EMF rises → Iq starts to sag below the command
       → loop detects the error → voltage raised → Iq held again
       → (repeat, at ever-higher speed and ever-higher required voltage)
```

This defense really works — in Demo 1 you will see the current stay glued to 1 A during the whole spin-up. But it only works until the bus runs out:

```text
required voltage
      ▲
      │                        required ≈ IqR + Ke·ω
Vmax  ├─────────────────────────X──────────
      │                      ／
      │                   ／
      │                ／
      │             ／
      └──────────／──────────────────────► speed
```

At the point marked X, the controller has reached the maximum voltage the inverter can deliver from the finite DC bus. Beyond X, the commanded $I_q$ physically cannot be maintained: the real current — and with it the torque and the acceleration — falls away, no matter what the command says.

**The second is that "no load" never means "no opposition."** Bearing friction, gearbox friction, seal friction, and air drag are always there, and several of them grow with speed. So $\tau_{\mathrm{loss}}$ eats a bigger and bigger share of the commanded torque on the way up. Between the shrinking net torque, the voltage ceiling, and the firmware's own speed, thermal, and protection limits, the unloaded motor settles onto a steady plateau instead of accelerating forever. At small commanded currents that plateau can sit well below the voltage-limited top speed — the shaft simply stops climbing wherever the growing losses use up the whole commanded torque. You will watch this exact arc, spin-up to plateau, in Demo 1.

> **A constant $I_q$ means a constant torque only while the controller still has voltage left to enforce it.** Inside that envelope, the mode is a faithful torque source. At the edge of it, physics takes over.

### 4.3 Loaded: a steady speed can *emerge* — but it is never *controlled*

Between "stalled against a strong load" and "unloaded run-up to the plateau" lies the everyday case: a real mechanism whose opposing torque depends on how fast it runs. This is where constant-current operation produces its most misleading behavior — a stable, steady speed that *looks* exactly like speed control, and isn't.

Picture the AK80-9 driving a conveyor belt at a constant $I_q$, so a constant $\tau_{\mathrm{motor}}$. At standstill, the motor's torque beats the conveyor's opposition, so the belt accelerates. But a conveyor's total opposing torque — belt friction, bearing drag, air resistance — *grows with speed*. As the belt speeds up, $\tau_{\mathrm{opposing}}(\omega)$ climbs toward the fixed $\tau_{\mathrm{motor}}$ until the two meet. At that point $\tau_{\mathrm{net}} = 0$, the acceleration ends, and the belt cruises along at a perfectly steady speed:

```text
constant Iq → constant torque → belt accelerates → speed-dependent
opposition grows → torques balance → τ_net = 0 → steady speed
```

That steady speed is real — but look carefully at where it came from. It is the **equilibrium point of the mechanical system**: the speed at which *this particular belt's* losses happen to absorb *this particular torque*. The controller did not compute it, does not know it, and will not defend it. Put a few boxes on the belt and $\tau_{\mathrm{load}}$ rises: the balance breaks, the belt slows, and it settles again at a *new, lower* equilibrium speed — permanently. And note that the controller has done nothing wrong! Its one and only promise is "maintain $I_q$," and it is still keeping that promise perfectly while your conveyor runs slow.

> **Constant current ⇒ approximately constant torque. Constant speed happens only where $\tau_{\mathrm{net}} = 0$ — as an outcome of the mechanics, never as a goal of the controller.**

If your real requirement is *"hold this speed even when the load changes,"* that is a different job for a different loop — Velocity Loop Mode, which wraps this very current loop inside a speed controller and adjusts $I_q$ for you, automatically (Ch 8, where the conveyor story gets its happy ending). If your requirement is *"apply this torque, whatever speed results,"* then you are already in the right mode.

**The whole section in one picture** — one commanded current, and every behavior of this chapter falling out of what happens to stand in its way:

```text
                        Iq* held constant  →  τ ≈ 0.5701·Iq* held constant
                                     │
        ┌────────────────────────────┼─────────────────────────────┐
        ▼                            ▼                             ▼
   nothing in the way          a speed-dependent load         an immovable load
   (unloaded, §4.2)                (§4.3)                     (blocked, §5)
        │                            │                             │
   accelerates until           accelerates until               ω = 0, yet
   voltage/losses cap it       τ_motor = τ_opposing(ω)         Iq and τ at full
        │                            │                         commanded value
   fast plateau —              steady speed — an                    │
   not a target                equilibrium, not a target       heat, not motion
```

## 5. What Happens If the Shaft Is Blocked

Now take §4.1's third row to its extreme: command $I_q^* = 5$ A, and let the mechanism (or, gently, your own hand through a folded cloth) stop the output shaft from rotating at all. The speed is zero. The current is not.

Remember, the current loop neither knows nor cares about speed. So it simply carries on regulating $I_q$ to 5 A, and the motor carries on pressing against the obstruction with the full commanded torque of about 2.85 N·m — for as long as you let it. **Zero speed does not mean zero current, and it does not mean zero torque.**

Electrically, a stall is actually the loop's *easiest* operating point. With $\omega = 0$ there is no back-EMF at all, so §4.2's headroom problem disappears, and the controller holds the commanded current with voltage to spare.

Thermally, it is the *hardest*. The windings dissipate copper losses of $P_{\mathrm{Cu}} = I^2R$ continuously, and at zero speed there is no motion carrying energy out of the actuator — every watt you command becomes heat, right there in the windings. A motor that is standing perfectly still can be working at its electrical and thermal limit while looking completely idle.

Used deliberately and modestly, this stalled-torque behavior is a genuinely useful feature — pressing, tensioning, and holding tasks all live here, and you can feel it with your own hand at the bench (gently, through a folded cloth, at 1 A or less). But please treat any stalled current above a few amps as an actively heating condition: watch the driver temperature in Real-time Data, keep large-current stalls short, and never leave one running unattended. The series' full treatment of holding — why zero speed still costs current, how quickly things heat up, and what "holding" does and does not guarantee — is Ch 7 §5, where Current Brake Mode makes holding its entire purpose.

## 6. Commanded $I_q$ Is Not the DC Supply Current

A quick reminder in ten lines, because this confusion tends to reappear at exactly this bench moment: you command 2 A, you glance at the bench power supply, and it shows a completely different number — often far *less*, at low and moderate speeds. Nothing is wrong. The supply displays $I_{\mathrm{in}}$, the DC input current feeding the driver board. Your command sets $I_q$, the FOC q-axis component circulating inside the motor. They are two different physical quantities, connected only through the power balance: the bus must supply roughly the delivered mechanical power plus the losses, so $V_{\mathrm{bus}} I_{\mathrm{in}} \approx P_{\mathrm{mech}} + P_{\mathrm{loss}}$. At modest speeds, that needs only a fraction of an amp from the supply — even while the loop is faithfully holding $I_q$ at 2 A inside the motor. In general,

$$
I_{\mathrm{in}} \neq I_q ,
$$

and neither one can stand in for the other. Ch 2 owns the full story ($I_{\mathrm{in}}$ and input power in §2; the whole six-current cast in §1). You can see it in a single glance at the bench: the supply's front panel and CubeMarsTool's $I_q$ field, side by side, disagreeing.

## 7. The ±60 A Protocol Range Is Not a Rating

Five lines, placed here because this is the chapter where you start typing current values. The protocol can encode Current Loop commands anywhere in ±60 A (§2), but the AK80-9's hardware ratings are **12 A continuous / 28 A peak**. The wide encoding range exists to serve the whole AK driver family — it says nothing about what *this* motor can tolerate. The driver will happily accept a 45 A command; the windings will not survive it happily. So treat 12 A as your everyday ceiling, 28 A as a short-burst ceiling, and stay well below both while learning. The full explanation — every protocol range placed next to every rating, and why both sets of numbers exist — is Ch 2 §9.

## 8. The GUI `T` Control: Where Servo "Torque Mode" Actually Lives

Open CubeMarsTool's Servo Control interface and, right next to the I field we have been using, you will find a control labelled **`T`** that accepts a value in **N·m**. A torque field, in torque units, in the Servo interface — surely this must be a dedicated *Servo Torque Mode*, sitting alongside Current Loop, Brake, Velocity, and the rest?

It is not — and finding out why teaches you something important: the difference between the **conveniences the GUI offers** and the **modes the protocol actually defines**.

**What the manual says `T` does.** The `T` field is documented not under any "torque mode" heading, but under **§3.3.1.3 "Braking Mode"** (p. 25): *"enter the desired torque T, and the motor will brake with the desired torque."* So `T` is a **braking-by-torque convenience**. Instead of asking you for a braking current in amps (that is the B field — Ch 7), the GUI lets you describe the desired *braking* strength directly in N·m and does the conversion for you. It commands how strongly the motor resists motion — it is *not* a general-purpose torque command.

**What the protocol actually defines.** The Servo CAN protocol contains exactly seven control modes (manual §4.1, p. 31–32):

| Servo control mode | CAN Control Mode ID |
|---|:---:|
| Duty Cycle Mode | 0 |
| Current Loop Mode | 1 |
| Current Brake Mode | 2 |
| Velocity Mode | 3 |
| Position Mode | 4 |
| Set Origin Mode | 5 |
| Position–Velocity Loop Mode | 6 |

with the matching packet definitions:

```c
typedef enum {
    CAN_PACKET_SET_DUTY = 0,       // Duty Cycle Mode
    CAN_PACKET_SET_CURRENT,        // Current Loop Mode
    CAN_PACKET_SET_CURRENT_BRAKE,  // Current Brake Mode
    CAN_PACKET_SET_RPM,            // Speed Mode
    CAN_PACKET_SET_POS,            // Position Mode
    CAN_PACKET_SET_ORIGIN_HERE,    // Set origin position mode (zero mode)
    CAN_PACKET_SET_POS_SPD,        // Position-Velocity Loop Mode
} CAN_PACKET_ID;
```

Read the list twice if you like: there is no `CAN_PACKET_SET_TORQUE`, no eighth Servo ID, no frame anywhere in the Servo protocol that carries a torque in N·m. The `T` field exists only inside the GUI, as a friendlier front door to braking.

**What this means when you write your own CAN code.** Two simple rules cover every case:

1. **You want general torque control, in either direction → Current Loop Mode, ID 1.** Convert the desired torque to a current with §2's formula ($I_q^* = \tau^*/0.5701$) and send that current. This chapter *is* the Servo protocol's torque mode.
2. **You want braking / holding → Current Brake Mode, ID 2.** Its command is an unsigned braking-current magnitude, 0–60 A at the protocol level (manual §4.1.3) — the direction is chosen by the firmware, always opposing motion. Ch 7 is its chapter.

One last thing to keep separate: none of this is **MIT torque control** either. MIT (Force Control) mode is a completely separate protocol family under Control ID 8. Its command packet carries a torque value directly in N·m, with an AK80-9 range of ±18 N·m — the one and only place this actuator accepts newton-metres over the wire. (The manual's naming for MIT's torque pattern varies from page to page; Ch 11 settles the terminology and covers the mode in full.)

> **In one breath:** the GUI's `T` is a friendly shortcut for braking; the Servo protocol has no torque frame; real Servo torque control is this chapter's Current Loop ID 1 with $I_q = \tau/0.5701$; and direct N·m commands exist only in MIT mode (Ch 11).

## 9. Nearest Neighbors: Three Pointers

**vs. Duty Cycle Mode (Ch 5).** Duty holds the voltage fixed and lets the rising back-EMF squeeze the current down; Current Loop holds the current fixed and spends rising voltage to defend it — until the bus runs out (§4.2). Same physics, opposite fixed point. The full head-to-head is in Ch 12.

**vs. Current Brake Mode (Ch 7).** Here the command is a signed $I_q$ and *you* choose the torque direction; there the command is an unsigned magnitude and the *firmware* chooses the direction that opposes motion. Braking and holding get their own chapter next.

**vs. Velocity Loop Mode (Ch 8).** Velocity mode is this mode with a supervisor on top: an outer loop measures the speed and continuously rewrites the $I_q$ command that, here, you write by hand. §4.3's conveyor sits exactly on the boundary between them — full comparison in Ch 12.

## Demos

Both demos follow Ch 4 §6's ground rules: motor bench-mounted and calibrated per Ch 4, output shaft **unloaded** (except where a demo deliberately adds a hand, pad, or lever), 48 V supply with a conservative current limit, and CubeMarsTool connected with the Real-time Data panel open and the Stop button located. The manual's own preconditions for the mode apply throughout (§3.3.1.5): motor input power stable, connectors properly connected, upper computer successfully connected. Commands go into the Servo Control interface's **I field**, in amps, and commanding **0 A ends each demo** — it removes all torque and lets the shaft coast down on friction.

The currents we use (±1 A, which is about 0.57 N·m by §2's table) are deliberately gentle. But be aware that Demo 1 reaches **high speeds on a free shaft**: fit a tape flag or marker collar so rotation is visible, keep hands, hair, and cables clear, and know where Stop is *before* you send the first command.

### Demo 1 — Small Current, Big Speed

This demo makes the chapter's main idea real: a current too small to even warm the motor sends the shaft to high speed — because current is torque, and torque adds up over time.

**Starting condition:** calibrated per Ch 4, unloaded, shaft at rest and free to spin, Real-time Data plotting speed and current.

**Commands and expected observations:**

1. Command $I_q^* = 1$ A — about 0.57 N·m at the output shaft — and then do not touch anything else.
2. The shaft begins turning immediately, and it **keeps accelerating**: a smooth, continuous spin-up, not a step to some fixed "0.5 A speed." This is §4.2 happening live in front of you — a sustained net torque, adding up into speed.
3. Over the next seconds, the acceleration fades and the speed levels off onto a **steady plateau** — a couple of hundred rpm at the shaft, the point where the motor's losses have used up the whole 0.57 N·m. Expect the telemetry ERPM field to read in the tens of thousands; on our bench it settles near 38,000 ERPM ≈ 200 rpm (divide by 189 for shaft rpm — Ch 3 §2). Meanwhile, notice that the current trace has stayed pinned near 1 A the whole time: the loop kept its one promise while the speed ran free.
4. Command 0 A: the torque disappears, and the shaft coasts down on friction alone — one last reminder that there was never any speed target in the first place.

What to check, reading the commands and the outcomes side by side:

| Quantity | Commanded? | What you observe |
|---|:---:|---|
| Current $I_q$ | **yes** — 1 A | telemetry current pinned near 1 A, from first motion to plateau |
| Torque | indirectly — ≈ 0.57 N·m | a constant push; you cannot see it directly, but the current tells you it is there |
| Speed | **no** | rises continuously, then plateaus at a couple of hundred rpm (tens of thousands of ERPM) |

Write down the plateau values you measure (shaft rpm, ERPM, bus voltage) — they are your own bench's data points for §4.2's voltage/loss ceiling.

One thing the camera cannot show: the *mechanism* that ends the acceleration — back-EMF eating the voltage headroom until the controller hits its ceiling (§4.2) — happens inside the electronics. What you *can* see, at the end of Video 6.1, is its visible *effect*: the plateau.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
    <figure>
      <img src="images\image7.png" 
          alt="Servo General Control panel with the I field highlighted and 1A entered.">
      <figcaption>
        Figure 6.1: Servo Control panel with the I field highlighted and 1 A entered, framed together with the Real-time Data speed field mid-spin-up — the reader's reference for where the command goes and where its (indirect) effect appears. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
    <video src="videos\video4.mp4" width="640" height="360" controls></video>
     <p style="width: 640px; text-align: center; margin-top: 4px;">
     <i>Video 6.1: Output Shaft of the Motor rotating at a constant current of 1 A.</i>
     </p>
</figure>
</div>
  

### Demo 2 — Direction Flip

**Starting condition:** calibrated per Ch 4, unloaded, shaft flag fitted, at rest.

**Commands and expected observations:**

1. Command $I_q^* = +1$ A and let the shaft spin up for a couple of seconds (no need to reach the plateau).
2. Command $I_q^* = -1$ A in one step. The shaft does **not** jump into reverse. Instead, it visibly **decelerates** — the negative torque acting as a brake against the existing motion — slows down through a momentary standstill, and then **accelerates smoothly in the opposite direction**, all under one unchanging command.
3. Command 0 A and let it coast down.

One command produced two behaviors — braking, then reverse drive — because the sign fixed the *torque* direction, while the mechanics decided, moment by moment, what that torque actually did (§3).

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
    <video src="videos\video5.mp4" width="640" height="360" controls></video>
     <p style="width: 640px; text-align: center; margin-top: 4px;">
     <i>Video 6.2: Continuous clip of the flagged shaft: spinning forward under +1 A → command flipped to −1 A → visible deceleration → momentary stop → clean spin-up in reverse.</i>
     </p>
  </figure>
</div>

## When to Use Current Loop Mode — and When Not

Current Loop Mode is the right choice whenever the quantity you actually care about is **force or torque** — and the resulting speed or position is either unimportant, or somebody else's job:

**1. Force-limited gripping and pressing.** A gripper, clamp, or press driven at a fixed $I_q$ pushes with a bounded, predictable force, no matter where the jaws stop: the current limit *is* the force limit, thanks to §2's conversion. Objects of different sizes get held with the same firmness, and nothing bad happens when the motion stops — §5's stalled-torque behavior is exactly the feature you want here, kept within thermal reason.

**2. Tensioning.** Winding film, wire, thread, or webbing at constant tension is constant-torque work: command the $I_q$ that corresponds to the desired tension (times the spool radius), and let the pay-out speed float freely — which, as §4.3 showed, is exactly what this mode does naturally.

**3. A torque interface for your own outer controller.** If you are building your own speed, position, force, or impedance loop on external hardware, this mode is the natural input to the plant: your algorithm computes a desired torque every cycle, you convert it (§2) and send ID 1, and the actuator's inner loop handles all the fast electrical dynamics for you. You are then standing exactly where Ch 8's velocity controller stands internally — one rung up the cascade, with the current loop as your foundation.

**4. Torque-limited motion for safe interaction.** Where a mechanism meets people or delicate structures, driving it through a deliberately small $I_q$ guarantees that it can never push harder than the commanded torque — no matter what goes wrong with the motion plan. (For richer compliant behavior — stiffness and damping, not just a torque cap — MIT mode is the fuller tool, Ch 11.)

**5. Characterization.** A commanded, regulated torque is a clean experimental input: step $I_q$ and log the speed to estimate inertia and friction, or stall the shaft against a scale or load cell to compare the real torque constant with §2's model.

**When not:** the moment your requirement contains a number in rpm or in degrees, this is the wrong mode. A particular speed held against load changes is Ch 8; a position reached and held is Ch 9/10; dedicated braking or holding of a stopped shaft is Ch 7. And an unattended large stalled current is §5's thermal warning, not a use case. The systematic decision framework across all the modes is Ch 12.

## Where These Ideas Go Next

This chapter's mental model carries a lot of weight in the rest of the series:

| Established here | Continues in |
|---|---|
| Current commands torque; torque causes acceleration, not speed (§4) | Ch 8 §4–5 (the loop that *does* command speed — by rewriting $I_q$), Ch 11 (MIT torque = the same idea in N·m) |
| The voltage-limited speed ceiling under a held current (§4.2) | Ch 8 §6 (why an over-ambitious speed target cannot be reached) |
| A steady speed that emerges but is not controlled (§4.3's conveyor) | Ch 8 §4 (the loop that corrects the same disturbance), Ch 12 (mode choice) |
| Stalled shaft: full torque, zero speed, real heat (§5) | Ch 7 §5 — the series' definitive holding & heating treatment |
| Signed $I_q$ = a torque direction that you choose (§3) | Ch 7 §3 (the unsigned brake command as its counterpart) |
| No Servo torque frame; `T` = a braking shortcut (§8) | Ch 7 (Brake Mode proper), Ch 11 (MIT's direct N·m command) |
| The inner current loop as the cascade's foundation (§1, §9) | Ch 8 → Ch 9 → Ch 10, each wrapping the loop below it |

## Key Takeaways

- Current Loop Mode (Servo ID 1) closes the innermost loop: you command **$I_q$**, the controller regulates it against a measurement — and since $\tau_{\mathrm{out}} \approx 0.5701\,I_q$, this makes it the Servo family's torque mode (Ch 2 §6).
- Torque → current conversion: $I_q^* = \tau^*/0.5701$. A 2 N·m request is a 3.51 A command. It is a model, not a torque sensor (Ch 2 §6.6).
- The command is signed: the sign gives the **torque** direction, not the rotation direction. On a moving shaft, an opposing command brakes first, passes through zero, and then reverses (Demo 2).
- The core model, told once for the series: constant $I_q$ ⇒ approximately constant torque ⇒ **an acceleration set by $J\alpha = \tau_{\mathrm{motor}} - \tau_{\mathrm{load}} - \tau_{\mathrm{loss}}$** — never a commanded speed. A steady speed exists only where the torques balance, and only as an outcome.
- Unloaded, even a tiny current runs the shaft up to a high plateau: the loop defends $I_q$ against back-EMF by raising the voltage, until the bus's ceiling ends the climb (§4.2; Ch 5 §3–4 for back-EMF itself).
- A blocked shaft keeps its full commanded current and torque at zero speed — electrically easy, thermally expensive ($P_{\mathrm{Cu}} = I^2R$); Ch 7 §5 owns holding in full.
- $I_q$ is not the supply current ($I_{\mathrm{in}} \neq I_q$, Ch 2 §2), and the ±60 A protocol range is not a rating (12 A continuous / 28 A peak — Ch 2 §9).
- There is no Servo torque CAN frame: the GUI's `T` field is a braking-by-torque shortcut (manual §3.3.1.3); real Servo torque control is this chapter's ID 1; direct N·m commands live only in MIT mode (Ch 11).

## Safety Notes

- **Tiny currents reach big speeds.** An unloaded shaft under any sustained positive $I_q$ accelerates toward its voltage-limited top speed (§4.2, Demo 1) — secure the bench, fit a shaft marker, keep hands, hair, and cables clear, and know where Stop is *before* commanding.
- **A stall is a heating condition, not a resting one.** A blocked shaft dissipates $I^2R$ heat continuously, with no motion to carry it away (§5). Keep stalled currents small and brief, watch the Real-time Data temperature field, and never leave a stalled command unattended.
- Touch a powered shaft only through a pad, cloth, or lever — never bare fingers on a spinning or torque-bearing shaft (Ch 4 §6).
- Direction flips are dynamic events: the decelerate-and-reverse of Demo 2 gives a jerk to whatever is attached to the shaft. Do it unloaded, at the demo's gentle 1 A.
- The protocol's ±60 A is an encoding, not a permission: stay within 12 A continuous / 28 A peak, and well below both while learning (Ch 2 §9, Ch 4 §6).

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §4.1 "Servo Mode Control Modes and Description" (p. 31: Current Loop Mode defined as "a specified Iq current is given to the motor… output torque = iq·KT… can be used as a torque loop"; the seven Servo Control Mode IDs 0–6; extended-frame ID layout); §4.1.2 "Current Loop Mode" (p. 33: data-transmission definition — int32 payload = current × 1000, −60,000…+60,000 ↔ ±60 A; the `comm_can_set_current` example routine; `CAN_PACKET_SET_CURRENT` in the packet enum, p. 32); §3.3.1.5 "Current Loop Mode" (p. 26: GUI operation — "enter the desired current I, and the motor will operate at the desired current"); §3.3.1.3 "Braking Mode" (p. 25: the `T` field — "enter the desired torque T, and the motor will brake with the desired torque"); §4.1.3 "Current Brake Mode" (p. 33–34: brake-current magnitude 0–60 A, cited for §8's pointer); §4.2 / Parameter Ranges (p. 37–39: MIT Control ID 8 and the AK80-9 ±18 N·m torque range, cited for §8's disambiguation).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — torque constants 0.095 N·m/A (motor side) and 0.5701 N·m/A (output side), rated/peak currents 12 A / 28 A, and no-load speed context, via this series' reference table (Ch 1 §2) and their definitive treatment in Ch 2 §6 and §9.

---

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 7 — Current Brake Mode →](07_Current_Brake_Mode.md)
