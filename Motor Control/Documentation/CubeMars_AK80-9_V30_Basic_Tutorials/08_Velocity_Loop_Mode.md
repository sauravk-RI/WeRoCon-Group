# Chapter 8 — Velocity Loop Mode

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In Chapter 5, we compared Duty Cycle Mode to a car's accelerator pedal, and we promised that a "cruise control" mode was coming. In Chapter 6, we saw a conveyor belt that ran at a steady speed — but nobody was actually controlling that speed, and when boxes were loaded onto the belt, it slowed down and stayed slow (Ch 6 §4.3). This chapter keeps both promises. **Velocity Loop Mode** — Servo Control Mode ID 3 — is the first Servo mode where you command the thing most beginners *expect* to command: **speed**. You type in a target speed in ERPM, and the controller automatically produces whatever torque (and therefore whatever current) is needed to reach that speed and to keep it, even when the load changes.

That last part — "even when the load changes" — is really the whole chapter. Inside, this mode is simply Chapter 6's current loop with a supervisor added on top. This supervisor is an outer control loop. It measures the speed, compares it with your target, and then continuously adjusts the current command — the same $I_q$ command that you typed by hand in Chapter 6. Once you see this structure, everything special about the mode makes sense: why the current rises by itself when a load appears, why a fast unloaded motor uses almost no current, and why the motor stops *at* your target speed instead of racing up to its physical limit.

Here is the road map for this chapter:

- what Velocity Loop Mode is, its two command ranges (both are real!), and the manual's control diagram (§1);
- why your command is a *target*, not an instant change of speed (§2);
- the feedback loop: speed error → torque request → the inner current loop (§3);
- the most important idea of the mode — the load, not the speed, decides the current (§4);
- why the motor stops at your target instead of accelerating forever (§5);
- what happens when the target is impossible to reach (§6);
- a list of common misconceptions (§7) and short pointers to the neighboring modes (§8);

followed by a bench demo, advice on when (and when not) to use this mode, safety notes, and sources. As always, we assume a connected, calibrated motor and the safety ground rules of Ch 4 §6.

## 1. What Velocity Loop Mode Is

The manual describes the mode in one line (§4.1, p. 31): *"Velocity Mode: A specified operating speed is given to the motor (acceleration is default to the maximum value)."* And on the software side (§3.3.1.6, p. 27): in the Servo Control interface, *"enter the desired speed S (±50000 ERPM), and the motor will operate at the desired speed (with the default maximum acceleration)."*

Please read that part in the brackets twice, because it is really a warning: **this mode does not let you shape the acceleration**. When you command a new speed, the controller chases it with full available effort. The spin-up is quick and aggressive by design. (If you want to choose the acceleration yourself — for example, a gentle, smooth ramp — that is Position–Velocity Mode's job, in Ch 10. Here, the ramp is simply whatever maximum torque and physics give you.)

The command is signed: a positive value means one rotation direction, and a negative value means the other. The unit is **ERPM** — *electrical* rpm, not the rpm of the output shaft. Quick reminder, at its allowed two-line length: divide ERPM by 189 to get the shaft rpm (21 pole pairs × 9:1 gearbox; the full explanation and a worked table are in Ch 3 §2). Build this habit now, because our demos depend on it: 1,000 ERPM is only about 5 rpm — a crawl you can barely see — while 5,000–10,000 ERPM (about 26–53 rpm) is motion you can comfortably watch.

**There are two command ranges, and both are real.** The CubeMarsTool GUI's S field accepts **±50,000 ERPM** (manual §3.3.1.6, p. 27 — about ±265 rpm at the shaft). The CAN Velocity Loop message accepts **±100,000 ERPM** (manual §4.1.4, p. 35 — about ±529 rpm, which sits right at the actuator's published 570 rpm no-load speed; see the sanity check in Ch 3 §2). Neither number is a mistake. The GUI simply offers a narrower window than the protocol:

| Interface | Range | ≈ Shaft speed (÷189) | Manual source |
|---|---|---|---|
| CubeMarsTool GUI, S field | ±50,000 ERPM | ≈ ±265 rpm | §3.3.1.6, p. 27 |
| CAN `CAN_PACKET_SET_RPM` message | ±100,000 ERPM | ≈ ±529 rpm | §4.1.4, p. 35 |

At the protocol level, the command uses **Control Mode ID 3** with the packet ID `CAN_PACKET_SET_RPM` (enum list, p. 32). The speed travels as a plain int32 number in Data[0..3], and the number simply *is* the ERPM — there is no ×1000 scaling like the current messages, and no ×100,000 like duty. The manual's send routine (§4.1.4, p. 35) is the familiar four lines:

```c
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}
```

The same manual page also gives us something even more valuable than the message format: the **Simplified Control Diagram for Velocity Loop**. Translated into English and drawn flat, it looks like this:

```text
set speed ──►(−)──► [ speed-loop Kp ]──┐
              ▲   └►[ 1/s ]►[ Ki ]─────┴►(+)──► [ torque-limit ] ──► iq_ref ──► FOC current loop ──► motor
              │                                   protection                     (id_ref = 0)          │
              └───────────────[ d/dt ]◄── measured position ◄──── encoder ◄────────────────────────────┘
```

Every claim from this chapter's introduction is drawn in that picture. Let us walk through it slowly:

- The velocity controller is a **PI regulator** — "PI" means it has a *Proportional* part (the Kp block) and an *Integral* part (the Ki block with the 1/s integrator). Do not worry about the mathematics of PI control here; the simple idea is enough: the P part pushes harder when the speed error is bigger, and the I part slowly removes any small leftover error, so that the final speed lands almost exactly on your target.
- The output of this controller is **not** a speed. It is **`iq_ref`** — a *current request*. That request first passes through a torque-limit protection block (a safety clip), and then goes to exactly the same FOC current loop that you met in Ch 2 §5 and commanded directly in Ch 6, still running with $I_d \approx 0$.
- The measured speed comes from the encoder. The driver reads the position again and again and asks: "how fast is the position changing?" That is the `d/dt` block — the speed is calculated from position, using the same sensor every other mode uses.

So, in one sentence: **Velocity Loop Mode is Chapter 6 with the human replaced by a PI controller.** In Ch 6, *you* held the current pen. Here, an automatic supervisor holds it — and it has much faster reflexes than you.

## 2. A Target, Not an Instant Change

When you see the S field, it is tempting to read it as *"set the speed to 5,000 ERPM"* — as if speed were a value the firmware can simply write into the motor. It cannot. The command really means: **"work continuously so that the motor's speed becomes 5,000 ERPM, and then stays there."** Think of a thermostat: setting it to 22 °C does not make the room 22 °C instantly. It starts a process that *pushes the room toward* 22 °C.

Why can't the motor jump to the new speed instantly? Because of the core mechanical fact from Ch 6 §4: motors do not produce speed directly. They produce **torque**, and torque changes speed only through acceleration ($\tau_{\mathrm{net}} = J\alpha$). The rotor and gearbox have inertia — stored rotational energy that cannot appear or disappear in zero time. So a speed command from rest does not create a jump. It creates a fast, continuous climb:

```text
command: 5,000 ERPM (from rest)

speed:  0 ─► 1,800 ─► 3,600 ─► 4,700 ─► 4,970 ─► ≈5,000 ERPM  (settled)
        └──── full-effort acceleration ────┘└─ easing in ─┘
```

How long does the climb take? It depends on the inertia attached to the shaft, on how much torque the controller can apply (remember §1: acceleration defaults to maximum, so on an unloaded bench the climb is *fast* — a fraction of a second), on the available bus voltage, and on any load pushing back. The controller's promise is simple: it will keep pushing, with up to full effort, until the measured speed matches the target — and then it will keep watching (§3).

Because the command is a standing target and not a one-time action, it behaves nicely when you change it:

- Send a **new target while the motor is still climbing** toward the old one? No problem. The comparison in §3 simply starts using the new target, and the chase changes direction smoothly.
- Send a target **lower** than the current speed? The chase runs backwards: the controller applies braking torque and slows the motor down to the new number (§5 explains why it lands *on* the number, not below it).
- Send **zero**? The mode brakes the shaft to a controlled stop.

In every case, the meaning is the same: *make the measured speed equal this number, starting now.*

Two practical points before you go to the bench:

- **The telemetry speed field is the truth; the S field is the wish.** During the climb — and during every disturbance in §4 — the two values honestly disagree for a moment. That gap between them is the *velocity error*, and it is not a fault. It is the input that the whole mode runs on.
- **Reaching the target does not end the controller's work.** "Arrived" only means the error is currently zero. The measuring, comparing, and correcting continue at full rate — and that is exactly why the speed *stays* at the target when the world changes (§3, §4).

One friendly way to say it: your command is not an order that the motor performs once and then forgets. It is a standing instruction that the controller checks again and again, hundreds of times per second, forever — until you replace it.

## 3. The Feedback Loop: Error → Torque → Current

The controller's entire "thinking" fits into one subtraction. Let $\omega_d$ be the desired speed and $\omega$ the measured speed (calculated from the encoder position, §1). The **velocity error** is:

$$
e_\omega = \omega_d - \omega .
$$

The sign of this error is the whole decision:

| Velocity error | Meaning | Controller's response |
|---|---|---|
| $e_\omega > 0$ | too slow | increase the torque request |
| $e_\omega = 0$ | on target | keep just enough torque to hold the speed |
| $e_\omega < 0$ | too fast | reduce torque — or even apply braking torque |

The *size* of the error matters too, not only the sign, because the PI controller pushes harder when the error is bigger. Three snapshots with this chapter's demo target make it concrete:

| $\omega_d$ | measured $\omega$ | $e_\omega$ | What the controller "thinks" |
|---:|---:|---:|---|
| 5,000 ERPM | 3,500 ERPM | +1,500 | "Much too slow" — large torque request, hard acceleration |
| 5,000 ERPM | 4,900 ERPM | +100 | "Almost there" — small correcting torque, gentle approach |
| 5,000 ERPM | 5,250 ERPM | −250 | "Too fast" — cut the torque, brake if needed |

The loop never stops. It runs like this, thousands of times per second: measure the speed → calculate $e_\omega$ → convert it (through the PI rule) into a torque request → express that request as `iq_ref` → let the inner FOC current loop turn it into real winding current, and therefore real torque (Ch 2 §6's constants doing their usual job) → the torque speeds up or slows down the rotor → the encoder reports the new position → measure the speed again. Around and around:

```text
ω_d ──►(compare)──► e_ω ──► PI controller ──► torque request ──► iq_ref
          ▲                                                        │
          │                                              inner current loop (Ch 6)
          │                                                        │
          └────── measured ω ◄── encoder ◄──────── motor ◄─────────┘
```

Because the measured result is fed back into the next decision, this is called a **closed-loop** controller. The word "closed" simply means: the information travels in a circle — out to the motor, back through the encoder, and around again.

The best way to feel what "closed loop" means is to remember its opposite, which you already know. Duty Cycle Mode (Ch 5) is **open loop**: in the diagram above, its bottom feedback wire is cut. Duty mode fixes a voltage and never looks at the speedometer. That is exactly why, in Ch 5 §6, a touch of friction drags the speed down *permanently* — the disturbance is never even noticed. Close the wire, and the same disturbance becomes a detected error, and a detected error becomes a correction (§4).

One more note about the layered structure: nothing from Chapters 2–6 is thrown away or bypassed here. The velocity controller sits *above* the current controller and writes its command. In Ch 6, you held the `iq_ref` pen. This mode takes the pen from you and hands it to the PI regulator.

## 4. The Big Idea: The Load Decides the Current, Not the Speed

*(This section is this chapter's permanent contribution to the series. When later chapters say "load → current, automatically," they will point here.)*

Everything special about Velocity Loop Mode is one story, told from three different starting points.

**The load increases.** The motor is cruising at 5,000 ERPM with almost no load, sipping a small current. Now a load arrives — a hand drags on the shaft, the robot starts climbing a slope, boxes land on the conveyor. The extra opposing torque slows the rotor down. The measured speed dips below the target, so $e_\omega$ becomes positive. The controller responds in the only way it can: a bigger torque request, so a bigger `iq_ref`, so more current and more torque — until the speed is pushed back up to the target. And there it *stays*, with the controller supplying the higher current for as long as the load remains:

```text
load ↑ → speed dips → e_ω > 0 → torque request ↑ → Iq ↑ → speed recovers → (higher Iq held)
```

Notice what you did *not* do. You never asked for more current. You never asked for more torque. You asked for 5,000 ERPM, once, and the controller worked out the new "price" of that speed the moment the price changed. In real bench numbers: a lightly loaded shaft cruising on a few tenths of an amp can, under a serious load, run at the *same* speed on several amps — the commanded field unchanged, the current field completely different. Watch the Real-time Data current trace while you gently drag the spinning flag through a cloth, and you will see that current jump happen all by itself — the whole value of this mode in a single trace.

Also notice *who* did each part of the work, because this is the cascade of §1 doing its job: the velocity loop noticed the dip and raised its torque request; the inner current loop (Ch 6's machinery, unchanged) turned that request into real winding current; and Ch 2 §6's constants turned the current into torque. Three layers, one correction, and zero effort from you.

**The load decreases.** Now play the film backwards. The load is removed. The torque that was needed a moment ago is now too much, so the rotor speeds up past the target. $e_\omega$ becomes negative, and the controller cuts the torque request. If the shaft still runs too fast (a falling load can actually *drive* the motor), the request goes negative and the mode brakes its own motor back down to the target. Correction works in both directions, automatically.

**No load at all.** At a steady 5,000 ERPM with no load, $e_\omega \approx 0$: no acceleration is wanted, so no accelerating torque is requested. The only torque left to supply is the small amount that pays for bearing friction, gearbox friction, and air resistance. So the current is *small* — no matter how fast the shaft turns. A motor can spin near its top speed on a few hundred milliamps if nothing is fighting it.

Put the three stories together and you get the fact that surprises new engineers most often. Here it is, stated once for the whole series:

> **Speed does not decide the current. The load does.** Current buys torque (Ch 2 §6), and the velocity controller buys exactly as much torque as the load demands at the target speed — no more (the error would go negative) and no less (it would go positive).

A two-case comparison makes it impossible to miss:

| Case | Speed | Load | Required torque | Current |
|---|---:|---|---|---|
| A | 3,000 ERPM (≈ 16 rpm) | heavy friction load | large | **large** |
| B | 20,000 ERPM (≈ 106 rpm) | almost none | tiny (losses only) | **tiny** |

Case B spins more than six times faster than Case A, yet uses far less current — because current follows torque, and torque follows load.

One honest footnote to "the speed stays constant": it never stays *perfectly* constant. The loop is reactive. A load change must first cause a small, short dip in speed before there is any error to correct — the controller cannot fix a problem it has not yet seen. A well-tuned controller (helped by the integral part, §1) keeps that dip small and short, but it can never make it zero: detection needs a deviation. So the little dip you see whenever a load arrives is not a flaw. It is the feedback principle in action.

And now, the promised analogy, in its allowed six lines: this mode is **cruise control**. You set 80 km/h. On flat road, the engine barely works. Going uphill, the car sags for a moment and the system adds throttle. Going downhill, it backs off or brakes. The driver's foot — Ch 5's accelerator pedal, which commanded *effort* — has been replaced by a system that commands the *outcome*. You choose the speed; the controller negotiates with the terrain, continuously.

## 5. Why It Stops *At* Your Target — Not at the Physical Limit

Chapter 6 left us with a question that this section must answer. There, an unloaded motor fed with a constant 1 A kept accelerating and accelerating, all the way up to a steady plateau set by the voltage ceiling and the losses (Ch 6 §4.2). Velocity Loop Mode also works by producing torque — so why doesn't *it* run away to the same plateau? Why does an unloaded motor commanded to 5,000 ERPM arrive at 5,000 and calmly park there?

Because the two modes keep different things constant. Current Loop Mode keeps the **command** constant: 1 A is 1 A at every speed. The controller never asks "have we arrived?" — there is no destination to arrive at — so the torque continues until physics itself (back-EMF meeting the voltage ceiling, plus losses) chokes it off. Velocity Loop Mode keeps the **target** constant, and lets the torque *shrink together with the remaining error*. Starting from rest with a 5,000 ERPM command, the error is large and the controller pushes hard. At 3,000 ERPM the error — and with it the requested torque — is smaller. At 4,800 ERPM, smaller still. And as $e_\omega \to 0$, the accelerating torque winds down to zero *automatically, by design*:

```text
Current Loop (Ch 6):            Velocity Loop (this mode):

command Iq = constant           command ω_d = constant
   ↓                               ↓
torque ≈ constant               torque follows the shrinking error
   ↓                               ↓
accelerates until physics       accelerates, ever more gently,
imposes the ceiling             until ω = ω_d — then stops accelerating
   ↓                               ↓
plateau set by voltage/losses   plateau set by YOUR number
```

Here is the landing, frame by frame, with this chapter's demo command. Watch how the error does all the steering:

| Moment | measured $\omega$ | $e_\omega$ | Torque request | Result |
|---|---:|---:|---|---|
| command sent, at rest | 0 | +5,000 | large | hard acceleration |
| mid-climb | 3,000 | +2,000 | smaller | still accelerating, less fiercely |
| approaching | 4,800 | +200 | small | gentle final approach |
| arrived | ≈ 5,000 | ≈ 0 | friction-sized only | constant speed — *at your number* |

Nothing from outside slowed that approach down. The torque became smaller because the thing that creates it — the error — was used up by the very acceleration it caused. Ch 6's constant command has no such self-shrinking ingredient. That is the entire difference between the two behaviors.

Slowing down works the same way, just mirrored: command a *lower* speed than the current one, the error becomes negative, the controller applies braking torque, and the motor slows down to the new target — instead of just coasting on friction.

Does the torque fall all the way to zero at the target? Almost never. Even a constant speed has a running cost: friction and air resistance (plus any external load, §4). So the controller settles at the small "maintenance" torque — and the small maintenance current — that keeps $e_\omega$ pinned at zero. That little fraction-of-an-amp you will see on the current field at a steady, unloaded speed is not noise. It is the loop paying the standing cost of the speed you ordered.

> **Ch 6's promise was: "keep this current, whatever speed results." This mode's promise is: "keep this speed, whatever current it takes." Same physics — opposite choice of what stays fixed.**

## 6. When the Target Is Impossible to Reach

Now command something unreasonable — say, the protocol-maximum 100,000 ERPM on a loaded shaft, or any target beyond what the supply voltage can support. The controller does not show an error message. It simply *tries*. Forever.

The limiting mechanism is the one you first met in Ch 5 §4 and saw formalized in Ch 6 §4.2: back-EMF grows with speed and eats up the supply voltage. Past a certain speed, no controller can push the needed current — and therefore the needed torque — through the windings, no matter how large `iq_ref` becomes. (The torque-limit block from §1's diagram and the firmware's protections also clip the request.) So the motor accelerates up to its voltage-and-load-limited maximum and holds there, while the velocity error stays positive forever:

```text
ω_d = 100,000 ERPM   →   ω settles at (the achievable maximum)   →   e_ω stays large, forever
                          controller: pushing at full effort, indefinitely
```

A velocity controller cannot overrule physics. It can only spend all the torque the actuator can produce. The practical meaning is important: an unreachable target means a motor running **flat-out at its limits, indefinitely** — maximum sustainable current, maximum heat, zero margin. That is a condition to detect and fix, not one to leave running. Two habits protect you:

- **Choose targets comfortably inside the achievable range.** The ÷189 rule, together with the 570 rpm no-load speed (Ch 1 §4), tells you roughly where the range ends — and remember that it shrinks under load and on a lower supply voltage.
- **Watch the gap between commanded and measured speed.** A large, persistent gap in the telemetry is a red flag: either the target is unreachable, or the mechanism is jammed — and in both cases, the controller is pouring current in, trying anyway. This same check works at normal targets, too: commanded-versus-measured is the mode's free health readout, on every Real-time Data screen.

## 7. Common Misconceptions

Five common misreadings of this mode, each answered in two lines:

- **"Velocity Mode controls speed directly."** Not quite. It controls *torque*, recalculated continuously from the speed error; speed is the settled result (§3, §5). You will feel this difference the first time the current changes "on its own."
- **"Higher speed means higher current."** No — the load sets the current, not the speed (§4). Fast-and-unloaded is cheap; slow-and-loaded is expensive. §4's two-case table shows both.
- **"The controller calculates the required torque once."** It recalculates hundreds to thousands of times per second, forever — which is exactly why disturbances get corrected instead of piling up (§3, §4).
- **"The commanded speed is guaranteed exactly, at all times."** Corrections need a detectable error first, so every load change causes a brief, small dip (§4) — and an unreachable target is never met at all (§6).
- **"The motor is at the commanded speed the instant you press send."** Inertia forbids it. The command starts a full-effort chase, not a teleport (§2) — and "full effort" is literal, because acceleration defaults to maximum (§1).

## 8. Nearest Neighbors: Four Pointers

**vs. Current Loop Mode (Ch 6).** There, you write `iq_ref` by hand and the speed is whatever happens to result; here, a PI supervisor writes it for you and the speed is the promise (§5). Ch 6 §4.3's conveyor sits right on the border between them — full head-to-head in Ch 12.

**vs. Current Brake Mode (Ch 7).** Brake Mode opposes whatever motion exists — its only implicit target is zero. Velocity Mode *defends* motion at a target you choose, braking or driving as needed. One mode resists speed, the other maintains it — see Ch 12.

**vs. Position Mode (Ch 9, next).** Velocity Mode answers "how fast?" and never stops by itself; Position Mode answers "where?" and stopping at the answer is its whole job — with this chapter's loop running *inside* it, as the middle layer of the cascade. The full contrast is Ch 9 §1 and Ch 12.

**vs. MIT Velocity Control (Ch 11).** MIT's velocity pattern ($K_p = 0$, $K_d > 0$) lets you choose the error-to-torque gain yourself, in every command; Servo Velocity Mode keeps its PI gains internal and asks only for the target. Flexibility versus simplicity — Ch 11 §8 and Ch 12.

## Demo — Hit and Hold a Speed

The demo follows Ch 4 §6's ground rules: motor bench-mounted and calibrated per Ch 4, output shaft **unloaded**, 48 V supply with a conservative current limit, CubeMarsTool connected with the Real-time Data panel open and the Stop button located. The manual's preconditions for the mode apply throughout (§3.3.1.6): motor input power stable, connectors properly connected, motor in servo mode, upper computer successfully connected. Velocity commands go into the Servo Control interface's **S field**, in ERPM; commanding **S = 0 ends the demo** by regulating the shaft to a stop. All speeds below follow Ch 3 §2's ÷189 rule and are chosen to be *visibly* moving (the old habit of commanding 1,000 ERPM gives only a ~5 rpm crawl): **5,000 ERPM ≈ 26 rpm** and **10,000 ERPM ≈ 53 rpm**. Two mode-specific cautions before your first command: spin-up happens at the **default maximum acceleration** (§1), so the step to the target is quick — and an obstructed shaft makes the controller *raise* the current automatically (§4), so if you ever apply a friction disturbance, do it gently, through a pad, and briefly.

The mode's basic contract, seen with your own eyes: one number in, that speed out — then a second number, and that speed out.

**Starting condition:** calibrated per Ch 4, unloaded, shaft marked with a tape flag or marker collar, Real-time Data plotting speed and current.

**Commands and expected observations:**

1. Command **S = 5,000 ERPM**. The shaft steps up quickly (§1's maximum default acceleration — expect a snap, not a gentle ramp) and settles at a steady, comfortable ≈ 26 rpm. The telemetry speed field reads ≈ 5,000 ERPM and *stays* there. Look at the current field: only a small fraction of an amp — the friction-only cost from §4's third story.
2. Command **S = 10,000 ERPM**. A second quick step, settling at ≈ 53 rpm — double the ERPM, double the shaft speed. The ÷189 rule, working on camera. The steady current barely changes: twice the speed, still almost no load, still a tiny cost (§4).
3. Watch the settled speed field for a while. It does not drift, sag, or wander — the loop is re-checking it thousands of times per second (§3).
4. Command **S = 0**: the controller actively brakes the shaft down to a stop — a *controlled* landing at zero, clearly different from Ch 6's slow friction coast-down after power-off (§5).

| Quantity | Commanded by you? | What you observe |
|---|:---:|---|
| Speed | **yes** — 5,000 then 10,000 ERPM | settles at ≈ 26 then ≈ 53 rpm and holds |
| Acceleration | no — defaults to maximum | quick, snappy steps between targets (§1) |
| Current | no | small at both speeds; chosen by the controller (§4) |
| Stop behavior | yes — S = 0 | regulated stop at zero, not a coast-down |

For a picture of what a settled speed looks like on the bench, Figure 8.1 and Video 8.1 show the shaft turning at a rock-steady 38,073 ERPM (≈ 200 rpm) while the panel's speed field holds still — the same unwavering hold you have just produced at 5,000 and 10,000 ERPM, at a brisker pace that is still inside the GUI's ±50,000 ERPM window.

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image8.png" 
          alt="Servo Control Panel for Motor spinning at constant speed 38073 ERPM">
      <figcaption>
        Figure 8.1: Servo Control Panel for Motor spinning at constant speed 38073 ERPM. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video6.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 8.1: Motor spinning at constant speed 38073 ERPM</i>
      </p>
</figure>
</div>

## When to Use Velocity Loop Mode — and When Not

Use this mode whenever the job is a **steady rotation speed under a changing load** — the exact task the mode was built for, solved with one number: conveyor belts and feed rollers that must not slow down when material lands on them; wheels and drive bases that translate "travel at this velocity" into a shaft speed; fans, pumps, spindles, and mixers whose process cares about rpm, not torque. Its strengths are the simplest possible interface (one signed ERPM value), automatic load compensation (§4), and correction in both directions — it brakes an overspeeding shaft just as readily as it drives a slow one.

Skip it when one of its limits gets in the way. If the job is really a *torque* or a *force*, commanding speed is the wrong contract — use Ch 6. If the job is a *position*, remember this mode never stops on its own — use Ch 9/10, which wrap this loop rather than replace it. If you need to shape the *acceleration* — soft starts, smooth ramps — this mode's default-maximum acceleration (§1) is exactly what you do not want: Ch 10 owns the trapezoid profile. If the shaft may be blocked during normal operation, remember the mode's instinct is to *fight harder* (§4, §6) — a jam becomes a maximum-effort heating condition, so plan protection for it. And a target near or beyond the achievable limit buys you a motor running flat-out forever (§6). The complete mode-selection framework is Ch 12.

## Where These Ideas Go Next

| Established here | Continues in |
|---|---|
| **Load, not speed, decides the current — corrected automatically** (§4) | Ch 9 §4 and Ch 10 §6 cite it for the inner loops of position control; Ch 12's decision framework |
| The cascade: an outer loop writes the inner loop's command (§1, §3) | Ch 9 (position wraps velocity wraps current) — the series' structural spine, completed |
| Target-seeking torque that shrinks with the error (§5) | Ch 9 §3 (position error plays the same role), Ch 11 (MIT's explicit error-times-gain rule) |
| Closed vs open loop, told through one disturbance (§3–§4) | Ch 12's Servo-mode comparisons (Boxes 1 and 3) |
| Two real command ranges, GUI vs protocol (§1) | Ch 3 §6's master table records both; Ch 12's cautions recap |
| Unreachable target = permanent full effort (§6) | Ch 9's max-speed warning context, Ch 12's cross-cutting cautions |
| Internal PI gains vs MIT's exposed gains (§1, §8) | Ch 11 §8 (MIT Velocity), Ch 12's Servo-vs-MIT discussion |

## Key Takeaways

- Velocity Loop Mode (Servo ID 3, `CAN_PACKET_SET_RPM`, int32 = ERPM) commands a **target speed**; acceleration defaults to maximum. GUI S field: ±50,000 ERPM; CAN message: ±100,000 ERPM — both ranges are real, the GUI is simply narrower (§1). Divide by 189 for shaft rpm (Ch 3 §2).
- Structurally, it is **Ch 6 with a supervisor**: a PI velocity controller converts the speed error into `iq_ref` for the same FOC current loop, with the speed measured from the encoder — the manual's own control diagram, translated (§1, §3).
- The command is a standing target that the loop re-checks thousands of times per second — not an instant change of state (§2).
- **The core fact, owned by this chapter:** the load decides the current. Load on → dip → automatic current rise → recovery. Load off → the reverse. No load → tiny current at any speed. Corrections need a brief, detectable dip first — small, but never zero (§4).
- It stops *at* the target because the accelerating torque shrinks together with the error — unlike Ch 6's constant command, which accelerates until physics sets the ceiling. At steady speed, a small leftover current pays the friction bill (§5).
- An unreachable target is chased at full effort, forever: the motor parks at its voltage-and-load-limited maximum with a permanent positive error and maximum sustained current (§6).
- The mode is cruise control: you pick the outcome, and the regulator negotiates with the terrain — the closed-loop answer to Ch 5's accelerator pedal.

## Safety Notes

- **Spin-up happens at the default maximum acceleration** (§1): every new target is chased at full effort. Start with modest targets (≈ 5,000–10,000 ERPM), keep the shaft unloaded while learning, and expect the step to be quick enough to jerk anything attached to the shaft.
- **A blocked shaft makes the controller raise the current automatically** (§4). A jam here is not a stall at *your* chosen current, as in Ch 6 — it is an escalating fight, up to the firmware's limits. Apply disturbances gently, through a pad, briefly, and with the current field in view.
- **Unreachable or excessive targets mean sustained full effort** (§6): maximum current and maximum heat, indefinitely. Treat a persistent gap between commanded and measured speed as a fault condition; stay well inside the achievable range and within the 12 A continuous / 28 A peak ratings, regardless of the ±100,000 ERPM encoding (Ch 2 §9).
- Long runs under a heavy load are a heating condition even at constant speed — the current is high whenever the load is (§4). Watch the Real-time Data temperature field during loaded runs (Ch 7 §5's physics).
- The usual high-speed free-shaft precautions apply throughout (Ch 4 §6): tape flag fitted; hands, hair, and cables clear; the Stop button located before the first command. Touch a spinning shaft only through a pad or cloth.
- S = 0 is a *regulated* stop — the mode brakes the shaft down to zero. Use it to end experiments, rather than cutting the power in mid-spin.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §4.1 "Servo Mode Control Modes and Description" (p. 31: "Velocity Mode: A specified operating speed is given to the motor (acceleration is default to the maximum value)"; Servo Control Mode ID 3 among the seven mode IDs; extended-frame ID layout); §4.1.4 "Velocity Loop Mode" (p. 35: the Simplified Control Diagram for Velocity Loop — PI speed regulator with torque-limit protection feeding `iq_ref` into the FOC current loop, `id_ref = 0`, speed feedback from differentiated position; data-transmission definition — int32 payload, −100,000 … +100,000 representing −100,000 … +100,000 electrical RPM; the `comm_can_set_rpm` example routine); the CAN packet enum (p. 32: `CAN_PACKET_SET_RPM`); §3.3.1.6 "Velocity Loop Mode" (p. 27: GUI operation — "enter the desired speed S (±50000 ERPM), and the motor will operate at the desired speed (with the default maximum acceleration)"); §4.1.2 "Current Loop Mode" (p. 33: cited for §5's contrast, via Ch 6).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — 21 pole pairs and the 9:1 reduction behind the ÷189 ERPM conversion (via Ch 3 §2), the 570 rpm no-load speed used in §1/§6, and the 12 A / 28 A ratings cited in the safety notes, via this series' reference table (Ch 1 §2).
3. **CubeMarsTool upper-computer software** — Servo Control S field and Real-time Data plots shown in Figure 8.1; © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.