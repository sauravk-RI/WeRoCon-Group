# Chapter 5 — Duty Cycle Mode

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

Welcome to Part II! This is the chapter where you finally make the motor **move**. Everything in Part I (Chapters 1–4) was preparation; from here on, every chapter is hands-on.

Part II teaches the Servo modes in a special order. The order follows the motor controller's own internal structure, called the **control cascade** — the loops inside the controller, stacked one inside the other, from voltage at the bottom to position at the top:

```text
   Ch 5        Ch 6         Ch 8         Ch 9
  ┌──────┐   ┌─────────┐  ┌──────────┐  ┌──────────┐
  │ DUTY │ → │ CURRENT │→ │ VELOCITY │→ │ POSITION │
  │ (V)  │   │  (Iq)   │  │  (ERPM)  │  │   (°)    │
  └──────┘   └─────────┘  └──────────┘  └──────────┘
   voltage    innermost     wraps the     wraps the
   effort,      loop       current loop  velocity loop
  open-loop
```

Duty Cycle Mode sits at the very bottom of this ladder. In this mode, you command the **voltage effort** of the drive — how strongly the electronics push on the motor — and *nothing else*. The controller does not watch the speed, does not watch the current, and does not correct anything for you. Engineers call this kind of control **open-loop**: there is no feedback loop closing around a target.

That makes Duty Cycle Mode the simplest and most "raw" mode the AK80-9 offers. It is also the perfect place to start learning, and here is why: because no control loop stands between your command and the motor, you get to watch the motor's own natural electrical behavior directly. The star of that behavior is something called **back-EMF** — a voltage the motor generates by itself when it spins. Don't worry if that term is new; this chapter will explain it step by step, and what you learn here will be used again and again in later chapters (Ch 6 §4 and Ch 8 §6 both point back here).

One honest warning before we begin: Duty Cycle Mode can feel confusing at first, because the thing you command is not a familiar mechanical quantity like torque or speed. That confusion is normal, and this chapter clears it up piece by piece:

- what Duty Cycle Mode is, and what the *signed* duty command really means physically (§1);
- the command range and the CAN protocol encoding (§2);
- why a voltage command makes a motor move at all (§3);
- the life of a duty step, from the first burst of current to a steady speed — the heart of the chapter (§4);
- negative duty: how a one-way DC supply can drive the motor backwards (§5);
- why duty *looks like* speed control but is not (§6);
- why the DC-bus voltage matters (§7);

followed by a hands-on bench demo, advice on when (and when not) to use this mode, safety notes, and sources. Everything assumes a connected, calibrated motor and Ch 4 §6's safety ground rules.

## 1. What Duty Cycle Mode Is — and What the Signed Command Really Means

Let's start with the manual's own definition, which is only one line (§4.1, p. 31): *"A specified duty cycle voltage is given to the motor, similar to square wave drive form."*

Duty Cycle Mode is Servo Control Mode ID 0 (Ch 4 §1). Its command is one single signed number:

$$
-0.95 \le D_{\mathrm{cmd}} \le +0.95
$$

This number carries two pieces of information:

- Its **magnitude** (the size, ignoring the sign) is the requested drive level — what fraction of the DC-bus voltage the inverter should use to push on the motor.
- Its **sign** (+ or −) is the requested drive direction.

So $D_{\mathrm{cmd}} = +0.20$ simply means: *"push on the motor with roughly 20% of the available voltage, in the positive direction."*

And that is the **whole** command. You did not ask for any speed. You did not ask for any current or torque. So the controller does not hold any of those for you — whatever current, torque, and speed appear are simply the *result* of the motor's situation. The chain of cause and effect looks like this:

$$
D \;\rightarrow\; V \;\rightarrow\; I_q \;\rightarrow\; \tau \;\rightarrow\; \alpha \;\rightarrow\; \omega
$$

Read it left to right: your duty command $D$ sets an effective voltage $V$; that voltage pushes a torque-producing current $I_q$ through the windings (Ch 2 §5); the current makes torque $\tau$; torque makes angular acceleration $\alpha$; and acceleration, over time, gives the motor its speed $\omega$. Only the first arrow is under your control. Everything after it belongs to physics.

This is the single most important fact of the chapter, so let's say it clearly once: **Duty Cycle Mode is open-loop with respect to every mechanical quantity.** Every behavior we study in §4 follows from plain electrical physics, not from any clever control algorithm — because in this mode, there isn't one.

To place the duty command onto the series' master hardware picture (Ch 2 §10 — supply → inverter → windings → torque → gearbox), here is where it enters and what it passes through:

```text
                YOU
                 │  D_cmd  (signed drive level)
                 ▼
        FOC / modulation           ── decides the three-phase action
                 │                    at the commanded drive level
                 ▼
          PWM generation           ── per-leg switching patterns
                 │
                 ▼
         MOSFET inverter           ── switches the 48 V bus
                 │
                 ▼
      three-phase voltages         ── the quantity duty actually sets
                 │
                 ▼
      three-phase currents → Iq → torque → acceleration → speed
                             (all of these: unregulated results)
```

Notice where the arrow from "YOU" stops having authority: at the three-phase voltages. Everything below that line is decided by the motor's physics and by whatever load is attached.

### The signed command is not the physical PWM duty ratio

Here is a question that confuses almost every beginner, so let's face it directly. In Ch 2 §3 we defined the *physical* PWM duty ratio of a switch:

$$
D_{\mathrm{PWM}} = \frac{t_{\mathrm{ON}}}{T_{\mathrm{PWM}}}
$$

— the fraction of one PWM period that a switch spends turned ON. Because a switch cannot be ON for a *negative* amount of time, this physical ratio is always between 0 and 1. It can never be negative.

But the CubeMars command $D_{\mathrm{cmd}}$ *can* be negative. So these must be two different things — and they are. Mixing them up causes two classic misunderstandings:

**Misunderstanding one: "$D_{\mathrm{cmd}} = -0.2$ means −20% ON time."** No — that would be physically meaningless. Remember the two jobs the command does:

```text
             CubeMars Duty Command  D_cmd
                        |
              +---------+---------+
              |                   |
          |D_cmd|              sign(D_cmd)
              |                   |
              v                   v
       drive/modulation      drive direction
          strength           (+ or −, per the
       (0 … 0.95)         configured convention)
```

The physical PWM ratios inside the inverter stay in their normal 0–1 range in *both* directions. What the sign changes is *which switching arrangement* those ratios are applied to — and that flips the direction of the electrical push. §5 shows exactly how.

**Misunderstanding two: "$D_{\mathrm{cmd}} = 0.5$ means every MOSFET is ON 50% of the time."** Also no. Think back to Ch 2 §4: if all six switches ran one identical fixed pattern, the stator's magnetic field would stand still — and a still field cannot rotate the rotor. So the controller keeps doing its normal job underneath your command: FOC reads the rotor position from the encoder and decides what three-phase action is needed; the modulator turns that into three *different, continuously changing* switching patterns, one per phase leg; the inverter switches them (Ch 2 §5). Your duty command only sets the **overall signed drive level** at which all of this machinery operates.

So here is the honest one-sentence summary of the mode: *you say how strongly, and in which direction, the motor is electrically driven; the controller handles the three-phase switching underneath; and physics decides everything else.*

## 2. The Command Range and the Protocol

**In CubeMarsTool**, Duty Cycle Mode lives in the Servo Control interface's duty field (Ch 4 §3, area E — General Control). The manual gives the usable GUI range as **0.005–0.95 by default** (§3.3.1.8, p. 28): enter the desired duty cycle and *"the motor will operate at the desired duty cycle."*

What do these numbers mean in volts? On a 48 V bus, use the simple picture from Ch 2 §3 — effective drive ≈ duty × bus voltage:

| $|D_{\mathrm{cmd}}|$ | Effective drive @ 48 V | What it means in practice |
|---:|---:|---|
| 0.005 | ≈ 0.24 V | GUI minimum — often too weak to beat friction; the shaft may not move |
| 0.05 | ≈ 2.4 V | a gentle first-spin value (the demo's first step) |
| 0.25 | ≈ 12 V | the worked-example drive used in §4 |
| 0.50 | ≈ 24 V | a strong drive — treat it with respect at low speed (§4, Stage 5) |
| 0.95 | ≈ 45.6 V | GUI maximum — the drive keeps a safety margin below the full bus |

Both ends of the range teach something. The minimum (0.005) shows that the smallest command the software *accepts* is not necessarily enough to make the shaft *move*. And the maximum stops at 0.95 rather than 1.00 because the drive electronics keep a small working margin below full bus voltage — it is a practical ceiling, not a missing 5% of power.

**Over CAN**, the command is Control Mode ID 0 with a 4-byte payload: a signed 32-bit integer (int32) equal to **duty × 100,000**, sent across the frame's first four data bytes (manual §4.1.1, p. 32–33):

| Data bit | Data[0] | Data[1] | Data[2] | Data[3] |
|---|---|---|---|---|
| Contents | duty bits 25–32 | duty bits 17–24 | duty bits 9–16 | duty bits 1–8 |

The manual's own example function shows the scaling in a single line:

```c
void comm_can_set_duty(uint8_t controller_id, float duty) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}
```

So $D_{\mathrm{cmd}} = +0.20$ travels as the number 20,000, and $D_{\mathrm{cmd}} = -0.20$ travels as −20,000. Notice something interesting: **the protocol is signed even though the GUI documents a positive range.** The duty row of Ch 3 §6's master units table records both facts. And as always in this series: an encoding range only tells you what the *message format* can express, not what is sensible or safe to command (Ch 2 §9).

## 3. Why a Voltage Command Moves the Motor

Here is a fair question: duty commands a *voltage* — but torque comes from *current* (Ch 2 §6). So how does a voltage command end up moving anything?

The answer is the motor's electrical equation, and it is the one equation this whole chapter runs on. A motor winding has three electrical properties: **resistance** (it resists current), **inductance** (it resists *changes* in current), and — once the rotor starts turning — a self-generated voltage called **back-EMF** (short for "back electromotive force"). Back-EMF is the motor acting as a generator at the same time as it acts as a motor: a spinning rotor with magnets induces a voltage in the windings, and that voltage pushes *back* against your drive. In the simplified DC-motor-like form we use for intuition:

$$
V \;=\; iR \;+\; L\frac{di}{dt} \;+\; E,
\qquad
E = K_e\,\omega
$$

where $V$ is the applied effective drive voltage, $i$ the motor current in this simplified model, $R$ the winding resistance, $L$ the inductance, and $E$ the back-EMF — which grows in direct proportion to speed $\omega$, through the back-EMF constant $K_e$.

A friendly way to read the equation: think of $V$ as a fixed **budget** of voltage that gets spent in three ways —

```text
applied voltage  =  resistive drop (iR)
                  + voltage that changes the current (L di/dt)
                  + back-EMF generated by rotation (Ke·ω)
```

Let's meet the three "spenders" one by one:

- **$iR$, the resistive term** — the steady cost of carrying current at all. Motor windings are built with *low* resistance on purpose, so this cost is small per ampere. Keep that in mind: it means any leftover voltage turns into a *lot* of current (§4, Stages 1 and 5).
- **$L\,di/dt$, the inductive term** — the "transient" term. It only exists while the current is *changing*, and it is the reason current cannot jump instantly when you step the duty. (This is Ch 2 §3's rule "voltages cause currents" written as mathematics.)
- **$K_e\omega$, the back-EMF term** — zero when the motor stands still, and growing steadily as the motor speeds up. This is the term the whole chapter is really about.

Whatever the back-EMF takes from the budget is no longer available for driving current. The leftover part, $V - K_e\omega$, is called the **voltage headroom** — the portion of your drive that actually pushes current through $R$ and $L$. Remember this word, *headroom*; we will use it constantly.

Two short reminders of things owned by earlier chapters. First, cause and effect always runs voltage → current, never the other way: the inverter sets voltage conditions, and the currents *respond* through the winding's dynamics (Ch 2 §3). Second, the current that matters for torque is the FOC current $I_q$, which becomes output torque via $\tau_{\mathrm{out}} \approx 0.5701\,I_q$ (Ch 2 §6) — in this chapter's simplified reasoning, the single current $i$ stands in for it.

> **A note on the model.** $V = iR + L\,di/dt + K_e\omega$ is deliberately simplified — it treats our three-phase motor like a simple DC motor. The rigorous model for this kind of motor uses the d–q voltage equations under FOC. But every conclusion we draw from the simple form — the current burst at standstill, current falling as speed rises, the headroom picture — carries over correctly. Simple model, correct intuition.

## 4. The Life of a Duty Step: From Inrush to No-Load Speed

This section is the heart of the chapter — and the series' one full treatment of the idea that *back-EMF rises with speed and limits the current*. Later chapters point back here instead of explaining it again.

We will follow one single experiment all the way through, in slow motion. The motor is at rest, nothing is attached to the shaft, and at time $t = 0$ you apply one fixed duty command: $D_{\mathrm{cmd}} = 0.25$ on a 48 V bus. That is an effective drive of about $0.25 \times 48 = 12$ V, held constant the whole time. (This is essentially the experiment you will run for real in this chapter's demo — there at a duty of 0.20.)

The story has six stages:

1. standstill **inrush** — full headroom, current rises fast;
2. acceleration makes back-EMF grow;
3. growing back-EMF squeezes the current (and the torque with it);
4. the motor settles at the **no-load speed for that duty**;
5. the practical lesson: current is highest at *low* speed;
6. one misconception to clear up: fixed duty never "fights" back-EMF.

**Stage 1 — standstill inrush.** At rest, $\omega = 0$, so $K_e\omega = 0$: there is *no back-EMF at all*, and the entire 12 V is available as headroom to drive current. With $i = 0$ at the first instant, the equation reduces to $V = L\,di/dt$, that is,

$$
\frac{di}{dt} = \frac{V}{L},
$$

a fast initial rise of current, held back only by the winding inductance. (**Inrush** is the engineer's word for this sudden surge of current when power is first applied.) Step by step:

```text
duty command applied → voltage drive applied
        → motor stationary, back-EMF ≈ 0
        → full drive voltage available as headroom
        → current rises fast (di/dt = V/L)
        → Iq produces torque → motor accelerates
```

Now a thought experiment: if the motor were somehow held at standstill long enough for the current to stop changing ($di/dt \to 0$), the current would settle toward $i = V/R$. And since winding resistance $R$ is deliberately small, that number is *large*. This is why a big duty command applied at low speed produces a big current spike — and why, in this mode, the largest currents happen at *low* speed, not high speed (Stage 5 returns to this). In real life, the driver's built-in current limits and protections cap the actual inrush; what matters for us is the tendency.

**Stage 2 — acceleration raises back-EMF.** The inrush current is torque-producing current, so the rotor accelerates. And as $\omega$ rises, the motor increasingly behaves like a generator wired against itself: $E = K_e\omega$ grows and pushes back against your applied drive.

```text
applied drive  ───────►
               ◄───────  back-EMF (grows with speed)
```

**Stage 3 — rising back-EMF squeezes the current.** Rearrange the electrical equation to isolate what actually drives the current:

$$
\frac{di}{dt} \;=\; \frac{V - K_e\omega - iR}{L}.
$$

Look at the top of that fraction. Your duty command is fixed and the bus voltage is fixed — so $V$ is fixed. Nobody raises the drive to answer the rising back-EMF. The headroom $V - K_e\omega$ simply shrinks, and the current slides downward. Once the current is roughly steady ($di/dt \approx 0$), the balance becomes beautifully simple:

$$
\boxed{\; i \;=\; \frac{V - K_e\omega}{R} \;}
$$

Let's put numbers in, with our $V = 12$ V of effective drive:

| Motor state | $K_e\omega$ | Steady-model current |
|---|---:|---:|
| standstill | 0 V | $12/R$ |
| accelerating, mid-speed | 8 V | $4/R$ |
| near top speed for this duty | 11 V | $1/R$ |
| idealized lossless limit | ≈ 12 V | ≈ 0 |

Do you see the pattern? Every bit of speed the motor gains converts drive headroom into back-EMF — and the current, and with it $I_q$ and torque via $\tau = K_T I_q$ (Ch 2 §6), falls step by step:

$$
\boxed{\;\omega \uparrow \;\Rightarrow\; E \uparrow \;\Rightarrow\; i \downarrow \;\Rightarrow\; \tau \downarrow\;}
$$

If we sketch this tendency against speed, we get a falling line: lots of current and torque available near standstill, tapering toward zero as the speed approaches the point where back-EMF eats the whole drive:

```text
 current /
 torque
   ▲
   │\
   │ \
   │  \        fixed duty, fixed bus voltage
   │   \
   │    \
   │     \
   └──────────▶ speed
          (≈ where Ke·ω ≈ V)
```

(This is a conceptual sketch from the simplified model, not the AK80-9's measured torque–speed curve — but its shape is exactly what the demo's telemetry will draw for you in real time.)

One small but important refinement of language: the current does not *"decide"* to decrease so that the equation stays balanced. There is no little algorithm adjusting it. The applied voltage, the resistance, the inductance, and the back-EMF simply *are* the physics, and the current evolves the only way it can. The equation *describes* the motor; it is not an extra controller.

**Stage 4 — settling at the no-load speed for that duty.** Here many beginners expect the motor to slow back down: "torque is falling, so the motor must slow, right?" No — and this is worth a moment. Torque does not set speed; torque sets **acceleration** ($\tau_{\mathrm{net}} = J\alpha$). Falling torque only means *weaker and weaker acceleration*. A motor needs zero *net* torque, not zero speed, to keep cruising at a constant rate. (The full "torque causes acceleration, not speed" story is the heart of the next chapter — Ch 6 §4.)

There is one more real-world ingredient: a real motor is not lossless. Bearing friction, gearbox friction, seal friction, air drag, and electromagnetic losses all demand a small amount of torque just to keep the shaft turning. So the speed climbs until the motor's shrinking torque exactly covers those losses — and there the whole system settles:

```text
        fixed duty
             │
             ▼
    fixed drive voltage
             │
             ▼
      motor accelerates ──────────────┐
             │                        │  (this loop runs continuously
             ▼                        │   until balance is reached)
       back-EMF rises                 │
             │                        │
             ▼                        │
   current ↓ , torque ↓ ──────────────┘
             │
             ▼
  motor torque ≈ loss torque
             │
             ▼
   net torque ≈ 0 → acceleration ≈ 0
             │
             ▼
  steady "no-load speed for this duty"
```

The settling point is the **no-load speed for that duty** — a bigger duty settles at a higher speed (§6 explains this pattern). And because the loss torque is small but not zero, the settled current is also small but not zero: a little residual $I_q$ keeps paying the friction bill. On the bench, this whole stage has a very recognizable signature in Real-time Data: a current spike at the step, decaying to a small steady value while the speed flattens out. Watch for exactly this signature during the demo's steps.

**Stage 5 — why current is highest at low speed.** Now read $i = (V - K_e\omega)/R$ backwards, as a warning. Headroom is *largest at standstill* — every volt of your drive meets zero opposition — and the low winding resistance turns that headroom into large current and large torque. High speed, on the other hand, is the *low*-current, *low*-torque end of the curve.

So the risky moments in Duty Cycle Mode are starts, stalls, and any moment the shaft is slow while the duty is large. Notice that this is exactly the opposite of everyday intuition, which says "spinning faster = working harder." For this mode, slow is the hot, high-current condition. Ch 4 §6's "start small" ground rule exists for precisely this stage.

**Stage 6 — the misconception to clear up.** It is very tempting to imagine the controller *noticing* the rising back-EMF and pumping in more current to fight it, in a runaway spiral. Let's put the wrong picture and the right picture side by side:

```text
   THE MISCONCEPTION                 WHAT FIXED DUTY ACTUALLY MEANS

   speed increases                       fixed duty
        │                                     │
        ▼                                     ▼
   back-EMF increases              ≈ fixed voltage drive
        │                                     │
        ▼                                     ▼
   "controller raises current"          speed increases
        │                                     │
        ▼                                     ▼
   torque increases                    back-EMF increases
        │                                     │
        ▼                                     ▼
   motor accelerates                less headroom for current
   even harder…                               │
        ✗                                     ▼
   (not this mode)                current ↓ → torque ↓ → acceleration ↓
                                              ✓
```

The left column is exactly what a fixed duty command does **not** ask for. Fixed duty means fixed voltage effort. The controller is not holding any current target, so the rising back-EMF is simply *allowed* to squeeze the current down. Is there a mode that *does* hold the current against back-EMF — raising its voltage output as far as the bus allows to keep $I_q$ pinned to your command? Yes: that is the very next chapter (Ch 6). Here, physics wins by design.

And that is the whole arc, in one line: **inrush → acceleration → back-EMF rises → current and torque fall → settle at that duty's no-load speed** — with the biggest current at the *lowest* speed, and no hidden regulation anywhere. Every observation in this chapter's demos is one stage of this story made visible.

## 5. Negative Duty: Reverse Drive from a One-Way Supply

The sign of $D_{\mathrm{cmd}}$ selects the drive direction. That sounds simple, but it hides two very good questions: *what exactly does "negative drive" do to the shaft?* and *how can electronics powered from a positive-only supply push in reverse at all?* Let's answer both.

**Question one: what the sign means mechanically.** A negative duty commands a drive — and therefore a torque tendency — in the negative direction. It does **not** announce that the motor is currently spinning backwards. Torque direction and rotation direction are two different things, and the difference shows up clearly when we compare two starting conditions:

```text
   FROM STANDSTILL                    WHILE SPINNING FORWARD

   motor stationary                   motor rotating forward (ω > 0)
        │                                      │
        ▼                                      ▼
   D_cmd = −0.3                          D_cmd = −0.3
        │                                      │
        ▼                                      ▼
   negative drive → negative torque   negative torque opposes motion
        │                                      │
        ▼                                      ▼
   accelerates in the                 forward speed decreases…
   negative direction                 …reaches zero…
                                               │
                                               ▼
                                      …and if the command persists,
                                      accelerates in reverse
```

So a negative duty applied to a forward-spinning shaft acts first as a *brake*, and only after the speed crosses zero does it become a reverse drive. You will see this "slow — stop — reverse" sequence with your own eyes in the demo's last step. The rule to remember: the command's sign describes the *requested push direction*; the shaft's actual rotation direction follows along, with a delay. (The same torque-versus-rotation distinction comes back, in current form, in Ch 6 §3.)

**Question two: how the hardware does it.** The DC supply never reverses — it stays `+Vbus / GND` the whole time. What changes is *which motor terminal the inverter connects to the higher voltage*. That flips the sign of the line-to-line voltages ($V_{AB} = V_A - V_B$ goes from positive to negative, and so on), which reverses the direction the phase currents are pushed — the polarity trick we met in Ch 2 §3.

The easiest picture is the single-phase version: an **H-bridge** around a simple DC motor. An H-bridge is four switches arranged in an "H" shape with the motor as the crossbar, and it can connect either motor terminal to either supply rail:

```text
   forward drive:                       reverse drive:

      +                  −                 −                  +
      A ────[ MOTOR ]──── B                A ────[ MOTOR ]──── B

      current  A ──────▶ B                 current  A ◀────── B
```

For forward drive, one diagonal pair of switches is PWM-controlled, so current crosses the motor left-to-right. For reverse, the *other* diagonal is used, so current crosses right-to-left. Now here is the key point: a 20% drive strength uses a perfectly normal, positive $D_{\mathrm{PWM}} = 0.2$ in **both** cases. What differs is only *which switching arrangement* that 20% is applied to:

```text
command +0.2:  20% drive strength, forward arrangement PWM'd
command −0.2:  20% drive strength, reverse arrangement PWM'd
```

The physical ON-time fraction never went negative; the *direction of the electrical push* changed. The AK80-9's three-phase inverter is the bigger, more sophisticated cousin of this H-bridge — FOC coordinates the polarity flips across all three phases according to rotor position — but the principle is exactly the same. And that is why a signed command needs no negative power supply and no impossible negative ON-time.

## 6. Why Duty Seems to Control Speed (But Doesn't)

Try running the motor, unloaded, at a few different duty values, and you will notice a clean pattern:

```text
   small duty   →  lower steady speed
   larger duty  →  higher steady speed
```

Section 4 already explains why: each duty's fixed voltage can support rotation only up to the speed where back-EMF (plus losses) uses it all up, so a bigger voltage budget buys a higher settling speed. In fact, unloaded and warmed up, the relationship is roughly proportional — which is just the KV logic from Ch 1 §6 in action.

So… isn't that speed control? **No — and here is the one decisive reason:** the duty-to-speed relationship only holds *while the conditions hold*. Nothing in this mode measures the speed, and nothing corrects it. Change the load, the friction, or the bus voltage (§7), and the speed at the very same duty changes too — and *stays* changed:

```text
        same duty command
              │
      ┌───────┴────────┐
      │                │
   light load       heavy load
      │                │
      ▼                ▼
  higher speed     lower speed      ← nothing regulates either one
```

$D_{\mathrm{cmd}} = 0.5$ does not mean "50% of maximum speed." It means 50% voltage effort — and the speed that comes out depends entirely on the situation. You can even feel this at the bench: while the shaft spins at a fixed duty, a light touch of friction through a folded cloth visibly drags the speed down — and nothing comes to rescue it.

The best mental model for this mode is the **accelerator pedal of a car**. Holding the pedal at 20% does not command 20 km/h, and it does not command 20% of top speed. It commands a *level of drive effort* — and the resulting speed depends on the hill, the load, the headwind, and the road. Flat road, light car: fast. Uphill, heavy load: slow — at the *same* pedal position. Duty is the electrical version of that pedal: *"drive with this much effort,"* with current, torque, and speed left to the situation. The analogy is not electrically exact, but as a one-line summary of open-loop voltage drive, it is excellent — and when Ch 8 introduces the mode that behaves like *cruise control* instead, you will already feel the difference.

## 7. The Same Duty on a Different Bus

One more property to be aware of: duty is a **normalized** command. It is a *fraction* of whatever the DC bus provides — not an absolute voltage:

```text
   D_cmd = 0.5  on a 48 V bus  →  effective drive ≈ 24 V
   D_cmd = 0.5  on a 24 V bus  →  effective drive ≈ 12 V
```

Same command, half the electrical drive — and therefore a lower settling speed and less available torque. A duty value *by itself* does not fully describe what the motor receives; the bus voltage is the other half of the description. This is one more reason duty must never be read directly as a torque, a current, or a speed. Two practical consequences:

- **Always record the bus voltage together with the duty.** When you log or report duty-mode results, write down the bus voltage next to the command (Real-time Data shows it continuously). "0.3 duty" means different physics at 24 V, 36 V, and 48 V — and all three are legal supplies inside the 18–52 V allowable range (Ch 1 §5).
- **Battery-powered rigs drift.** A battery's voltage sags as it discharges (and starts high when fully charged). So a fixed duty on a battery rig delivers a slowly shrinking drive over a working session — one more way the "same" command gives different speeds, with no regulation to hide it from you.

## Demo — Step the Duty, Watch the Speed Follow

Time to put your hands on the motor. The demo follows Ch 4 §6's ground rules: motor bench-mounted and calibrated per Ch 4, output shaft **unloaded**, 48 V supply with a conservative current limit, CubeMarsTool connected with the Real-time Data panel open, and the Stop button located *before* you send anything. The manual's own preconditions for the mode apply throughout (§3.3.1.8): motor input power stable, connectors properly connected, motor in servo mode, upper computer successfully connected. Commands go into the Servo Control interface's duty field (Ch 4 §3).

The approximate speeds quoted below come from the simple no-load estimate speed ≈ KV × (duty × V_bus) / 9 (Ch 1 §6). Treat them as rough sanity checks, not exact predictions — friction pulls the real values somewhat lower, and its effect is proportionally strongest at the smallest duty.

Your first commanded motion in the whole series: step the duty up, watch the speed follow, then reverse.

**Starting condition:** calibrated per Ch 4, unloaded, shaft free to spin.

**Commands, in order, pausing a few seconds at each so the speed can settle:**

1. $D_{\mathrm{cmd}} = 0.05$ — expect slow but clearly visible rotation, very roughly 25 rpm at the output shaft ($100 \times 0.05 \times 48 / 9 \approx 27$).
2. $D_{\mathrm{cmd}} = 0.10$ — the speed roughly doubles, to around 50 rpm.
3. $D_{\mathrm{cmd}} = 0.20$ — roughly doubles again, toward ~100 rpm. This is the run recorded in Figure 5.1 and Video 5.1.
4. $D_{\mathrm{cmd}} = 0.50$ — a strong drive (≈ 24 V). The shaft climbs to roughly 265 rpm — close to a 50,000 ERPM reading. Treat this step with respect: raise the duty to 0.50 *from* 0.20, never from rest, and keep hands, hair, and cables well clear (Video 5.2 shows this run).
5. $D_{\mathrm{cmd}} = -0.10$ — from the fast forward spin, the shaft brakes, pauses at zero for a moment, and spins up in the opposite direction to about the step-2 speed (Video 5.3 shows this reversal).

**What you should see.** Each upward step gives a short burst of acceleration that fades away as the new settling speed is reached — §4's whole story, compressed into a couple of seconds. The reversal shows §5's sign logic live: negative duty first brakes the forward motion, then drives reverse rotation. Rough settled values to compare against:

| Step | $D_{\mathrm{cmd}}$ | Effective drive @ 48 V | Est. shaft speed | Est. telemetry ERPM (×189, Ch 3 §2) |
|---:|---:|---:|---:|---:|
| 1 | +0.05 | ≈ 2.4 V | ~25 rpm | ~5,000 |
| 2 | +0.10 | ≈ 4.8 V | ~50 rpm | ~10,000 |
| 3 | +0.20 | ≈ 9.6 V | ~105 rpm | ~20,000 |
| 4 | +0.50 | ≈ 24 V | ~265 rpm | ~50,000 |
| 5 | −0.10 | ≈ 4.8 V reversed | ~50 rpm reverse | ~−10,000 |

The ERPM column doubles as a live check of Ch 3's ÷189 rule: whatever shaft speed you see, the telemetry ERPM field should read about 189 times it. One practical note: the manual documents the GUI duty range as 0.005–0.95; if your CubeMarsTool build refuses a negative entry in the duty field, send the reversal over CAN instead (§2) — the protocol command is signed.



<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
          <img src="images\image6.png" 
               alt="Servo Control / General Control panel with the duty field highlighted, value 0.20 entered, and the Real-time Data speed field visible in the same frame showing the corresponding settled ERPM — the reader's reference for where the command goes and where its effect appears.">
          <figcaption>
          Figure 5.1: Servo Control / General Control panel (Right) with the duty field highlighted, value 0.20 entered, and the Real-time Data (Left) speed field visible in the same frame showing the corresponding settled ERPM — the reader's reference for where the command goes and where its effect appears.
          </figcaption>
     </figure>
  </figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
     <figure style="text-align: center; margin: 0;">
    <video src="videos\video1.mp4" width="640" height="360" controls></video>
     <p style="width: 640px; text-align: center; margin-top: 4px;">
     <i>Video 5.1: Output Shaft of the Motor rotating at Duty-Cycle 0.2.</i>
     </p>
  </figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
     <figure style="text-align: center; margin: 0;">
     <video src="videos\video2.mp4" width="640" height="360" controls></video>
     <p style="width: 640px; text-align: center; margin-top: 4px;">
     <i>Video 5.2: Output Shaft of the Motor rotating at Duty-Cycle 0.5.</i>
     </p>
  </figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
     <figure style="text-align: center; margin: 0;">
     <video src="videos\video3.mp4" width="640" height="360" controls></video>
     <p style="width: 640px; text-align: center; margin-top: 4px;">
     <i>Video 5.3: Output Shaft of the Motor rotating at Duty-Cycle -0.1.</i>
     </p>
  </figure>
</div>

## When to Use Duty Cycle Mode — and When Not

A fair question at this point: the later modes will regulate current, speed, and position *for* you — so why would anyone ever command raw voltage effort? The answer: sometimes the *absence* of regulation is exactly what you want. Either the application genuinely doesn't need it, or a controller of your own is supplying the regulation from outside:

```text
        Duty Cycle Mode
              │
              ▼
   command electrical drive level
              │
              ▼
   current, torque and speed result
     from the operating conditions
     (no internal loop shapes them)
```

Five situations where that trade makes sense:

**1. Simple open-loop drive.** Some mechanisms only need "run slowly this way, run harder, stop, run the other way" — and the exact speed or torque genuinely does not matter. For those, a handful of duty values (±0.05, ±0.2, 0) is the simplest possible interface: no gains to tune, no targets to manage.

**2. Testing and commissioning.** A small duty command is the fastest way to answer the first-contact questions about a new setup: does the motor rotate at all? Is the direction convention what you expect? Does the encoder feedback track? Does the mechanism move freely? How do current and speed respond as the drive rises? This series itself uses Duty Cycle Mode as your very first spin for exactly this reason. But increase the duty *slowly* here — §4 Stage 5's low-speed current warning matters most at commissioning time, when surprises are likeliest.

**3. Your own controller, outside the drive.** Maybe you are building your own control loop — your own speed or force controller, running on your own hardware. Duty is the natural low-level output for it: your algorithm measures, computes, and commands the drive effort, while the AK80-9 quietly handles commutation, FOC, and switching underneath. This gives your controller lower-level authority than handing the drive a ready-made velocity target. (That said, for many applications commanding *current* instead is easier and safer, because the inner current loop then manages the motor's electrical dynamics for you — Ch 6.)

**4. Characterization and system identification.** Step through a set of duties $D_1, D_2, D_3, \ldots$ while logging $I_q$, speed, and bus voltage, and you map out how the motor-plus-mechanism responds to drive level. This is valuable raw material for system identification, friction estimation, and controller design — valuable precisely *because* no internal loop is shaping the response you are trying to measure.

**5. When precise regulation is simply not required.** A roller, a fan, or an agitator that just needs to spin "about this fast" under predictable conditions can live happily on one fixed duty. The caveat is §6's: if the load changes, the speed changes with it — and stays changed. The moment your requirement becomes "*maintain* this speed even when the load varies," this is the wrong mode.

**The nearest-neighbor contrast, in three lines.** Duty Cycle Mode commands voltage effort and lets back-EMF squeeze the current down as speed rises; Current Loop Mode commands the current itself, and its inner loop raises the voltage as far as the bus allows to hold $I_q$ against back-EMF. One mode lets physics set the torque; the other pins it. The full treatment is Ch 6 (§4 for the core behavior), and the systematic mode-by-mode comparison — including duty vs. current vs. velocity head-to-heads — is Ch 12.

## Where These Ideas Go Next

Duty Cycle Mode is the reference point that the rest of Part II is measured against:

| Established here | Continues in |
|---|---|
| Back-EMF rises with speed and limits current; voltage headroom (§3–4) | Ch 6 §4 (why constant current can't accelerate forever), Ch 8 §6 (why a too-high speed target can't be reached) |
| Torque → acceleration, previewed in one line (§4 Stage 4) | Ch 6 §4 — the full treatment |
| Sign of command = push/torque direction ≠ rotation direction (§5) | Ch 6 §3 (signed $I_q$), Ch 7 §3 (unsigned brake command) |
| "Nothing regulates the speed": a slowdown is never corrected (§6) | Ch 8 §4 (the same disturbance, *with* recovery) |
| Open-loop voltage effort as one end of the control spectrum | Ch 12 (choosing a mode; all head-to-head comparisons) |

## Key Takeaways

- Duty Cycle Mode (Servo ID 0) commands a **signed voltage-drive level**, $-0.95 \le D_{\mathrm{cmd}} \le +0.95$ — magnitude = effort, sign = direction. Nothing mechanical is regulated: $D \to V \to I_q \to \tau \to \alpha \to \omega$, open-loop all the way.
- The signed command is not a physical PWM duty ratio: $D_{\mathrm{PWM}} = t_{\mathrm{ON}}/T_{\mathrm{PWM}}$ always stays in 0–1; the sign only selects which switching arrangement is modulated (§1, §5).
- GUI range 0.005–0.95 (default); CAN payload int32 = duty × 100,000, signed (manual §3.3.1.8, §4.1.1).
- One equation runs the chapter: $V = iR + L\,di/dt + K_e\omega$. The headroom $V - K_e\omega$ is what drives the current; at steady state, $i = (V - K_e\omega)/R$.
- The duty-step story, told once for the whole series: **inrush at standstill → acceleration → back-EMF rises → current and torque fall → settle at that duty's no-load speed** — with current highest at *low* speed, and a fixed duty never raising current to fight back-EMF.
- Duty only *looks like* speed control: unloaded, the settling speed tracks the duty — but any change in load or bus voltage moves the speed, and nothing corrects it. Think accelerator pedal, not cruise control.
- Same duty, different bus, different physics: duty is a normalized fraction, so always record the bus voltage together with your duty-mode results.

## Safety Notes

- **Low speed is the high-current condition in this mode.** Starts, stalls, and slow shafts under a large duty draw the most current and produce the most torque (§4 Stage 5). Increase duty gradually from small values, and never apply a large duty to a stalled or blocked shaft.
- Duty Cycle Mode regulates nothing — no speed, current, or position target will catch a runaway situation for you. Keep commands small, keep the shaft unloaded, and keep CubeMarsTool's Stop button and the supply switch within reach (Ch 4 §6).
- If you try §6's friction touch, apply it only through a pad or cloth — never bare fingers on a spinning shaft — and keep hands, hair, and clothing clear of the rotating marker and shaft.
- Watch Real-time Data's current and temperature fields during all duty experiments, and stay well inside Ch 1 §2's ratings (12 A continuous / 28 A peak) — protocol encodings are not permissions (Ch 2 §9).

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §4.1 "Servo Mode Control Modes and Description" (p. 31: Duty Cycle Mode defined as "a specified duty cycle voltage… similar to square wave drive form"; Control Mode ID 0; extended-frame ID layout); §3.3.1.8 "Duty Cycle Mode" (p. 28: GUI operation and the default 0.005–0.95 duty range); §4.1.1 "Duty Cycle Mode" (p. 32–33: data-transmission definition — 4-byte payload, duty bits 1–32; the `comm_can_set_duty` example routine with int32 = duty × 100,000 scaling; `CAN_PACKET_SET_DUTY = 0` in the CAN packet enum).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — KV100, 9:1 reduction, and ratings used for the demo speed estimates and safety limits, via this series' reference table (Ch 1 §2) and unit conversions (Ch 3 §2).
3. **CubeMarsTool upper-computer software** — Servo Control duty field and Real-time Data plots shown in Figure 5.1; © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.