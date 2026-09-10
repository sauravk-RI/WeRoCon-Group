# Chapter 7 — Current Brake Mode

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 8 — Velocity Loop Mode →](08_Velocity_Loop_Mode.md)

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In Chapter 6, you learned how to command torque directly. You typed a current value with a **sign** (+ or −), and the sign chose the direction of the push. In this chapter, we meet a close cousin of that mode: **Current Brake Mode**, which is Servo Control Mode ID 2.

Here is the big difference. In Current Brake Mode, you do **not** choose the direction. You only type a number — a braking-current value — and the controller chooses the direction by itself. It always pushes **against** the motion of the shaft. So the motor slows down, stops, and then actively resists anything that tries to move it again.

Why does this simple mode deserve a whole chapter? Two reasons.

First, there is a very interesting question hiding inside it: *why is there no negative brake command?* The answer teaches us something useful about how control modes are designed.

Second, braking naturally leads to **holding** — keeping a stopped shaft in place. And holding is where this chapter does its most important job for the whole series. Section 5 explains, once and for all, why **zero speed does not mean zero current, and zero current is not needed for zero speed to make heat** — in short: **"zero speed ≠ zero current ≠ zero heat."** Every later chapter where the motor holds a position against a load — Position Mode (Ch 9), Position–Velocity Mode (Ch 10), and MIT Mode (Ch 11) — will simply point back here instead of explaining it again.

Here is the plan for this chapter:

- what Current Brake Mode is, its command range, and the manual's temperature warning (§1);
- why braking is still just torque, and why braking does not mean spinning backwards (§2);
- how the controller chooses the torque direction — and why the command has no sign (§3);
- what happens after the shaft stops, with and without a load (§4);
- the full story of zero speed ≠ zero current ≠ zero heat (§5);
- why this is not a mechanical lock, and what happens if the load is too strong (§6);
- two short reminders with pointers: brake current vs supply current (§7), and yes, FOC is still working underneath (§8);
- where to use this mode, its limits, and how to use it safely (§9);
- and short pointer comparisons with the neighboring modes (§10);

followed by a bench demo, a short "when to use it" guide, safety notes, and sources. As always, we assume your motor is connected and calibrated, and that you follow the safety ground rules from Ch 4 §6.

## 1. What Current Brake Mode Is

The manual describes this mode in one sentence (§4.1, p. 31): *"A specified braking current is given to the motor to hold it in the current position (pay attention to motor temperature when using)."* Using it in the software is also just one step (§3.3.1.7, p. 27): in the Servo Control screen of CubeMarsTool, *"enter the desired braking current B, and the motor will brake with the desired current."*

So: you type a number in the **B field**, in amps, and the motor brakes.

At the CAN protocol level, the message is called `CAN_PACKET_SET_CURRENT_BRAKE` (you can find it in the packet list on p. 32 of the manual). The number is sent the same way as in Ch 6: an int32 value equal to the current × 1000. But look carefully at the allowed range (§4.1.3, p. 33–34): the values go from **0 to 60,000, meaning 0 to 60 A**. Compare this with Current Loop Mode:

| | Current Loop (ID 1) | Current Brake (ID 2) |
|---|---|---|
| What you command | signed $I_q$ (+ or −) | a braking current — **size only, no sign** |
| Protocol range | −60 A … +60 A | **0 A … 60 A** |
| Who chooses the torque direction | **you** (with the sign) | **the controller** |

Notice: there is no negative brake command. This is not a mistake in the protocol. It is actually a big clue about how the mode works, and we will explain it fully in §3.

If you want to command this mode from your own CAN code instead of the GUI, the manual gives a short example function (p. 34). It is almost the same as Ch 6's, only the packet ID changes:

```c
void comm_can_set_cb(uint8_t controller_id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id |
        ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}
```

One honest note before we continue. In Current Loop Mode, the manual clearly says the command is $I_q$, the torque-producing current. Here, the manual only says "braking current" — it does **not** call it $I_q$. So we should treat the brake command as a "braking effort" number that the controller converts into motor current internally. In this chapter, we will sometimes estimate the resulting torque using Ch 2 §6's constant (torque ≈ 0.5701 × current, in N·m). Those numbers are good *estimates* for planning your experiments — just remember they are not an exact promise from the datasheet.

Two quick reminders from earlier chapters (two lines each, as promised):

- The 0–60 A range is only what the *protocol can encode* — it is **not** permission. The motor's real ratings are still 12 A continuous and 28 A peak (Ch 2 §9).
- The number you command here is **not** the number your bench power supply will show (§7 below; the full explanation is in Ch 2 §2).

One last thing. Look again at the manual's one-line definition. It ends with a warning: *"pay attention to motor temperature when using."* CubeMars puts this warning on **this** mode and on no other Servo mode. By the end of §5, you will understand exactly why.

## 2. Braking Is Still Torque — and Braking Is Not Reverse Rotation

When you switch to Brake Mode, nothing inside the actuator turns into a friction brake. There are no brake pads. The motor keeps doing the only thing a motor can do: it produces **electromagnetic torque** (Ch 2 §6).

So what makes a torque a "braking" torque? Only its **direction compared to the motion**. "Driving torque" and "braking torque" are not two different kinds of torque. They are the same thing, just pointing different ways:

| Shaft motion | Torque direction | Result | We call it |
|---|---|---|---|
| rotating → | → (same direction) | speed goes up | driving torque |
| rotating → | ← (opposite direction) | speed goes down | braking torque |

The physics is the same equation we used in Ch 6 §4, just with the torque pointing the other way. When torque opposes the motion, $\tau_{\mathrm{net}} = J\alpha$ gives a negative $\alpha$ (angular acceleration), so the speed falls.

Now, a very common doubt for beginners:

> *"If the torque pushes against the rotation, won't the motor just start spinning backwards?"*

Not immediately — and in this mode, not at all. An opposing torque first **slows the shaft down**. The speed gets smaller and smaller, until it reaches zero. Only if a reversing torque *kept pushing past zero* would the shaft start turning the other way:

```text
forward rotation → braking → ω = 0 → (only if torque kept pushing:) reverse
                   └───── Brake Mode works here ─────┘
```

Current Brake Mode is designed to do the bracketed part and then **stop the story at zero speed**. Once the shaft is stopped, the mode's goal changes to *keeping* it stopped (§4). It never tries to drive the shaft onward in reverse.

(You may remember Ch 6 Demo 2, where the motor braked, passed through zero, and reversed. That happened because in Current Loop Mode, *your* signed command kept pushing past zero. In Brake Mode, the controller has no such plan.)

## 3. Who Chooses the Direction — and Why There Is No Negative Command

Here is the key question of the whole chapter:

> *If I never tell the controller which direction to push, how does it know?*

The answer: it already knows everything it needs. The actuator has an encoder that constantly measures the shaft's position and velocity — the FOC system from Ch 2 §5 depends on these measurements anyway. So the controller always knows which way the shaft is turning right now.

The braking rule is then just common sense. If the measured velocity is positive ($\omega > 0$), the shaft is turning in the positive direction, so braking needs **negative** torque. If $\omega < 0$, the shaft is turning the other way, so braking needs **positive** torque. There is nothing left for you to decide:

```text
        measured ω
            │
     ┌──────┴──────┐
     │             │
   ω > 0         ω < 0
     │             │
     ▼             ▼
 apply torque   apply torque
   opposing       opposing
  (negative)     (positive)
```

You supply the **"how hard"** (the size of the current). The measurement supplies the **"which way."** This division of work is exactly why the command range starts at zero.

Compare with Ch 6. There, the signed command exists because only *you* know what you want — maybe accelerate clockwise, maybe counter-clockwise, maybe brake, maybe reverse. The controller cannot guess your intention, so you must give it the sign.

In Brake Mode, your intention is already fixed by the mode itself: *oppose whatever motion exists*. A sign would add nothing. (Think about it: what would "+5 A of braking" even mean on a shaft spinning forward? There is only one direction that brakes it.) So the interface itself tells you the mode's purpose:

> **Current Loop Mode gives you a signed torque dial: "push this hard, in this direction." Current Brake Mode gives you an unsigned effort dial: "oppose motion, this hard." The controller handles the sign, because for braking, the sign is never a real choice.**

Here is a small lesson you can carry to other motor drivers too: when a command range is one-sided (only 0 and up), the controller is quietly telling you that it has taken the direction decision for itself.

## 4. After the Shaft Stops: Resting, or Actively Holding

The braking works, and the shaft reaches $\omega = 0$. What happens next? It depends on the mechanical situation.

**Case 1 — no external load.** Imagine the shaft was spinning freely with nothing attached. After braking stops it, nothing is trying to move it anymore. Opposing motion that does not exist needs almost no effort. So the mode just sits quietly, drawing only a small current and producing almost no torque. (The demo ends exactly in this state — watch the current reading after the stop.)

**Case 2 — a load keeps pushing.** Now imagine a robot-arm joint with gravity pulling the arm down, or a winch with a weight hanging from it. The shaft stops — but gravity never stops. The moment the shaft starts to move even a little, the controller sees motion and pushes back. In practice, this means the motor **leans against the load continuously**, producing whatever opposing torque is needed (up to the strength of your commanded current) to keep the shaft pinned in place. From outside, the joint simply looks *held*.

The two cases, side by side:

| At $\omega = 0$ | Is anything pushing? | Torque produced | Current drawn | Heat |
|---|---|---|---|---|
| No load (Case 1) | no | almost none | small | almost none |
| Load present (Case 2) | yes, continuously | continuous opposing torque | continuous holding current | continuous — see §5 |

Now you can see why the manual's definition says "hold it in the current position." Braking and holding are really the **same behavior** — oppose motion — seen before and after the speed reaches zero. And you can also guess why the same sentence warns about temperature. That brings us to the most important section of this chapter.

## 5. Zero Speed ≠ Zero Current ≠ Zero Heat

*(This section is the series' one official home for holding, current, and heat. Chapters 9, 10, and 11 will point here instead of repeating it.)*

Many beginners have this intuition:

> *"The shaft is not moving, so the motor is not doing anything, so it cannot be using power, so it cannot get hot."*

This sounds very reasonable — and it is wrong. Let us break the chain link by link, because each link teaches something.

**Zero speed is true — and the mechanical output power really is zero.** Mechanical power is torque times speed: $P_{\mathrm{mech}} = \tau\omega$. When the shaft is holding still, $\omega = 0$, so $P_{\mathrm{mech}} = 0$ exactly. The motor is doing no mechanical work on the world, no matter how hard it is pushing. So far, the intuition is correct.

**But zero speed does not mean zero current.** Remember Case 2 from §4: holding a load means producing torque *continuously*. And producing torque requires current — the relationship $\tau_{\mathrm{out}} \approx 0.5701\,I_q$ (Ch 2 §6) does not stop working just because the shaft stopped moving.

Let us put a number on it. A joint holding about 2.85 N·m against gravity needs roughly 5 A flowing in its windings — while a person watching from outside sees a motor that appears to be doing nothing at all. The current is there to make the *torque*, and the load demands that torque whether the shaft moves or not. Speed was never part of the deal.

**And current always makes heat — because speed does not appear in the loss equation.** The motor windings are made of copper wire, and copper wire has resistance $R$. Current flowing through resistance produces heat:

$$
P_{\mathrm{Cu}} = I^2 R .
$$

Look carefully at this equation. There is **no speed in it anywhere**. So even at $\omega = 0$, the motor dissipates heat. In fact, at standstill the situation is worst in two ways: none of the electrical power is leaving as useful mechanical work, and the motor is not spinning any cooling air past itself either. A motor holding a load while standing still is, from a thermal point of view, basically a resistor with a shaft attached.

The full chain, worth memorizing:

```text
load keeps pushing → holding torque needed → winding current needed
      → I²R copper loss → heat, continuously, at zero speed
```

**Why bigger currents are much worse, not just a little worse.** The loss grows with the *square* of the current. That means doubling the current makes four times the heat:

| Holding current | Copper loss (relative) | |
|---:|---:|---|
| 2 A | $2^2 = 4$ | our baseline |
| 5 A | $5^2 = 25$ | 2.5× the current → **6.25×** the heat |
| 10 A | $10^2 = 100$ | 5× the current → **25×** the heat |

This is why choosing a big brake current "just to be safe" is actually the *opposite* of safe. It is why CubeMars attaches its temperature warning to this mode and no other. And it is why you should never leave a strong hold running unattended. The rule is simple: **command the smallest current that actually holds your load, and keep an eye on the temperature reading during any long hold.** (Keep the temperature field open during any hold you try, and you will watch this happen with your own eyes.)

One final note about scope. Nothing in this section is special to Brake Mode. *Any* mode that keeps producing torque at standstill pays the same $I^2R$ bill — a Position Mode hold fighting gravity (Ch 9), a stiff MIT hold (Ch 11), or Ch 6 §5's blocked shaft. Brake Mode is simply where our series faces this physics directly, because for this mode, holding is the main purpose, not a side effect.

## 6. Not a Lock: What Happens When the Load Is Too Strong

Think about a mechanical brake — a bicycle brake, or the parking pawl in a car. It locks the wheel using friction or metal parts, and it **stays locked even with no power**. Current Brake Mode is completely different. It is an **active electronic brake**: the holding torque exists only while the driver is powered and producing it.

```text
mechanical brake:   power removed → still locked
Current Brake Mode: power removed → holding torque gone, instantly
```

Cut the power, and a gravity-loaded arm falls immediately. So remember: if your machine must stay in place even during a power failure, it needs a real mechanical brake or a self-locking gearbox. No software command can replace that.

The hold is also **not infinitely strong**. It is limited by whichever limit is reached first: the current you commanded, the actuator's torque capability (Ch 1's ratings), and heat (§5). The mathematics of an overload is one line. Say the motor can produce 8 N·m of opposing torque, and the load pushes with 5 N·m: the motor wins, and the shaft stays still. Now increase the load to 15 N·m: the motor still pushes with its full 8 N·m — but the extra 7 N·m belongs to the load, and the shaft turns. The mode does not fail with an alarm. It simply loses the pushing contest, while resisting all the way.

There is one more thing this mode does **not** do, and it surprises many people. When the shaft is forced away from its place, nothing inside the controller remembers where the shaft *used to be*. No error builds up. Nothing pulls it back. Brake Mode only ever asks *"is the shaft moving?"* — never *"where should the shaft be?"*

Here is the concrete version, which you can try yourself right after the demo: brake the shaft to a stop, then twist it away by hand (gently, by the flag). The mode resists your twist the whole way. But the moment you let go, it **leaves the shaft exactly where you left it**, and holds the new angle just as happily as the old one. A Position Mode hold in the same situation would drive the shaft straight back to its target (Ch 9). So do not think of Brake Mode as "Position Mode without a target" — it has a genuinely different goal. If "come back to where it was" is a requirement for your application, you need a position target instead (see §10).

## 7. Brake Current Is Not the DC Supply Current

A two-line reminder of a rule from Ch 2: commanding B = 5 A does **not** make your bench power supply display 5 A. The supply measures the actuator's DC input current $I_{\mathrm{in}}$, which depends on bus voltage, losses, and (while braking a moving shaft) even energy flowing *back* from the motor. The brake command is an internal control number. In general, $I_{\mathrm{in}} \neq I_{\mathrm{brake}}$ — the full story is in Ch 2 §2 and §10.

## 8. Yes, FOC Is Still Working Underneath

Same inverter, same three-phase windings, same FOC current control (Ch 2 §§3–5) — none of the motor-control machinery changes in this mode. The only difference is *who writes the torque request* that FOC then carries out. In Ch 6, you wrote a signed $I_q$ yourself. Here, the brake logic writes it for you: it takes the **size** from your command and the **sign** from the measured velocity (§3). CubeMars does not publish the deeper implementation details, and nothing in this chapter depends on them.

## 9. Where to Use It, Its Limits, and How to Use It Safely

**Where this mode fits.** Reach for Current Brake Mode when your requirement is, in plain words, *"stop, and/or stay put, firmly, for a while."* Some typical situations:

**1. Controlled stopping.** Wheels, conveyors, spindles, and other mechanisms that must come to rest on command. Instead of letting them coast down slowly on friction, you get a repeatable, adjustable stopping effort (the demo). One number chooses between a gentle stop and a hard stop.

**2. Park-and-hold under load.** A robot-arm joint that finished its motion and must not sag under gravity. A winch pausing in the middle of a lift. A lift table waiting between floors. Instead of keeping a full position controller running, you command one brake value sized for the load, and the joint stays leaned-against and pinned (§4, Case 2) — simply and cheaply. (When a position hold, with its "come back to target" behavior, is worth the extra complexity is discussed in Ch 12, scenario (d).)

**3. Holding back a descent.** A mobile robot going down a ramp, or any mechanism that gravity would make run away, is this mode's job description written out in full: gravity creates motion, the controller opposes it, and your commanded current sets how strongly.

**4. Temporary holding inside a bigger sequence.** "Hold here while the rest of the machine does something else" is a very common state in real systems. This mode implements it with the least machinery of any option: no target, no motion profile, one unsigned number, and the hold engages by itself the moment anything pushes.

**Where it does not fit.** The limits are the mirror image of everything above: it is **not a lock** (power off = hold off, §6); its strength is **finite** (§6); a long or heavy hold has a **heat cost** that grows with current squared (§5); and it is **not position control** — a displaced shaft stays displaced, with no memory and no return (§6). If any of these four is a problem for your application, look at the neighboring modes in §10.

**Safe habits for this mode:** command the smallest current that does the job; treat every hold as a heating process with a time limit; keep the temperature field of Real-time Data visible during holds; and never let a powered hold be the only thing between a load and the floor.

## 10. Nearest Neighbors: Four Short Pointers

**vs. Current Loop Mode (Ch 6).** Signed torque that *you* aim, versus unsigned effort that the *controller* aims against motion (§3). Same current control underneath, opposite ownership of the direction — full comparison in Ch 12.

**vs. Velocity Loop Mode (Ch 8, coming next).** Velocity Mode maintains motion at a target speed, adding torque when a load slows it down. Brake Mode has no speed target except, in effect, zero. One mode defends motion, the other opposes it — see Ch 12.

**vs. Position Mode (Ch 9).** Both can keep a shaft still, but Position Mode asks *"where should the shaft be?"* and drives any error back to the target, while Brake Mode asks only *"is it moving?"* and holds wherever it stands (§6). Which hold to choose, and when, is Ch 12 scenario (d).

**vs. MIT Torque Control (Ch 11).** MIT torque is a signed N·m command — you choose the direction, like Ch 6 but in torque units. Brake Mode chooses the braking direction automatically. The full Servo-vs-MIT picture is in Ch 11 §14 and Ch 12.

## Demo — Braking a Spinning Shaft

The demo follows Ch 4 §6's ground rules: motor bench-mounted and calibrated per Ch 4, output shaft **unloaded**, 48 V supply with a conservative current limit, and CubeMarsTool connected with the Real-time Data panel open and the Stop button located. The manual's preconditions for this mode apply throughout (§3.3.1.7): motor input power stable, connectors properly connected, upper computer successfully connected.

Brake commands go into the **B field** of the Servo Control screen, in amps. The demo also uses Ch 6's **I field** to spin the shaft up first. Commanding **B = 0 ends a brake or hold**, and the shaft becomes free again. The values we use (0.5–3 A, which is roughly 0.3–1.7 N·m of opposing torque using Ch 2 §6's estimate, with §1's caution) are deliberately modest. One more warning: any *holding* you try afterwards is a *heating* process. Keep the temperature field in view, and keep every hold short.

This is the mode's signature trick: a freely spinning shaft, one unsigned command, one sharp commanded stop — with the direction chosen entirely by the controller.

**Starting condition:** calibrated per Ch 4, unloaded, shaft marked with a tape flag or marker collar, Real-time Data plotting speed and current.

**Commands and expected observations:**

1. First, spin the shaft up exactly like in Ch 6 Demo 1: command $I_q = 1$ A in the **I field** and let the motor accelerate to a fast, steady, clearly visible spin. Expect a couple of hundred rpm at the shaft and an ERPM reading in the tens of thousands — on our bench it settles at the steady 38,073 ERPM ≈ 200 rpm you can see in Figure 7.1 and Video 7.1 (divide by 189 to get shaft rpm, Ch 3 §2).
2. Now command **B = 2 A** in the brake field. The shaft slows down hard and stops within a moment. Compare this with the slow, lazy coast-down you saw when Ch 6's demos ended with 0 A — the difference is very visible. About 1.1 N·m of opposing torque did this, and you never told the controller which way to push.
3. After the stop, look at the telemetry: with nothing trying to move the shaft, the current settles to a small value. This is §4's Case 1, live: the mode is "holding" a shaft that needs no holding.
4. Command B = 0, spin the shaft up again, and repeat the stop with **B = 0.5 A** and then **B = 3 A**. The same motion ends gently the first time, and very sharply the second. The number really is a "braking effort" dial.
5. Optional, but worth doing once: spin the shaft up in the *opposite* direction (I field, −1 A) and brake with the same unsigned B = 2 A. You get an identical stop, made by torque in the opposite direction, with zero change to your command — §3 demonstrated in one move.

A quick check of what was commanded versus what you saw:

| Quantity | Did you command it? | What you observe |
|---|:---:|---|
| Braking effort | **yes** — B = 0.5 / 2 / 3 A | gentle → hard → very sharp stop; a real effort dial |
| Torque direction | **no** | always opposes the motion, in either spin direction (step 5) |
| Current after the stop | no | drops to a small value on the unloaded shaft (§4, Case 1) |
| Final position | no | wherever the stop happened to finish — no target exists (§6) |

One thing you cannot see directly: the controller's direction decision happens inside the firmware. Its only visible trace is that the same unsigned command stops motion in either direction (step 5).


<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image8.png" 
          alt="Servo Control Panel for Motor spinning at constant speed 38073 ERPM">
      <figcaption>
        Figure 7.1: Servo Control Panel for Motor spinning at constant speed 38073 ERPM. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video6.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 7.1: Motor spinning at constant speed 38073 ERPM</i>
      </p>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
     <figure>
      <img src="images\image9.png" 
          alt="Servo Control Panel for applying Braking Current of 2A">
      <figcaption>
        Figure 7.2: Servo Control Panel for applying Braking Current of 2A. <i>Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.</i>
      </figcaption>
    </figure>
</figure>
</div>

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
      <video src="videos\video7.mp4" width="640" height="360" controls></video>
      <p style="width: 640px; text-align: center; margin-top: 4px;">
      <i>Video 7.2: Suddenly applying braking torque of 2A while the motor is spinning at constant velocity of 38073 ERPM</i>
      </p>
</figure>
</div>

## When to Use Current Brake Mode — and When Not

Use it when your requirement is simply *oppose motion*: a commanded, repeatable stop of a moving mechanism; parking and holding a loaded joint, winch, or lift between motions; holding back a gravity-driven descent; any temporary, firm "stay put" inside a larger sequence. Its strengths are simplicity (one unsigned number), an adjustable effort dial (the demo's 0.5 / 2 / 3 A steps), and a hold that engages automatically the instant anything pushes (§4).

Avoid it when any of its four limits gets in your way: if the hold must survive a power loss, you need a mechanical brake (§6); if the load can be stronger than the motor, the shaft *will* move (§6); if the hold is heavy and long, the heat bill (growing with current squared) may be too high (§5); and if the shaft must *come back* after being pushed away — or must reach a particular angle in the first place — you want a position target, Ch 9/10, not a brake. The full decision framework across all the modes, including the specific "park-and-hold vs Position-hold" question, is in Ch 12.

## Where These Ideas Go Next

| Established here | Continues in |
|---|---|
| **Zero speed ≠ zero current ≠ zero heat** (§5) | Ch 9 §5, Ch 10 §6, Ch 11 §7 — every later holding discussion points back here |
| Holding = continuous torque against a persistent load (§4) | Ch 9 (position holds), Ch 11 (MIT stiffness holds) |
| An active hold is not a mechanical lock; power off = hold off (§6) | one-line reminders wherever later modes hold loads |
| An unsigned command means the controller owns the direction (§3) | Ch 11 §9 (MIT torque as the signed counterpart), Ch 12 (how a mode's interface reveals its intent) |
| No position memory: "is it moving?", never "where should it be?" (§6) | Ch 9 §1's contrast, Ch 12 scenario (d) |
| Brake value ≈ torque via 0.5701, with the "not labelled $I_q$" caution (§1) | Ch 12's comparison of the three "current-flavored" commands |

## Key Takeaways

- Current Brake Mode (Servo ID 2) takes a braking-current **size** with no sign, 0–60 A at the protocol level (`CAN_PACKET_SET_CURRENT_BRAKE`, int32 = A × 1000). The motor brakes moving shafts and holds stationary ones. The manual itself flags motor temperature in the mode's one-line definition.
- Braking is ordinary electromagnetic torque, pointed against the existing motion — and it slows the shaft to zero; it does not reverse it.
- There is no negative command because the sign is never yours to choose: the controller reads the encoder's velocity and opposes it (§3). A one-sided command range means the controller owns the direction.
- Stopped with no load, the mode rests quietly; stopped under load, it holds by leaning against the load continuously (§4).
- **The series' cornerstone, owned by this chapter:** $P_{\mathrm{mech}} = \tau\omega = 0$ at standstill, but holding torque requires current, and $P_{\mathrm{Cu}} = I^2R$ has no speed in it — so a holding motor heats up continuously while appearing to do nothing, and the heat grows with the *square* of the current (2 A → 10 A means 25× the heat).
- It is an active brake, not a lock: power off means hold off; its strength is finite; and a shaft that is forced away stays away — no memory, no return (§6).
- The brake command is not the supply current (Ch 2 §2); FOC still runs everything underneath, with the brake logic writing the torque request (§8); and the manual does not label the command $I_q$, so torque estimates via 0.5701 N·m/A are estimates (§1).

## Safety Notes

- **Every hold is a heating process with a time limit.** Watch the Real-time Data temperature field during any hold, keep holds short, and never leave a powered hold unattended — least of all as the only thing suspending a load.
- **Command the smallest value that holds.** Heat grows with the square of the current, so "extra margin" in the B field mostly becomes extra watts in the windings (§5). Stay within 12 A continuous / 28 A peak regardless of the 60 A encoding (Ch 2 §9), and far below both while learning.
- **A held load is one power failure away from a falling load** (§6). Set up every holding experiment — and every real application — so that a sudden loss of holding torque drops nothing onto people, fingers, or hardware.
- If you ever test the "load wins" boundary (§6), use only small weights and short levers. A load that wins the pushing contest accelerates away with the motor still resisting.
- Touch a powered or holding shaft only through a pad, cloth, or lever (Ch 4 §6) — a holding shaft carries torque even though nothing moves.
- Braking a spinning shaft is a sudden event: brake unloaded while learning (as in the demo), and expect the sharp stop to jerk anything attached to the shaft.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §4.1 "Servo Mode Control Modes and Description" (p. 31: Current Brake Mode defined as "a specified braking current is given to the motor to hold it in the current position (pay attention to motor temperature when using)"; Servo Control Mode ID 2 among the seven mode IDs; extended-frame ID layout); §4.1.3 "Current Brake Mode" (p. 33–34: data-transmission definition — int32 payload = current × 1000, values 0…60,000 ↔ 0…60 A; the `comm_can_set_cb` example routine); the CAN packet enum (p. 32: `CAN_PACKET_SET_CURRENT_BRAKE`); §3.3.1.7 "Braking Loop Mode" (p. 27: GUI operation — "enter the desired braking current B, and the motor will brake with the desired current"); §3.3.1.3 "Braking Mode" (p. 25: the separate GUI `T` braking-by-torque field, cited for disambiguation via Ch 6 §8); §4.1.2 "Current Loop Mode" (p. 33: the signed −60…+60 A range, cited for §1/§3's contrast).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — rated/peak currents 12 A / 28 A and the torque constants 0.095 N·m/A (motor side) / 0.5701 N·m/A (output side) used for this chapter's torque estimates, via this series' reference table (Ch 1 §2) and their definitive treatment in Ch 2 §6 and §9.

---

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 8 — Velocity Loop Mode →](08_Velocity_Loop_Mode.md)
