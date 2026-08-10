# Chapter 1 — Meet the AK80-9: What It Is and What Its Ratings Mean

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

Welcome! Before we start controlling the **CubeMars AK80-9 V3.0 KV100**, we should first understand two things: what this device actually is, and what the numbers on its specification sheet really mean.

If you open the datasheet, you will see terms like *rated torque*, *peak torque*, *rated speed*, *no-load speed*, *rated current*, and *peak current*. These terms sound similar, but they describe very different things. If we misunderstand them, we can easily pick the wrong motor for a project, push a motor harder than it can handle and break it, or overheat it.

Don't worry — this chapter does not need any electrical engineering background. Our only goal here is to answer one simple, practical question:

> **What does this number actually mean to me when I am using the motor?**

Here is what we will cover, step by step:

- what the actuator physically is — the five parts inside it, and its three external connections (§1);
- the ratings table — this will be our reference table for the whole series (§2);
- rated vs peak values, for both torque and current (§3);
- rated speed vs no-load speed, and why we should not mix them up (§4);
- supply voltage: 48 V nominal, the 18–52 V allowable range, and a warning about batteries (§5);
- the 9:1 gearbox, and what "KV100" actually means (§6);
- a short preview of the two torque constants (§7);
- and four everyday situations that tie everything together (§8).

Nothing in this chapter requires powering the motor. The first power-up, calibration, and the safety rules come in Ch 4. If you are curious about the electrical details behind these numbers, that is Ch 2. The different units the motor uses (like ERPM) are explained in Ch 3.

## 1. What the AK80-9 Actually Is

The AK80-9 is not just a motor. It is an **integrated actuator** — five different parts packed together into one compact cylinder (Ф98 × 38.5 mm, only 490 g):

- a **three-phase brushless motor (BLDC)** — this is the part that actually creates the turning force (torque). It has permanent magnets inside (21 pole pairs — we will use the pole-pair number later in Ch 3);
- a **9:1 planetary gearbox** — this slows down the motor's fast rotation and, in exchange, gives us much higher torque at the output (§6 explains how);
- a **magnetic encoder** — a small sensor that constantly measures the exact position of the rotor. The controller needs this information at every instant to drive the motor correctly;
- a **MOSFET inverter** — an electronic circuit that takes the DC power you supply and converts it into the special three-phase AC power that the motor windings need;
- a **driver board** — this is a small computer inside the actuator. It runs the high level control loops (current, velocity, position), uses the FOC (Field-Oriented Control) algorithm for controlling the MOSFET inverter, and also talks to your robot's main computer through the CAN bus or a serial cable, so it can receive your commands and send back data.

This is the one place in the whole series where we open up the actuator and look at what is inside. Every later chapter will simply refer back to this section. From the outside, all of these components are connected through just three connections:

- the **three-phase wire port** connecting the driver board to the motor windings (already connected at the factory — you never touch this);
- the **black XT30 2+2 connector** — one single plug that carries both DC power *and* the CAN bus;
- the **white CJT 3-pin serial connector** — used to talk to the CubeMarsTool software on your PC (we will use it in Ch 4).

Why is this multi-component integration such a big deal? With a normal hobby BLDC motor, you would need to buy an ESC (Electronic Speed Controller — the small driver board that switches battery power into the motor windings), an encoder, and a gearbox separately, wire them all together, and then tune the whole system yourself. Here, all of these parts are already matched, mounted, and tuned at the factory. The whole module gives your robot one clean, simple interface. The price of this convenience is that you must always work *through* the driver's control modes — and learning those modes is exactly what this series is about.

Because the driver sits inside the case, you never send power to the motor windings yourself. Instead, you supply DC power and send *commands* — a duty ratio, a current, a velocity, a position, or an MIT command — to the driver, and the internal controller algorithm that lives inside the driver does all the heavy-lifting work (controlling the MOSFET inverter to produce the required voltage needed for each of the three phase windings). Which command types exist, and when to use each one, is the subject of this entire series.

A good way to picture this is as **two levels of control**, one sitting on top of the other:

```text
YOU (or your robot's main computer)
        │
        │  sends ONE setpoint over CAN/serial:
        │  (Servo Mode Commands) a duty ratio, a current, a 
        │  velocity, a position, or an MIT command
        ▼
┌─────────────────────────────┐
│   HIGHER-LEVEL LOOP         │   Asks: "What result do you 
│   (the Servo mode or MIT    │         want?"
│    command you selected)    │
└─────────────┬───────────────┘
              │  converts your setpoint into a
              │  torque-producing current target
              ▼
┌─────────────────────────────┐
│   FOC CONTROLLER            │   "How do I make the windings
│   (lower level, inside      │    produce that current,
│    the driver, always       │    right now?"
│    running)                 │
└─────────────┬───────────────┘
              │  drives the three-phase currents
              ▼
        MOTOR WINDINGS
```

Notice the difference between the two levels. The setpoint you send is a single target value — it only changes when *you* send a new command. The FOC controller, on the other hand, never stops working as long as the motor is up and running: it runs thousands of times per second, constantly adjusting the real current flowing into the windings so that it matches whatever the higher level is asking for at that moment. We will meet this same two-level picture again, with full electrical detail, in Ch 2 §§5 and 10.

## 2. The Ratings Table — Our Reference for the Whole Series

The table below is the reference table for the whole series — every later chapter checks its numbers against this one. The values come from CubeMars' published AK80-9 V3.0 KV100 specifications on the official website and from the AK Series manual.

| Rating | AK80-9 V3.0 KV100 value | What does it mean for me? |
|---|---:|---|
| **Rated voltage** | **48 V DC** | The driver's intended **nominal supply voltage for continuous, long-duration normal performance**. The documented **allowable working voltage is 18–52 V**; never exceed 52 V. |
| **Rated current** | **12 A DC** | The published **rated-current operating point** — the current linked to continuous, long-duration normal performance. It is not a value that you have to necessarily command all the time, and it is different from the short-duration peak capability. |
| **Peak current** | **28 A DC** | The **peak-current specification**. The motor can handle this only for a *very short time*. Do not treat 28 A as a current that we can supply to the motor continuously. |
| **Rated torque** | **9 N·m** | The **normal rated output torque** — the number to look at when asking, "how much torque can this actuator produce during long, sustained operation?" |
| **Peak torque** | **22 N·m** | The **maximum short-duration torque**. Useful for brief moments like a quick acceleration or a sudden heavy load — not a torque to demand continuously. |
| **Rated speed** | **390 rpm** | The output-shaft speed linked to the **rated operating conditions**. It is not the fastest the shaft can turn. |
| **No-load speed** | **570 rpm** | Approximately how fast (maximum value) the output shaft can spin with **almost no external mechanical load**. |
| **Gear reduction ratio** | **9:1** | The internal motor turns about 9 times for every 1 turn of the output shaft. This trades speed for much more torque. As a robot builder, the speeds and torques you care about are the ones at the output shaft. |
| **Motor KV** | **KV100** | About 100 rpm per volt for the **internal motor** (no-load) — not 100 rpm/V at the geared output (§6). |
| **Motor torque constant** | **0.095 N·m/A** | The **motor-side** constant. It connects the current $I_q$ to the torque made by the internal motor **before the gearbox**. *(Don't worry if you don't know what this $I_q$ is yet. You will learn it in the second chapter.)*|
| **Effective output shaft torque constant** | **0.5701 N·m/A** | The value for estimating **torque at the actuator output shaft**. Not interchangeable with 0.095 N·m/A (§7, Ch 2 §6). |

Three simple words unlock most of this table:

> **Rated** = normal specified working condition. **Peak** = temporary high-performance capability. **No-load** = spinning freely with almost nothing resisting it.

## 3. Rated vs Peak: Torque and Current

*Rated* is the most important word on any motor datasheet. A **rated** value describes the manufacturer's intended **normal operating condition**. A **peak** value describes what the actuator can reach for a **short time** when much higher performance is needed. In simple words: rated is where the motor lives; peak is where it can visit, but only briefly.

Also notice: the difference between rated and peak is about *time*, not just about size. The datasheet does not tell us exactly how long "briefly" is — that depends on how hot the motor already is and on the surrounding temperature. So the practical rule is: plan your normal workload around the rated numbers, and treat everything between rated and peak as short-visit territory that produces extra heat.

**Torque.** The AK80-9 is rated at 9 N·m, with a peak of 22 N·m. Neither number means the motor "normally produces 22 N·m." If you are choosing an actuator for a robot joint that needs strong torque for long periods, the 9 N·m rated figure is the one that matters. The 22 N·m peak is what you can briefly use when accelerating a mechanism or handling a sudden large load. The rule is simple:

> **Peak torque is not continuous torque.** If your application needs about 22 N·m all the time, do not choose this motor just because the datasheet says "22 N·m peak."

**Current.** The same logic applies to 12 A rated vs 28 A peak. In this actuator, torque and current are closely connected (Ch 2 §6 explains the exact relationship), so the current ratings basically follow the torque ratings. But current needs extra care for one reason: **heat**. The heat produced in the windings grows with the *square* of the current:

$$
P_{\mathrm{Cu}} = I^2 R
$$

(This $P_{\mathrm{Cu}}$ is called the "copper loss" — it is simply the heat created when current flows through the resistance of the copper wire coiled inside the motor. It is the same reason any wire becomes warm when a large current passes through it.) Because of that *square*, running near the peak current heats the motor very quickly. This is the physical reason behind "peak = very short duration only."

> **Important:** do not confuse the 12 A / 28 A ratings with the DC current shown on your bench power supply (the lab power source you plug the actuator into, which displays the voltage and current it is delivering), and do not automatically assume they are the same as $I_q$.

## 4. Rated Speed vs No-Load Speed

The two speed numbers work in a similar way to the two torque numbers — but here, the difference between them is about *load*, not about time.

**Rated speed — 390 rpm** is the output-shaft speed linked to the rated operating point. In other words, it is how fast the actuator can turn while doing real, meaningful mechanical work under rated conditions.

**No-load speed — 570 rpm** is approximately how fast the output can spin when almost nothing is resisting it. The key phrase is "no load": this speed is a ceiling that the motor reaches only when it does not need to produce significant torque against anything.

Because of this, we must not combine the two specifications carelessly. The datasheet does **not** promise 22 N·m *at* 570 rpm:

> **Torque and speed cannot both be pushed to their maximum values at the same time.** As the mechanical load grows, the achievable speed falls below the no-load figure.

Why does this happen? The full answer involves the supply voltage, something called back-EMF, and the motor's current limits — and Ch 5 §4 tells this story properly, at the point where you can actually watch it happen (in Duty Cycle Mode). For now, just remember the simple version: read 390 rpm as "speed while working" and 570 rpm as "speed while spinning freely."

## 5. Supply Voltage: 48 V Nominal, 18–52 V Allowable

The AK80-9 is a **48 V-class actuator**: a good 48 V DC power system is the normal choice, and the datasheet's rated speed and torque numbers are measured at that supply voltage.

However, "48 V rated" is *not* an absolute maximum, and it is *not* a minimum either. The manual's driver specification says clearly:

- **Rated working voltage: 48 V.**
- **Allowable working voltage: 18–52 V.**
- **Never exceed 52 V** at the actuator's power input.

Two practical lessons follow from this.

**Be very careful with batteries.** A battery *labelled* "48 V" is only a nominal (average) label — it is not the actual voltage at its terminals. Take a 13S lithium-ion pack as an example: its nominal voltage is 13 × 3.7 V ≈ 48.1 V, but when fully charged it reaches 13 × 4.2 = **54.6 V — above the 52 V limit** and therefore not safe to use without extra regulation. A 12S pack (44.4 V nominal) reaches 12 × 4.2 = 50.4 V when fully charged, which is inside the range but leaves little margin. **So the rule is: power-system design must always use the fully charged voltage of the pack, checked against the 18–52 V allowable range — never the label on the battery.** There is one more reason to stay safely below the 52 V ceiling: when the motor brakes or slows down quickly, it can briefly push a little energy back into the supply line, which raises the voltage slightly for a moment. Leaving some margin below 52 V keeps you safe from these small voltage bumps.

**Below 48 V is allowed, but the motor will reach a lower top speed.** Any supply inside 18–52 V is within specification. However, a lower supply voltage leaves less "spare voltage" to keep pushing current into the motor as it speeds up. (Engineers call this spare voltage the *voltage headroom* — think of it as the extra push still left in the tank after overcoming the motor's own back-EMF.) Less headroom means a lower maximum speed. Ch 5 §7 will show this effect in Duty Cycle Mode. The published 390/570 rpm figures assume the 48 V nominal supply.

## 6. The 9:1 Gearbox and What "KV100" Means

**The gearbox.** Inside the actuator, a 9:1 planetary gearbox sits between the motor's rotor and the output shaft. The internal motor turns about 9 times for every single turn of the output:

$$
n_{\mathrm{out}} = \frac{n_m}{9}
$$

So if the internal motor spins at 900 rpm, the output shaft turns at about 100 rpm. Why would we deliberately make the output *slower*? Because the gearbox gives us **torque multiplication** in return: high speed and modest torque at the motor become lower speed and much higher torque at the shaft. In the ideal (lossless) picture, the same 9:1 factor multiplies the torque — **output torque ≈ 9 × motor torque**. Real gearboxes, of course, lose a little energy to friction, and that is exactly why the output-side torque coefficient in §7 is not simply $9 \times 0.095$ (Ch 2 §6.3 does that calculation carefully). This speed-for-torque trade is how a small, fast electric motor becomes a strong robotic actuator — and it is also why all the torque and speed rows in the datasheet describe the *output shaft*.

**KV100.** The "KV100" in the model name describes the internal motor's velocity constant: roughly **100 rpm per volt**, at no load, *for the internal motor* — not for the geared output. This one number lets us do a nice self-check of the datasheet. At the 48 V nominal supply, the internal motor's no-load speed is roughly $100 \times 48 = 4800$ rpm. Passing that through the 9:1 gearbox gives an ideal output speed of

$$
\frac{100 \times 48}{9} \approx 533\ \mathrm{rpm},
$$

which is very close to the published **570 rpm no-load speed**. So the three specifications — KV100, the 9:1 ratio, and 570 rpm — all agree with each other. That is a good sign that we are reading the datasheet correctly. (Ch 3 §2 repeats this same style of self-check for the ERPM conversion.)

Two warnings about KV. First, it always refers to the internal motor, never directly to the output shaft. Second, **a higher KV does not mean "more powerful"** — KV only describes speed-per-volt, not power or torque.

## 7. Two Torque Constants — a Short Preview

The datasheet lists two torque numbers: **0.095 N·m/A** (the motor-side torque constant, which relates the current $I_q$ to the torque made *before* the gearbox) and **0.5701 N·m/A** (the output-side coefficient, for the torque at the *geared output shaft*). These two are not interchangeable — one applies before the 9:1 gearbox, the other after it, with the gearbox's losses already included. Ch 2 §6 explains fully which one to use, and why the second is not simply $9 \times 0.095$.

## 8. Four Everyday Scenarios

To finish, let's imagine the actuator installed at a robot joint, and see which datasheet rows matter in four common situations.

**Scenario A — normal, sustained operation.** The robot is moving its mechanism and producing good torque for a long period. The numbers that matter here are the **rated** torque, speed, and current — plus the thermal limits. Choosing an actuator for this kind of work by looking at its *peak* numbers is the classic datasheet mistake that §3 warned about.

**Scenario B — a brief acceleration.** The joint needs a big torque for a short burst to accelerate a mechanism. Now the **peak** torque and peak current become relevant — but only briefly. This is exactly what the gap between 9 and 22 N·m exists for: short demands, paid for with extra heat, not a new "normal" operating point.

**Scenario C — spinning freely.** With almost no mechanical load, a high speed command can bring the shaft close to the **no-load speed** of 570 rpm. The moment a real load appears, §4 applies and the achievable speed drops below that ceiling.

**Scenario D — holding a heavy load.** The shaft reads **0 rpm**, but the motor is still producing strong torque against a load that is trying to move it. Zero speed does *not* mean the motor is doing nothing: it can be producing considerable torque and drawing considerable current while completely still — and that current heats the windings. This important idea, "zero speed ≠ zero current ≠ zero heat," gets its full treatment in Ch 7 §5.

Put together, these four scenarios summarize the whole chapter: use the *rated* rows for long, sustained work; use the *peak* rows for short bursts; use the *no-load* row for free spinning — and always keep an eye on current, because current means heat.

## About Demos in This Chapter

There is deliberately no hands-on demo here. This chapter comes before the first power-up, so nothing in it requires — or should tempt you into — connecting the actuator to a power supply yet. The series' first hands-on step is the connect-and-calibrate demo in Ch 4, and the first real motion is Ch 5's Duty Cycle "first spin."

## Where These Ideas Go Next

| Concept introduced here | Continues in |
|---|---|
| The internal anatomy (motor, inverter, encoder, FOC driver, gearbox) | Ch 2 (the electrical chain in detail) |
| Currents, $I_q$, and the two torque constants | Ch 2 §§1–6 |
| Speed units (rpm vs ERPM) and the pole-pair count | Ch 3 |
| Connectors, CubeMarsTool, calibration, first power-up | Ch 4 |
| Voltage headroom, back-EMF, and the torque–speed trade-off | Ch 5 |
| Holding torque and heating at standstill (Scenario D) | Ch 6 §5, Ch 7 §5 |

## Key Takeaways

- The AK80-9 V3.0 KV100 is an integrated actuator — a BLDC motor, a 9:1 planetary gearbox, a magnetic encoder, a MOSFET inverter, and an FOC driver, all in one 490 g module. You send commands; the internal driver runs the motor.
- **Rated** values (9 N·m, 12 A, 390 rpm) describe normal, sustained operation. **Peak** values (22 N·m, 28 A) are short-duration capabilities only. The **no-load** speed (570 rpm) assumes almost zero external load. Design around rated; save peak for short bursts.
- The supply is 48 V nominal, with a documented **18–52 V allowable range — never exceed 52 V**. Always check a battery's *fully charged* voltage against that limit, not the label.
- KV100 and the 9:1 ratio describe the internal motor and its gearing. The self-check $100 \times 48 / 9 \approx 533$ rpm ≈ the published 570 rpm no-load speed shows the datasheet is consistent.
- There are two torque constants — 0.095 N·m/A on the motor side, 0.5701 N·m/A on the output side — and they are not interchangeable (Ch 2 §6).

## Safety Notes

- Never exceed **52 V** at the power input. Before connecting any battery pack, check its *fully charged* voltage against the 18–52 V allowable range.
- Treat the peak torque and peak current (22 N·m / 28 A) as short-duration values only. Winding heat grows as $I^2R$, so sustained high current — including at stall, or while holding a load in place — heats the motor very quickly.
- Nothing in this chapter requires powering the motor. Before your first power-up, follow Ch 4's checklist, its calibration procedure, and its series-wide safety rules.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically §1.1/§1.2 "Driver Appearance Introduction & Product Specifications" (p. 7–8: rated working voltage 48 V, **allowable working voltage 18–52 V**, standby consumption ≤ 50 mA, CAN bit rate 1 Mbps, connector/port identification).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — rated voltage 48 V; rated/peak torque 9 / 22 N·m; rated/peak current 12 / 28 A; rated/no-load speed 390 / 570 rpm; reduction ratio 9:1; KV 100 rpm/V; KT 0.095 N·m/A; output torque–current coefficient 0.5701 N·m/A; 36 slots, 21 pole pairs; dimensions Ф98 × 38.5 mm; weight 490 g; XT30 2+2 power+CAN and CJT 3-pin serial interfaces.
