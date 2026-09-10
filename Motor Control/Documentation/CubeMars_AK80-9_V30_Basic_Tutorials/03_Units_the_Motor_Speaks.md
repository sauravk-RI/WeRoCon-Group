# Chapter 3 — The Units the Motor Speaks: Degrees, ERPM, rad/s, and N·m

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 4 — The Control-Mode Map and First Power-Up →](04_Control_Mode_Map_and_First_Power_Up.md)

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

Imagine this. You connect your AK80-9 for the first time, type in a speed of **1,000 ERPM**, and press start. You expect the shaft to spin fast. Instead, it crawls — about one full turn every 11 seconds. Is the motor broken?

No. The motor is fine. The **unit** fooled you.

The AK80-9 speaks four different "unit languages," and mixing them up is the number one beginner mistake with this actuator — more common than any wiring or physics mistake. The Servo modes use **degrees** for position and **ERPM** for speed. MIT mode uses **radians**, **rad/s**, and **N·m**. The telemetry (the data the motor sends back) uses its own number scalings. None of this is hard — but you must know which language you are speaking at each moment.

This short chapter is the one place in the series where every unit is explained. Later chapters will simply point back here. Step by step, we will cover:

- position units — degrees in the Servo modes, radians in MIT mode (§1);
- what ERPM really means, and the magic number **189** that converts it to shaft speed (§2);
- converting between rad/s and rpm (§3);
- acceleration in ERPM/s² (§4);
- torque in newton-metres, with a feel for how much 1 N·m actually is (§5);
- one master reference table with every unit, range, and scaling in one place (§6).

There is no hands-on demo in this chapter — it is a reference chapter, made to be bookmarked. Your first real motion command comes in Ch 5, after Ch 4 teaches calibration and the safety ground rules.

## 1. Position: Degrees for Servo, Radians for MIT

You already know degrees: one full turn of the shaft is $360°$. A **radian** is simply a second way to measure the same angle, and one full turn is $2\pi \approx 6.283$ rad. So $180° = \pi$ rad, $90° = \pi/2$ rad, and the conversions are:

$$
\theta_{\mathrm{deg}} = \theta_{\mathrm{rad}} \times \frac{180}{\pi},
\qquad
\theta_{\mathrm{rad}} = \theta_{\mathrm{deg}} \times \frac{\pi}{180}.
$$

One more thing before the ranges: every position in this series means the position of the **output shaft** — the part you can touch, after the 9:1 gearbox. You never command the inner rotor directly.

**The Servo modes speak degrees.** Multi-turn Position–Velocity mode accepts ±36,000°. That sounds huge, but it is simply ±100 full turns (Ch 10). Single-turn mode accepts one turn only, 0–359° (Ch 10). Plain Position Loop Mode's P field uses the same ±36,000° multi-turn range (Ch 9).

**MIT mode speaks radians.** Its position range for the AK80-9 is ±12.56 rad. Notice that $12.56 \approx 2 \times 2\pi$ — so this is only about **±2 full turns**, much less travel than the Servo modes give you (Ch 11). If you move a motion from one mode family to the other, convert carefully: a Servo target of 720° (2 turns) becomes $720 \times \pi/180 \approx 12.57$ rad in MIT mode — right at the edge of the MIT range.

## 2. What Is ERPM, Really? (And the Magic Number 189)

ERPM means **electrical revolutions per minute**. The important word is *electrical* — ERPM is **not** the speed of the shaft you can see. To understand it, we only need two facts about what is inside the actuator.

**Fact 1: the rotor has 21 pole pairs.** The spinning rotor carries permanent magnets arranged as 21 north–south pairs. Because of this, every time the rotor makes **one** real turn, the electrical pattern in the windings repeats **21 times**. In Ch 2 §§4–5 we saw that FOC works with the rotor's *electrical* angle $\theta_e$; here is its exact relationship to the mechanical angle $\theta_m$:

$$
\theta_e = 21\,\theta_m
\quad\Rightarrow\quad
\text{one rotor turn} = 21 \text{ electrical cycles}.
$$

The firmware counts speed in these electrical cycles, because that is the rhythm the electronics actually work at. So ERPM $= 21 \times$ rotor rpm.

**Fact 2: the gearbox slows things down by 9.** The rotor drives a 9:1 planetary gearbox, so the output shaft turns 9 times slower than the rotor: output rpm $=$ rotor rpm $\div\, 9$.

Now stack the two facts together: one turn of the output shaft = 9 rotor turns = $9 \times 21 = 189$ electrical cycles. That gives us the most important formula in this series:

$$
\boxed{\;\text{output rpm} = \frac{\text{ERPM}}{21 \times 9} = \frac{\text{ERPM}}{189}\;}
\qquad\text{and the other way:}\qquad
\text{ERPM} = 189 \times \text{output rpm}.
$$

Divide by 189 to go from a command to real shaft speed. Multiply by 189 to go from the speed you want to the command you must type. Here is what typical commands really do:

| ERPM command | Output-shaft speed | What you will see |
|---:|---:|---|
| 1,000 | ≈ 5.3 rpm | a very slow crawl — one turn every ~11 s |
| 5,000 | ≈ 26 rpm | slow but clearly visible rotation |
| 10,000 | ≈ 53 rpm | a comfortable demo speed |
| 50,000 | ≈ 265 rpm | the maximum the CubeMarsTool GUI allows |
| 100,000 | ≈ 529 rpm | the maximum the CAN protocol allows |

**A quick check that the formula is right.** The CAN protocol's top speed is 100,000 ERPM, and $100{,}000 \div 189 \approx 529$ rpm. Compare that with the AK80-9's published **no-load speed of 570 rpm** (Ch 1 §4), and with Ch 1 §6's estimate from the KV rating: $100 \times 48 \div 9 \approx 533$ rpm. All three land in the same place. ✔ If our pole-pair count were wrong, the answer would be off by a whole factor (2×, 3×, ...), not by a few percent. The small remaining gap is normal: real no-load speed depends on supply voltage and losses, while the 100,000 figure is just where the protocol designers placed the limit — close to the motor's real top speed.

Going the other way is what you will do most often when *planning* a motion: choose the shaft speed you want, then multiply by 189. Want a relaxed 120 rpm at the shaft? Command $189 \times 120 = 22{,}680$ ERPM. Want about 1 turn per second (60 rpm)? Command $189 \times 60 = 11{,}340$ ERPM.

Two practical tips before we move on. First, you now understand the opening story: the old example command of "1000 ERPM" gives only ~5 rpm, so either expect very slow motion, or use roughly 5,000–20,000 ERPM when you want rotation you can actually watch (the demos in Ch 8 and Ch 10 do exactly this). Second, you will meet **two different speed limits, and both are real**: the CubeMarsTool GUI's S field accepts ±50,000 ERPM (manual §3.3.1.6), while the CAN Velocity Loop accepts ±100,000 ERPM (manual §4.1.4). Neither is a misprint — the GUI simply allows a smaller range than the protocol. The table in §6 records both, with labels.

## 3. Converting Between rad/s and rpm

MIT mode commands velocity in **rad/s** at the output shaft, so you will often need to switch between rad/s and rpm. The logic is simple: one turn is $2\pi$ rad, and one minute is 60 seconds, so

$$
n\ \text{[rpm]} = \omega\ \text{[rad/s]} \times \frac{60}{2\pi} \approx 9.549\,\omega,
\qquad
\omega \approx 0.1047\,n .
$$

An easy way to remember it: **1 rad/s is roughly 9.5 rpm** — call it "about ten." For example, an MIT velocity target of 6 rad/s is $6 \times 9.549 \approx 57.3$ rpm, close to one turn per second. And if you ever need to connect all the way back to Servo units: output $\omega = \text{ERPM} \times 2\pi/(189 \times 60)$, so 100,000 ERPM ≈ 55.4 rad/s.

One interesting detail: the AK80-9's MIT velocity range is ±65 rad/s (manual p.39), which is about ±621 rpm — *more* than the motor's 570 rpm no-load speed. Don't worry, this is not a contradiction. It is the same pattern Ch 2 §9 taught for the ±60 A current range: a protocol range only says what the *message* can express, not what the *motor* can physically do.

## 4. Acceleration: ERPM/s²

In Position–Velocity mode (Ch 10) you also command an **acceleration** — how quickly the motor is allowed to change its speed. This series uses the manual's unit for it: **ERPM/s²**. Read it in the natural way: an acceleration of $a$ ERPM/s² means *the speed grows by $a$ ERPM every second*. To feel it at the shaft, divide by 189 as usual — that is how many rpm the output gains each second.

On the wire, the Position–Velocity CAN frame carries acceleration as an int16 with **1 LSB = 10 ERPM/s²**, so the range 0–32,767 counts covers 0–327,670 ERPM/s² (manual §4.1.7). (Earlier drafts of this series wrote "ERPM/s" in some examples. Wherever you see that, read it as ERPM/s² — same quantity, corrected label.)

A concrete example makes it clear. Suppose you command a target speed of 18,900 ERPM (≈ 100 rpm at the shaft) with an acceleration of 9,450 ERPM/s². The speed ramps up smoothly and reaches the target in $18{,}900 \div 9{,}450 = 2$ seconds — the shaft goes from rest to 100 rpm, gaining 50 rpm each second.

## 5. Torque: Newton-Metres at the Output Shaft

Torque is turning force, and in this series it is always measured in **newton-metres (N·m) at the output shaft**. Here is a feel for the size of 1 N·m: it is roughly the torque made by a **1 kg mass hanging 10 cm from the axis** ($1\ \mathrm{kg} \times 9.81\ \mathrm{m/s^2} \times 0.102\ \mathrm{m} \approx 1$ N·m). So the AK80-9's 9 N·m rated torque (Ch 1 §2) is like holding 9 kg at 10 cm — or 1 kg on a 90 cm arm. Its peak is 22 N·m, and MIT mode lets you command torque directly over ±18 N·m (Ch 11).

Note that only MIT mode takes torque in N·m directly. In the Servo modes, torque is commanded *indirectly*, as an $I_q$ current in amps (Ch 6), and you convert with the torque constants from Ch 2 §6 (mainly $\tau_{\mathrm{out}} \approx 0.5701\,I_q$). The GUI's T field is only a braking-by-torque convenience, not a general torque mode (Ch 6 §8). Telemetry also reports current, not torque — multiply by 0.5701 N·m/A yourself when you want a torque estimate on screen.

## 6. The Master Units Reference Table

Everything above, plus the telemetry scalings from the manual's CAN upload frame (§4.3.1, p.42), gathered in one table. This is the series' unit reference — later chapters point to rows of this table instead of repeating the numbers.

| Quantity | Servo GUI unit & range | Servo CAN unit & scaling | MIT unit & range | Telemetry unit & scaling |
|---|---|---|---|---|
| Position | degrees; Multi ±36,000° (±100 turns), Single 0–359°, Position-mode P ±36,000° | int32, 1 LSB = 0.0001°, ±36,000° | rad, ±12.56 (≈ ±2 turns) | int16, 1 LSB = 0.1°, ±3,200° |
| Velocity | ERPM; S field ±50,000 | Velocity mode: int32, 1 LSB = 1 ERPM, ±100,000 · Pos–Vel frame: int16, 1 LSB = 10 ERPM, ±327,680 | rad/s, ±65 | int16, 1 LSB = 10 ERPM, ±320,000 ERPM |
| Acceleration | ERPM/s² (Multi/Single trap control) | Pos–Vel frame: int16, 1 LSB = 10 ERPM/s², 0–327,670 | — (dynamics set by gains) | — |
| Current | A; I field (loop), B field (brake) | int32, 1 LSB = 0.001 A; loop ±60 A signed, brake 0–60 A magnitude | — (commands torque instead) | int16, 1 LSB = 0.01 A, ±60 A |
| Torque | N·m; T field (braking-by-torque only) | — (no Servo torque frame; command $I_q$, ÷0.5701 N·m/A) | N·m, ±18 | — (derive: current × 0.5701 N·m/A) |
| Duty cycle | dimensionless; 0.005–0.95 (default) | int32 = duty × 100,000, signed | — | — |
| Temperature | °C (Real-time Data) | — | — | int8, 1 LSB = 1 °C, −20…127 °C (driver board) |
| Error code | decoded in GUI | — | — | uint8, 0 = none … 7 = motor lock-up |

A few notes to read the table correctly:

- **A protocol range is not a rating.** Numbers like ±60 A, ±100,000 ERPM, ±327,680 ERPM, and ±65 rad/s only describe what the message format can *express*. What the motor can safely *do* is given by Ch 1 §2's ratings (12 A / 28 A, 570 rpm no-load, 9 / 22 N·m). Ch 2 §9 explains this fully.
- **Telemetry position only covers ±3,200°** — much less than the ±36,000° command range. The small int16 field simply cannot hold the full multi-turn span, so do not rely on the periodic upload frame alone when tracking many turns of travel.
- The telemetry column is the Servo-side CAN upload frame (sent automatically at 1–500 Hz, manual §4.3.1). MIT mode replies with its own message format, covered in Ch 11.
- MIT's gains $K_p$ (range 0–500) and $K_d$ (range 0–5) travel as plain numbers in the packet, but they act with the units N·m/rad and N·m/(rad/s) (Ch 11).
- Both velocity limits in the Servo columns are real: ±50,000 ERPM is the GUI's limit, ±100,000 ERPM the CAN protocol's (§2).

<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
    <figure>
      <img src="images\image1.png" 
          alt="CubeMarsTool Servo Control panel with the input fields annotated with their units — P (degrees), S (ERPM), the Multi/Single trap-control position, speed, and acceleration fields (°, ERPM, ERPM/s²), I (A), B (A), T (N·m), and duty (dimensionless)">
      <figcaption>
        Figure 3.1: CubeMarsTool Servo Control panel with the input fields annotated with their units — P (degrees), S (ERPM), the Multi/Single trap-control position, speed, and acceleration fields (°, ERPM, ERPM/s²), I (A), B (A), T (N·m), and duty (dimensionless).
        <i>
          Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.
        </i>
      </figcaption>
    </figure>
  </figure>
</div>



## Where These Units Appear Next

This chapter is pure reference; here is where each unit system gets put to work:

| Unit / conversion defined here | First used hands-on in |
|---|---|
| Duty (dimensionless, 0.005–0.95) | Ch 5 (Duty Cycle Mode) |
| Amps → N·m via 0.5701 (with Ch 2 §6) | Ch 6 (Current Loop), Ch 7 (Brake) |
| ERPM commands and the ÷189 conversion | Ch 8 (Velocity Loop) |
| Degrees, multi-turn ±36,000° span | Ch 9 (Position Loop), Ch 10 (Position–Velocity) |
| ERPM/s² acceleration | Ch 10 (Position–Velocity) |
| Radians, rad/s, N·m together | Ch 11 (MIT mode) |

## Key Takeaways

- The Servo modes speak degrees and ERPM; MIT mode speaks radians, rad/s, and N·m; the telemetry has its own integer scalings. §6's table maps them all.
- The formula to remember: **output rpm = ERPM ÷ 189** (21 pole pairs × 9:1 gearbox), and ERPM = 189 × output rpm. Quick check: 100,000 ERPM ≈ 529 rpm ≈ the 570 rpm no-load speed. ✔
- 1,000 ERPM is only a ~5.3 rpm crawl — use about 5,000–20,000 ERPM when you want motion you can see. The GUI allows ±50,000 ERPM; the CAN protocol ±100,000 ERPM; both limits are real.
- Acceleration is in ERPM/s², meaning "ERPM gained per second," with the CAN scaling 1 LSB = 10 ERPM/s² (range 0–327,670). Divide by 189 for rpm gained per second at the shaft.
- rad/s × 9.549 = rpm (roughly, ×10). MIT position ±12.56 rad ≈ ±2 turns; MIT commands torque directly in N·m (±18).
- Protocol and encoding ranges are message limits, never permissions — Ch 1 §2's ratings are what protect the hardware (Ch 2 §9).

## Safety Notes

- Unit confusion is a real safety issue, not just an annoyance. A number that is harmlessly slow as ERPM (1,000 ERPM ≈ 5.3 rpm) is fast if the motor read it as shaft rpm — and "fixing" a crawling demo by pushing the command toward a protocol limit can suddenly jump the shaft to hundreds of rpm. Always convert first (§2), then command — and follow Ch 4 §6's ground rules: start with no load, start small.
- Never treat the wide protocol ranges in §6's table as permission. Stay inside Ch 1 §2's ratings, and well below them while you are learning (Ch 2 §9).

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §3.3.1.1–3.3.1.8 (p. 24–28: GUI command fields and ranges — multi-turn ±100 turns / ±36,000°, single-turn 0–359°, velocity S ±50,000 ERPM, duty default 0.005–0.95, T/P/I/B fields); §4.1.1–4.1.7 (p. 33–37: Servo CAN scalings — duty × 100,000; current int32 ±60,000 ↔ ±60 A; brake 0–60,000 ↔ 0–60 A; velocity int32 ±100,000 ERPM; position int32 ±360,000,000 ↔ ±36,000°; Position–Velocity frame speed int16 1 LSB = 10 ERPM and acceleration int16 1 LSB = 10 ERPM/s², 0–327,670); §4.2 and the Parameter Ranges table (p. 38–39: MIT AK80-9 position ±12.56 rad, velocity ±65 rad/s, torque ±18 N·m, Kp 0–500, Kd 0–5); §4.3.1 (p. 42: telemetry — position int16 0.1°/LSB ±3,200°, speed int16 10 ERPM/LSB ±320,000 ERPM, current int16 0.01 A/LSB ±60 A, temperature int8 −20…127 °C, error codes 0–7, upload 1–500 Hz).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — 21 pole pairs, 9:1 reduction, KV100, 570 rpm no-load speed, 9 N·m rated / 22 N·m peak torque; summarized in this series' reference table (Ch 1 §2).
3. **CubeMarsTool upper-computer software** — Servo Control panel shown in Figure 3.1; © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.

---

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 4 — The Control-Mode Map and First Power-Up →](04_Control_Mode_Map_and_First_Power_Up.md)
