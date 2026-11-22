# Chapter 4 — The Control-Mode Map, CubeMarsTool, and First Power-Up

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 5 — Duty Cycle Mode →](05_Duty_Cycle_Mode.md)

---

**FirstAuthor:** Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
**Disclaimer:** This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

In Chapters 1–3, we learned what the AK80-9 is, how current becomes torque inside it, and which units it uses. Now it is time to move from theory to the real motor on your bench.

This chapter does four things:

- It shows you **all the control modes** the motor offers, so you have a map of the whole series (§1–§2).
- It gives you a tour of **CubeMarsTool**, the software on your PC that talks to the motor (§3).
- It walks you through the **first power-up** and the **calibration procedure** — a one-time setup step that the motor needs in some situations (§4–§5).
- It sets the **safety ground rules** that every later chapter will follow (§6).

We will not make the motor do any real work yet. The only motion in this chapter happens during calibration. Your first real "make it spin" moment comes in Chapter 5.

## 1. Two Families of Control

The AK80-9 gives you two different "families" of control. Every command you will ever send belongs to one of them.

**Family 1: Servo Control.** This family has six simple, single-purpose modes. Each mode controls exactly **one** thing: a duty cycle, a current, a braking current, a speed, a position, or a position with a speed profile. You pick the mode, you send one target value, and the motor takes care of the rest. Simple and direct.

**Family 2: Force Control, also called MIT Control.** This family is one single, more advanced mode. (The name "MIT" comes from the MIT Mini Cheetah robot, where this control style became famous.) Instead of one value, every MIT command sends **five values at the same time**: a desired position, a desired velocity, a feedforward torque, and two gains called $K_p$ and $K_d$. Depending on which values you use and which ones you set to zero, the motor behaves like a position controller, a velocity controller, or a torque controller. Do not worry if this sounds confusing right now — Chapter 11 explains it step by step.

How does the motor know which mode you want? Over the CAN bus, every mode has its own **ID number**. Here is the full map (from manual §4.1 and §4.2):

| Control Mode ID | Family | What it does |
|---:|---|---|
| 0 | Servo | Duty Cycle Mode |
| 1 | Servo | Current Loop Mode |
| 2 | Servo | Current Brake Mode |
| 3 | Servo | Velocity Loop Mode |
| 4 | Servo | Position Loop Mode |
| 5 | Servo | Set Origin Mode |
| 6 | Servo | Position–Velocity Loop Mode |
| 8 | MIT | Force Control (position / velocity / torque, decided by the data you send) |

One small note: **Set Origin (ID 5)** is not really a control mode. It is a helper function. It tells the motor: "your current position is now zero." All position commands are then measured from that zero point.

Here is a simple way to feel the difference between the two families. Servo modes are like a microwave oven: you set one number (the time), press start, and the machine handles everything else. MIT mode is like driving a car: you are adjusting several things together, continuously, and the behavior comes from the combination. Part II of this series (Chapters 5–10) teaches the Servo modes one by one. Part III (Chapter 11) brings everything together in MIT mode.

**One naming note, stated once for the whole series.** The manual (p. 38) calls the three MIT behaviors "Position Loop Mode / Velocity Loop Mode / Current Loop Mode" — the same names as the Servo modes! To avoid confusion, this series will always say **MIT Position, MIT Velocity, and MIT Torque**. When you read the manual and see the other names, remember they mean the same thing.

## 2. What Is Each Mode For?

Here is a short, friendly introduction to each mode — just one paragraph each. Every mode gets its own full chapter later, so you only need the general idea for now.

**Duty Cycle Mode (ID 0 → Chapter 5).** The most basic command of all. You give the motor a fraction between 0.005 and 0.95 (in the GUI), and the motor's electronics apply that fraction of the supply voltage to the motor. That is all. Nothing is regulated — the current, speed, and position simply follow the physics. This "raw" behavior is exactly why we start here: it lets you *see* the physics from Chapter 2 happen in real life.

**Current Loop Mode (ID 1 → Chapter 6).** You tell the motor how much current $I_q$ to use. And because torque ≈ 0.5701 × $I_q$ (Chapter 2 §6), commanding current is really commanding **torque**. The motor pushes with that torque; how fast it ends up spinning depends on the load.

**Current Brake Mode (ID 2 → Chapter 7).** You give a braking-current value, and the motor **resists** motion and tries to hold its current position. Note: this command has no minus sign — the controller automatically pushes against whichever direction the shaft is being moved. The manual adds a warning we should remember from day one: *"pay attention to motor temperature when using"*. Holding still takes continuous current, and continuous current makes heat.

**Velocity Loop Mode (ID 3 → Chapter 8).** You give a speed in ERPM (±50,000 in the GUI; ±100,000 over CAN), and the controller keeps adjusting the current so the motor holds that speed, even when the load changes. Remember from Chapter 3: shaft rpm = ERPM ÷ 189, so 10,000 ERPM is only about 53 rpm at the shaft.

**Position Loop Mode (ID 4 → Chapter 9).** You give a target position in degrees (up to ±36,000°, which is ±100 full turns), and the motor goes there. **Important:** the manual clearly says the motor moves there *"with maximum speed and acceleration"*. There is no gentle option in this mode. That is why Chapter 9's first exercises use small, nearby targets and no load.

**Position–Velocity Loop Mode (ID 6 → Chapter 10).** This is Position Mode with manners. You give the target position **and also** the speed and acceleration the motor should use on the way. The result is a smooth, controlled move. In CubeMarsTool it appears as **Multi Mode** (±36,000°) and **Single Mode** (0–359°, one turn only).

**MIT / Force Control (ID 8 → Chapter 11).** The all-in-one mode. Its rule is: torque $= K_p(p_{des}-p) + K_d(v_{des}-v) + \tau_{ff}$. Again — do not worry about this equation yet. The short version: the motor behaves like a programmable **spring and damper**, which is why walking robots love it. Set the right parts to zero, and it becomes MIT Position, MIT Velocity, or MIT Torque.

Not sure which mode you would need in a real project? This little table gives you a preview. Chapter 12 will explore these choices much more deeply:

| I want to… | Use this mode | Chapter |
|---|---|---|
| Just make the shaft spin and watch what a motor does | Duty Cycle | Ch 5 |
| Push with a specific torque / force | Current Loop | Ch 6 |
| Resist motion and hold the current position | Current Brake | Ch 7 |
| Spin at a fixed speed, no matter the load | Velocity Loop | Ch 8 |
| Go to a position; I do not care how aggressively | Position Loop | Ch 9 |
| Go to a position smoothly, with controlled speed | Position–Velocity | Ch 10 |
| Act like a spring–damper; build compliant robots | MIT (Force Control) | Ch 11 |

## 3. A Tour of CubeMarsTool

CubeMarsTool is the free program from CubeMars that runs on your PC. It talks to the motor through the serial cable, and it is where we will drive **all** the demos in this series. The interface has seven areas (manual §3.1). Let us walk through them once, so nothing surprises you later:

- **A — Configuration.** The settings area. Inside *Basic Settings* you will find the **Motor Settings** block with four buttons: Read, Motor Identification, Encoder Identification, and Write. These are the calibration buttons we will use in §5. The same page also has communication settings (CAN speed, CAN ID, serial baud rate, and so on). There is also an *Advanced Settings* page — and here the manual gives first-time users a clear warning: **do not change the advanced parameters just to see what happens**, or the motor may behave abnormally. If you ever do work there, always click *Read* first, before writing anything.
- **B — Real-time Status.** A health report: software and firmware versions, the current operating mode, and any fault messages. When something behaves strangely, look here first.
- **C — Real-time Data.** Live numbers and live graphs: currents, voltages, duty cycle, temperatures (both the motor and its MOSFETs), speed in ERPM, position, and encoder angle. This page is very important for us. Many things in a motor are invisible — you cannot *see* current, and you cannot *see* heat building up. But these plots show them clearly. Many demos in the coming chapters are really just "send a command, then watch this page."
- **D — Chinese–English switch.** The 中 / EN button in the top-right corner.
- **E — Control.** The place where commands are sent. It has several sections:
  - The upper **Trap Control** section belongs to Position–Velocity mode: **Multi Mode** (±36,000°) and **Single Mode** (0–359°), each with position, speed, and acceleration fields. It also has **Set zero point** and **Run to zero position** buttons. Careful with that last one — the manual warns that the motor runs to zero *at maximum speed*.
  - The **General Control** section has one input field for each simple Servo command: **T** (a braking-by-torque helper — Chapter 6 explains its story), **P** (position), **I** (current), **S** (speed), **B** (brake current), and **duty**. Each field has its own start button.
  - The **MIT Control** section takes the five MIT values: CAN ID, position, velocity, torque, $K_p$, and $K_d$.
  - A small **Unit Settings** panel can convert the displayed speed between RPM, ERPM, rad/s, and degrees-per-second. Enter 21 pole pairs and a 9:1 ratio for the AK80-9, and it does Chapter 3's math for you.
- **F — Connection.** Where you connect to the motor: click *Refresh*, choose the baud rate and the COM port, then click *Connect*. The text changes from "Not connected" to "Connected to COMx" when it works.
- **G — Stop.** The software emergency-stop button. One click and the motor stops working. **Find this button and remember where it is before you send any command.** This is one of our ground rules in §6.

## 4. First Power-Up: Checklist and LED Lights

Ready to power the motor for the first time? Go through this list slowly, in order:

1. **Bench, not robot.** Fix the motor to the bench, and make sure **nothing is attached to the output shaft**. Calibration must be done with no load (§5), and all our first demos assume a free shaft too.
2. **Check the wiring.** The three thick motor phase wires seated properly; the XT30 power+CAN connector and the small serial connector pushed fully in (Chapter 1 §1 shows these ports). Here's exactly which wire goes where, so you don't have to guess:

    | No. | Interface Function | Pin | Description | Color |
    |:---:|---|:---:|---|---|
    | 1 | **Serial communication** | 1 | Serial signal ground (GND) | Black |
    | | | 2 | Serial signal input (RX) | Yellow |
    | | | 3 | Serial signal output (TX) | Green |
    | 2 | **Power supply input and CAN communication** | 1 | Positive pole (+) | Red |
    | | | 2 | Negative pole (−) | Black |
    | | | 3 | CAN communication high side (CAN_H) | White |
    | | | 4 | CAN communication low side (CAN_L) | Blue |

3. **Set up the power supply.** 48 V (anywhere in the allowed 18–52 V range is fine — Chapter 1 §5). Set the supply's current limit low, a few amps. That is more than enough for calibration and first tests, and it protects you from surprises.
4. **Turn on the power and watch the LED lights.** The driver board has three small lights (manual §1.3):

   | Light | Color | Meaning |
   |---|---|---|
   | Power indicator | Blue | On = the driver board has power |
   | Operation indicator | Green | On = the motor is working (running a command) |
   | Fault indicator | Red | On = the driver board has a fault |

   The manual describes what a **healthy power-up** looks like: *the blue light stays on, and the green and red lights turn on for about 2 seconds and then go out.* So: a short green+red flash is normal — do not panic. After the flash, blue alone means all is well. But if the **red light stays on**, the board is reporting a fault: turn the power off and find the problem before going further.
5. **Connect CubeMarsTool** (§3, area F) and open **Real-time Status**: you should see a firmware version and no fault message.
6. **Look at Real-time Data**: the input voltage should match your supply, the currents should be near zero, and the temperatures near room temperature. (This is the same quiet-idle observation we did in Chapter 2's demo.)

### If something is not right

A quick help list, in the order you would meet the problems:

- **No blue light at all** → the board has no power. Check that the supply output is switched on, the polarity is correct, and the XT30 connector is fully seated.
- **Red light stays on after the 2-second flash** → a driver fault. Power off, re-check all wiring (especially the three phase wires), and try again. If it still happens, connect anyway and read the fault message in Real-time Status.
- **"Not connected" will not change** → wrong COM port or wrong baud rate. Plug the cable in, click *Refresh*, match the baud rate to the one in Basic Settings, and try again.
- **Connected, but the numbers look wrong or frozen** → click *Read* in Configuration first, so the software loads the board's real parameters (remember the Advanced-Settings warning in §3).

When everything looks healthy — and only then — we calibrate.

## 5. Calibration: Motor and Encoder Identification

The driver board needs to "know" two things: the electrical properties of the motor it is driving, and exactly how the encoder is aligned on the rotor. Teaching it these things is called **calibration** (the manual calls it "identification").

Good news first: **your motor was already calibrated at the factory.** A brand-new unit does not need it again. You only need to recalibrate when one of these three things happens (manual §3.2):

- you removed and reinstalled the driver board on the motor;
- you changed the order of the three phase wires;
- you updated the firmware.

Why do we still teach it carefully? Because after any of those events, **nothing in Chapters 5–11 will work correctly until calibration is done** — and the old drafts of this series never covered it at all.

Before the steps, read the manual's warnings. Please treat these as absolute rules, not suggestions:

> ⚠ *"The entire process of motor parameter and encoder parameter identification should be kept under no-load conditions; otherwise, it may lead to inaccurate identification parameters or motor damage."* — manual §3.2.1

> ⚠ *"The encoder parameter identification process generates heat, and performing this process consecutively multiple times can cause the motor temperature to rise sharply."* — manual §3.2.1

In plain words: **"no load" means nothing on the shaft.** No arm, no pulley, no coupling — a completely bare shaft. And calibration is not something to repeat again and again "just to be safe." Run it once, save it, done.

Now the procedure itself (manual §3.2.1). All four buttons are in Configuration → Basic Settings → Motor Settings:

- **Step 0 — Get ready.** Power stable, connectors seated, CubeMarsTool connected — in other words, §4's checklist is complete. Open the settings page.
- **Step 1 — Read.** Click **Read** and wait for the confirmation message. This copies the board's current parameters into the software.
- **Step 2 — Motor Identification.** Click **Motor Identification**. You will hear a **short, sharp beep, and then the motor starts turning with a noticeably loud noise**. This is completely normal — nothing is broken! It runs for about **10 seconds** and stops by itself. The software confirms when it is finished.
- **Step 3 — Encoder Identification.** Click **Encoder Identification**. This time the motor **turns slowly, for about 45 seconds**, then stops, and the software confirms. This is the step that generates heat — which is why we do not repeat it back-to-back.
- **Step 4 — Write.** Click **Write** and wait for the confirmation. The new parameters are now saved on the driver board. Calibration is finished, and the motor is ready to use.

To double-check your work: Real-time Status should show no faults, and Real-time Data at idle should show almost-zero current, room temperature, and an encoder angle that follows along when you slowly turn the shaft by hand.

### Demo — Connect and Calibrate

*Starting condition: motor fixed to the bench, output shaft completely free (no load), 48 V supply, §4's checklist done.* What you do: the Read → Motor Identification → Encoder Identification → Write sequence above. What you should see and hear: a beep, about 10 seconds of lively rotation, about 45 seconds of slow rotation, a confirmation after each step, and calm, healthy numbers in the telemetry afterwards.



<div style="display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%;">
  <figure style="text-align: center; margin: 0;">
    <figure>
      <img src="images\image2.jpeg" 
          alt="Connecting the CubeMars Motor with Laptop Using R-Link">
      <figcaption>
        Figure 4.1: Connecting the CubeMars Motor with Laptop Using R-Link.
      </figcaption>
    </figure>
    <figure>
      <img src="images\image3.jpeg" 
          alt="Complete Connection Bird's Eye View">
      <figcaption>
        Figure 4.2: Complete Connection Bird's Eye View.
      </figcaption>
    </figure>
    <figure>
      <img src="images\image4.png" 
          alt="Connection panel just after a successful connect — baud rate and COM port visible, status showing Connected to COMx">
      <figcaption>
        Figure 4.3*: Connection panel just after a successful connect — baud rate and COM port visible, status showing Connected to COMx
      </figcaption>
    </figure>
    <figure>
      <img src="images\image5.png" 
          alt="Configuration → Basic Settings page with the Read, Motor Identification, Encoder Identification, and Write buttons marked with numbered callouts in the order of the procedure.">
      <figcaption>
        Figure 4.4*: Configuration → Basic Settings page with the Read, Motor Identification, Encoder Identification, and Write buttons marked with numbered callouts in the order of the procedure.
      </figcaption>
    </figure>
    <figure>
      <img src="images\image0.png" 
          alt="Real-time Status panel after calibration, showing the firmware version, parameter status, operating mode, and an empty fault field.">
      <figcaption>
        Figure 4.5*: Real-time Status panel after calibration, showing the firmware version, parameter status, operating mode, and an empty fault field — our "this is what healthy looks like" reference picture for the series.
      </figcaption>
    </figure>
  </figure>
</div>

\* Screenshots captured from CubeMars' CubeMarsTool parameter-configuration software, © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.

## 6. The Series Safety Ground Rules

These six rules apply to **every** demo from Chapter 5 to Chapter 11. We state them once here, and later chapters simply say "follow Ch 4 §6's ground rules." Please read them slowly — they exist to protect both you and the motor.

1. **Start with no load.** The first time you try any mode, the output shaft must be bare and the motor fixed to the bench. Attach a load only when a demo specifically tells you to.
2. **Start small, and convert your units first.** Begin with gentle commands: a low duty, 1 A or less, a few thousand ERPM, position targets close by. And always convert units *before* sending (Chapter 3) — remember, 1,000 ERPM is only ~5.3 rpm, and the wide protocol ranges are **not** permission to use those values (Chapter 2 §9).
3. **Know where Stop is.** Before sending any command, find the Stop button in CubeMarsTool (§3, area G), and keep the power supply's off switch within arm's reach. Also remember: plain Position Mode and the "run to zero" button move at **maximum speed**.
4. **Watch the temperatures.** Keep the Real-time Data page open. Motor and MOSFET temperatures rise slowly under sustained current — especially when the motor is stalled or holding a position (Chapter 6 §5, Chapter 7 §5). If the temperature keeps climbing, stop and let the motor cool down.
5. **Respect the ratings.** While learning, stay well inside Chapter 1 §2's table: no more than 12 A sustained; treat 22 N·m / 28 A as short bursts only; never go above 52 V.
6. **Keep hands, hair, and cables clear.** Treat every powered motor as if it could move at any moment — because it can. A leftover command, a mode default, or a maximum-speed move can start motion instantly.

## Where This Chapter Is Used Next

This chapter ends Part I. Everything after it assumes a connected, calibrated, healthy motor:

| What we set up here | Where it is used |
|---|---|
| The mode map and CAN IDs 0–6, 8 (§1) | Every mode chapter's protocol section, and Ch 12 |
| General Control fields T/P/I/S/B/duty (§3) | Ch 5 (duty), Ch 6 (I and T), Ch 7 (B), Ch 8 (S), Ch 9 (P) |
| Trap Control Multi/Single (§3) | Ch 10 |
| The MIT Control panel (§3) | Ch 11 |
| Real-time Data plots as our "instrument" (§3) | Telemetry screenshots throughout Chs 5–11 |
| "Calibrated per Ch 4" starting condition (§5) | The first line of every demo block |
| The ground rules (§6) | Referenced as "Ch 4 §6's ground rules" in every demo |

## Key Takeaways

- There are two families: **Servo Control** (six one-purpose modes, CAN IDs 0–6, plus the Set Origin helper at ID 5) and **MIT / Force Control** (one unified mode at ID 8, whose behavior depends on the data you send). This series calls MIT's three uses MIT Position, MIT Velocity, and MIT Torque.
- CubeMarsTool is our cockpit: Configuration (where calibration lives), Real-time Status (health), Real-time Data (the live plots most demos depend on), Control (Trap Control, General Control, MIT panel), Connection — and the **Stop** button.
- A healthy power-up looks like this: blue light steady; green and red flash for ~2 seconds and go out. A red light that stays on means a fault.
- Calibration (Read → Motor Identification → Encoder Identification → Write) is needed only after reinstalling the driver board, changing the phase-wire order, or updating firmware. It must be done with **no load on the shaft**, and it should not be repeated back-to-back (it heats the motor).
- The six ground rules in §6 — no load first, small commands, Stop within reach, watch temperatures, respect ratings, hands clear — are assumed by every demo from Chapter 5 onward.

## Safety Notes

- Calibrate **only** with a completely bare output shaft. Calibrating under load can give wrong parameters or damage the motor, and repeating encoder identification heats the motor quickly (manual §3.2.1, quoted in §5).
- If the red fault light stays on, or Real-time Status shows a fault, stop and find the cause before commanding anything.
- Plain Position Mode (Ch 9) and the "run to zero position" button move at **maximum speed and acceleration** — never use them casually, and never with a load you have not planned for.
- All six rules of §6 apply to every chapter that follows.

## Sources / References

1. **CubeMars AK Series Module Product Manual, Ver. 3.0.1 (2025.03.14)** — specifically: §1.3 (p. 10: driver indicator-light definitions and the normal power-on behavior); §3.1.1–3.1.7 (p. 14–23: the upper-computer interface — Configuration Basic/Advanced settings and their warnings, Real-time Status, Real-time Data fields, the Control areas including Trap Control Multi ±36,000° / Single 0–359° and the run-to-zero maximum-speed caution, Unit Settings, Connection, Stop); §3.2/§3.2.1 (p. 23–24: when recalibration is needed, calibration Steps 0–4, and the no-load and heating warnings); §3.3.1–3.3.2 (p. 24–29: per-mode GUI operation — duty default 0.005–0.95, velocity S ±50,000 ERPM, Position Mode's maximum speed/acceleration, MIT position/velocity/torque operation); §4.1 (p. 31: the six Servo modes, their definitions, the extended-frame ID layout, Control Mode IDs 0–6); §4.2 (p. 37–38: Force Control shares Control Mode ID 8; the manual's "Position/Velocity/Current Loop" names for MIT's three uses).
2. **CubeMars AK80-9 V3.0 KV100 product specifications**, cubemars.com (accessed August 2026) — used through this series' reference table (Ch 1 §2) and unit conversions (Ch 3).
3. **CubeMarsTool upper-computer software** — all GUI descriptions and the CubeMarsTool screenshots shown in Figures 4.3–4.5; © CubeMars / Nanchang Kude Intelligent Technology Co., Ltd.

---

[← Back to Contents](00_Contents.md) | [Next Lesson: Chapter 5 — Duty Cycle Mode →](05_Duty_Cycle_Mode.md)
