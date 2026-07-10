# WeRoCon Motor Control Research Poster
## 48" × 36" Professional Poster Design

---

## 📋 Poster Specifications

**Format:** Landscape  
**Dimensions:** 48 inches wide × 36 inches tall  
**Resolution (300 DPI):** 14,400 × 10,800 pixels  
**Recommended Tools:** PowerPoint, Canva, Adobe Illustrator, or LaTeX (Beamer)  
**Color Scheme:** Dark theme with accent colors  

---

## 🎨 Design Layout

### Overall Structure (3-Column Layout)

```
┌─────────────────────────────────────────────────────────────┐
│                    HEADER (Full Width)                      │
│     ⚙️ WeRoCon Group - Advanced Motor Control             │
│     Real-time Trajectory Tracking Using CubeMars AK80-9   │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┬──────────────────┬──────────────────┐
│   LEFT COLUMN    │  CENTER COLUMN   │  RIGHT COLUMN    │
│   32% width      │   32% width      │   32% width      │
├──────────────────┼──────────────────┼──────────────────┤
│ • Project        │ • Control        │ • Key            │
│   Overview       │   Architecture   │   Innovations    │
│ • Hardware       │ • Performance    │ • Technical      │
│ • Software       │   Results        │   Parameters     │
└──────────────────┴──────────────────┴──────────────────┘

┌─────────────────────────────────────────────────────────────┐
│           BOTTOM SECTION (Full Width)                       │
│                Conclusion & Impact                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 Content Breakdown

### LEFT COLUMN (1/3 width)

#### 1. Project Overview
- **Title:** "🎯 Project Overview"
- **Box Style:** Rounded rectangle, accent1 (Blue) top border
- **Content:**
  ```
  Development of real-time trajectory tracking control systems 
  for CubeMars AK80-9 actuators in prosthetic joint applications.
  
  [Highlight Box in accent3 (Orange)]
  Goal: Implement high-fidelity gait trajectory tracking using 
  Winter dataset references with sub-2° RMS error.
  ```

#### 2. Hardware Stack
- **Title:** "🔧 Hardware Stack"
- **Content:** Bullet list
  - CubeMars AK80-9 V3 (9:1 gear ratio)
  - Raspberry Pi 4
  - Waveshare CAN HAT
  - 40V Battery System
  - Emergency Stop System

#### 3. Software Stack
- **Title:** "💻 Software Stack"
- **Content:**
  - **Languages:** Python (92.8%), MATLAB (7.2%)
  - **Libraries:** OpenSourceLeg, NumPy/Pandas, Matplotlib, python-can

---

### CENTER COLUMN (1/3 width)

#### 1. Control Architecture
- **Title:** "🎮 Control Architecture"
- **Content:**
  ```
  Three Implementation Modes:
  • MIT Mode: Low-latency impedance control (100 Hz)
  • Servo Mode: High-level velocity/position control
  • Dynamic Calibration: Relative joint zero-point setup
  
  Controller Strategy:
  • PID + Feedforward architecture
  • Lead-time compensation (0.10-0.13s)
  • Velocity filtering (Exponential Moving Average)
  • Slew-rate limiting for smooth commands
  ```

#### 2. Performance Results Table
- **Title:** "📊 Performance Summary"
- **Table Format:**
  ```
  ┌────────────────┬──────────┬─────────┐
  │ Metric         │ Knee     │ Ankle   │
  ├────────────────┼──────────┼─────────┤
  │ ROM            │ 64.7°    │ 29.5°   │
  │ RMS Error      │ 1.85°    │ 1.85°   │
  │ Peak Error     │ 9.21°    │ 9.21°   │
  │ Peak Current   │ 2.27 A   │ 2.27 A  │
  │ Peak Torque    │ 0.26 Nm  │ 0.26 Nm │
  └────────────────┴──────────┴─────────┘
  ```

---

### RIGHT COLUMN (1/3 width)

#### 1. Key Innovations
- **Title:** "✨ Key Innovations"
- **Content:** Bullet list with brief descriptions
  - **Circular Smoothing:** Eliminates start-end gait discontinuities
  - **Lead-Time Compensation:** 0.10-0.13s predictive control
  - **Multi-Mode Architecture:** Flexible control from MIT to high-level API
  - **Dynamic Calibration:** Automatic joint zero-point on boot
  - **Winter Dataset Integration:** Bio-realistic reference trajectories

#### 2. Controller Parameters
- **Title:** "⚡ Controller Tuning"
- **Content:** Two-part parameter table
  ```
  KNEE (v21):
  KP=50, KI=3, KD=1.5
  VEL_SCALE=18, ACC_SCALE=0.35
  LEAD_TIME=0.10s
  
  ANKLE (v24):
  KP=55, KI=3, KD=1.1
  VEL_SCALE=16.5, ACC_SCALE=0.28
  LEAD_TIME=0.13s
  ```

---

### BOTTOM SECTION (Full Width)

#### Conclusion & Impact
- **Title:** "🏆 Conclusion & Impact"
- **Height:** 2-3 cm
- **Content:**
  ```
  Successfully implemented robust trajectory tracking for 
  prosthetic joints achieving 1.85° RMS error with natural 
  gait patterns. Current work focuses on load-bearing 
  validation and multi-joint coordination.
  
  Impact: Advances in bio-realistic motor control enable 
  next-generation prosthetic devices with improved user 
  experience and natural motion fidelity.
  ```

---

## 🎨 Color Palette

| Name | RGB Value | Hex Code | Usage |
|------|-----------|----------|-------|
| Dark Background | (15, 17, 23) | #0F1117 | Poster background |
| Light Background | (26, 29, 39) | #1A1D27 | Content boxes |
| Accent 1 (Blue) | (79, 163, 224) | #4FA3E0 | Titles, borders |
| Accent 2 (Green) | (79, 206, 130) | #4FCE82 | Highlights |
| Accent 3 (Orange) | (240, 160, 80) | #F0A050 | Key points |
| Accent 4 (Cyan) | (79, 239, 239) | #4FEFEF | Metrics |
| Text Color | (200, 204, 216) | #C8CCD8 | Body text |

---

## 📐 Typography

- **Header Title:** 60-72pt, Bold, Accent 1 (Blue)
- **Subtitle:** 36-40pt, Regular, Accent 2 (Green)
- **Section Titles:** 28-32pt, Bold, Accent 1
- **Body Text:** 16-18pt, Regular, Text Color
- **Small Text:** 12-14pt, Regular, Text Color

---

## 📸 Image Placement

### Recommended Image Locations:

1. **Knee-Ankle Diagram** (Top-center optional)
   - Size: ~800×600px
   - Location: Can be placed in a 4th column or as background accent

2. **Knee Trajectory Graph** (Center background accent)
   - Size: ~1000×600px (20% opacity)
   - Location: Behind center column for visual interest

3. **Ankle Trajectory Graph** (Right background accent)
   - Size: ~1000×600px (20% opacity)
   - Location: Behind right column

*Note: Keep opacity low (15-20%) so text remains readable*

---

## 🔧 Implementation Instructions

### Option 1: Microsoft PowerPoint
1. Create new presentation
2. Set slide size to 48" × 36" (Slide → Slide Size → Custom)
3. Set background to dark gray (#0F1117)
4. Create 3 text boxes for columns
5. Add colored borders/rectangles for accent
6. Insert images with opacity

### Option 2: Canva
1. Create custom 48×36" design
2. Use Canva's design templates as starting point
3. Import color palette
4. Add text blocks and images
5. Export as high-resolution PNG/PDF

### Option 3: LaTeX (Beamer)
1. Use provided `RESEARCH_POSTER_48x36.tex` file
2. Install beamerposter package: `apt install texlive-fonts-recommended`
3. Compile: `pdflatex RESEARCH_POSTER_48x36.tex`
4. Output: PDF file suitable for printing

### Option 4: Adobe Illustrator
1. Create new document: 48" × 36" @ 300 DPI
2. Design layout using guides
3. Add color swatches from palette
4. Import images at high resolution
5. Export as PDF for printing

---

## ✅ Quality Checklist

- [ ] Text is readable from 6+ feet away
- [ ] All titles use accent colors
- [ ] No text is smaller than 12pt
- [ ] Images are high resolution (300 DPI)
- [ ] Color contrast meets accessibility standards
- [ ] All sections have clear headers with icons
- [ ] Spacing is balanced across all columns
- [ ] No content extends beyond safe margin (0.5" from edges)
- [ ] Footer is present with attribution
- [ ] Overall design matches WeRoCon branding

---

## 📥 Printing Instructions

**File Format:** PDF or high-resolution image (14,400×10,800 pixels)  
**Color Mode:** CMYK (for professional printing)  
**Paper Type:** Matte or glossy poster paper  
**Lamination:** Optional (protects from wear)  
**Print Service:** Use professional printing service for best quality  

---

## 🎓 References

- Winter, D. A. (1990). Biomechanics and Motor Control of Human Movement.
- CubeMars AK80-9 Documentation
- OpenSourceLeg API Documentation
- Gait Analysis Standards

---

**Last Updated:** 2026-07-10  
**Repository:** sauravk-RI/WeRoCon-Group  
**Contact:** WeRoCon Motor Control Research Lab
