# PTO Jitter Audit — ArfBot Teensy Firmware

**Branch:** `fix/pto-jitter-slow-motion`  
**Scope:** `Robots/AR4/Arduino/ArfBot` (Teensy pulse-train output for stepper drives)  
**Symptom:** Jittery movement during slow coordinated multi-axis motion  
**Date:** 2026-08-05

---

## System context

```text
CODESYS SoftMotion  →  velocity (deg/s)
        ↓
LREAL_TO_INT(vel * VelocityFactor / 360)  →  int16 frequency (Hz)
        ↓
EtherCAT process data (task ~4 ms)
        ↓
Teensy ArfBot sketch  →  software PTO  →  STEP/DIR to stepper drives
        ↑
Quadrature encoders (feedback to SoftMotion position loop)
```

The PLC sends a **signed integer frequency (Hz)** per axis. The Teensy converts that into a **50% duty step train** by polling the ARM cycle counter (`ARM_DWT_CYCCNT`) and toggling GPIO.

That design can work, but several details in the previous implementation made **low-speed multi-axis** motion especially sensitive to irregular step timing.

---

## Root causes (pre-fix)

### 1. Pulse generation gated on CRC success (high impact)

`Drive[i].run()` was only called inside the CRC-valid branch of `handleRx()`.

If a process-data frame failed checksum:

- Step generation stopped for that entire main-loop iteration
- The square wave could freeze mid-period
- The next good frame resumed from an inconsistent phase

**Effect:** intermittent stalls / phase jumps — often felt as jitter when steps are sparse (slow motion).

### 2. Software timer rebased to “now” (high impact)

Previous edge logic:

```cpp
if (elapsed > halfPeriod) {
  _sysTimeLast = _sysTime;   // snap to now
  toggle(step);
}
```

Every edge absorbed the residual main-loop latency at the moment the threshold was crossed. That latency varies because of:

- EasyCAT SPI (`MainTask`)
- Encoder interrupts (`ENCODER_OPTIMIZE_INTERRUPTS`, 12 lines)
- TX packing and CRC
- Digital input debounce reads

**Effect:** step-to-step period carried loop timing noise. At low Hz, each gap is long, so absolute timing error is very audible/visible.

### 3. Single edge per `run()` call — no catch-up (medium)

If the loop was late by multiple half-periods, only one edge was produced and time was rebased. That drops edges under load and distorts duty cycle / average rate.

### 4. Fully software bit-banged PTO on a busy loop (architectural)

Edges only occurred when `loop()` reached `run()`. Pulse timing shared the CPU with SPI and encoder IRQs. Hardware timers (or a fixed-rate step ISR) would isolate edge generation from that load.

### 5. Direction pin rewritten every call (medium)

DIR was written every `run()` invocation, including mid-pulse and on zero-crossings, without ensuring the step line was idle first. Drivers typically need DIR setup time before the next active edge.

### 6. Float period math every call (low–medium)

Period was computed with `float` and compared against a `uint32_t` cycle delta. Integer cycle math is more deterministic and cheaper.

### 7. PLC-side amplifiers (out of Arduino scope, still relevant)

| Factor | Why it matters at low speed |
|--------|-----------------------------|
| `LREAL_TO_INT` Hz quantization | Near-zero deadband; coarse velocity steps |
| EtherCAT update ~4 ms | Velocity stair-cased between cycles |
| SoftMotion position loop + encoders | Hunting / limit cycles if feedback is noisy |
| Stepper EMI on encoder lines | Documented in `Robots/AR4/Electrical/README.md` |

Example (J1, `VelocityFactor = 16000`):

| Joint speed | Theoretical Hz | After `LREAL_TO_INT` |
|-------------|----------------|----------------------|
| 1.0 °/s     | 44.4           | 44                   |
| 0.1 °/s     | 4.44           | 4                    |
| 0.02 °/s    | 0.89           | **0 (stopped)**      |

---

## Why slow coordinated motion is worst

| Factor | Why multi-axis slow motion suffers |
|--------|-------------------------------------|
| Sparse steps | Timing error is a large fraction of the step period |
| Variable joint rates | Six independent software timers share one jittery loop |
| Integer Hz + position loop | Hunting near zero velocity |
| CRC-gated `run()` | Freezes more obvious when edges are rare |
| Phase snap to “now” | Loop jitter becomes step-period jitter |

High-speed moves mask much of this via inertia and denser pulse trains.

---

## Changes on this branch (Arduino)

### `PTO.h` / `PTO.cpp`

1. **Phase-locked period advance**  
   `_sysTimeLast += _halfPeriodCycles` instead of `_sysTimeLast = now`.

2. **Limited multi-edge catch-up**  
   Up to `PTO_MAX_EDGES_PER_CALL` (default 8) half-period edges per `run()` so a late loop can recover without unbounded blocking.

3. **Integer half-period**  
   `halfPeriodCycles = F_CPU_ACTUAL / (2 * |frequency|)` via `uint64_t` math; recompute only when frequency changes.

4. **Safer DIR handling**  
   On sign change: force STEP idle (LOW), update DIR, re-base phase. On start-from-stop: ensure DIR is driven and phase is fresh.

5. **Explicit member initialization** in the constructor / `init()`.

### `ArfBot.ino`

1. **`handlePTO()`** always calls `Drive[i].run(Frequency[i].ival)` using the last good frequency.
2. **`handleRx()`** only updates `Frequency[]` when CRC is valid — it no longer owns pulse generation.
3. **`handlePTO()` is invoked twice per loop** (before EtherCAT/SPI work and after RX) so edges are not blocked behind a full SPI transaction.

---

## What this does *not* fix

- Hardware-timer / ISR-based step generation (still software bit-bang)
- Integer Hz resolution and SoftMotion gains on the PLC
- Encoder EMI (use 0.01 µF A/B filters if counts look noisy)
- Maximum step rate still limited by main-loop rate and `PTO_MAX_EDGES_PER_CALL`

---

## How to test

1. Flash this sketch to the Teensy (same procedure as wiki / AR4 setup).
2. **Single-axis slow jog** (e.g. 1–5% velocity) — listen for roughness vs previous firmware.
3. **Coordinated slow linear move** in tool/base coordinates at low path speed.
4. Optional: scope STEP on one axis — edges should show more uniform period under SPI load.
5. Optional SoftMotion trace: commanded velocity vs actual encoder rate; remaining low-speed hunting points to PLC quantization/loop, not PTO phase.

### Pass criteria (practical)

- Audible step roughness reduced on slow multi-axis moves
- No freezes / stutter correlated with EtherCAT noise (CRC path)
- Direction reversals clean (no extra step glitch at zero cross)

---

## Follow-up recommendations

**Medium term (Teensy)**

- Per-axis hardware timer or one high-rate ISR with phase accumulators (DDS / Bresenham)
- Keep EtherCAT path as “update target frequency only”

**PLC**

- Round instead of truncate when converting to Hz, or send higher-resolution frequency
- Review position-loop gains and deadband at low speed
- Confirm encoder feedback quality under load (EMI)

---

## Files touched

| File | Change |
|------|--------|
| `PTO.h` | Timing state, max-edges define, `applyFrequency` helper |
| `PTO.cpp` | Phase lock, integer period, DIR setup, catch-up |
| `ArfBot.ino` | `handlePTO()`, CRC decoupled from pulse gen |
| `PTO-Jitter-Audit.md` | This report |

---

## Questions that still help triage residual jitter

1. Is residual jitter **audible motor roughness** or **Cartesian path hesitation**?
2. Single-axis slow jog vs multi-axis only?
3. Approximate speed (°/s or % of max) where it starts/stops?
4. Do SoftMotion encoder traces look smooth when motion feels bad?
