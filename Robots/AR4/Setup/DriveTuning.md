# Drive Tuning

ArfBotOS uses CODESYS SoftMotion `SM_Drive_PosControl`. The PLC closes the position loop: it sends a velocity command to the Teensy / stepper drives and reads encoder position back over EtherCAT.

`DeadTime` (`D`) is **not milliseconds**. It is the number of **MainTask / SoftMotion bus cycles** that the actual position lags the setpoint.

## What "cycle time" means

Official CODESYS wording:

> Determine the time difference between the start movement of the set position and the first reaction of the actual position.  
> `D = time difference / cycle time`

That cycle time is the **SoftMotion axis bus cycle**, not EtherCAT DC and not the visu refresh.

In this project:

| Clock | Configured value | Role |
|---|---|---|
| `MainTask` | 1 ms | SoftMotion bus cycle. **This is the cycle `D` is counted in.** |
| `GVL.dwMainInterval` | microseconds (`1000` = 1.0 ms) | Live MainTask interval, already measured in `_00_Main` |
| EtherCAT `MasterCycleTime` | 4000 µs (4 ms) | How often process data is exchanged with the EMCU |

Do **not** divide by the 4 ms EtherCAT cycle. That underestimates `D` by about 4x.

Example: if the graph shows actual starting 10 ms after setpoint, and MainTask is 1.0 ms:

```text
D = 10 ms / 1.0 ms = 10.0 cycles
```

Measure the **horizontal time gap at the start of motion**, not the vertical following-error gap while the axis is already moving.

## Recommended first pass

START TUNE now uses the official SoftMotion function block `SMC_EstimateDeadTime` (`SM3_Basic`).

CODESYS requires **constant velocity**. The existing 5° `MC_MoveRelative` still runs (so the axis does not run away). After the move reaches cruise, the PLC enables `SMC_EstimateDeadTime`, collects up to 64 `DeadTimeCycles` samples, and takes the **median**.

1. Set `Kp` to `0` and write parameters.
2. Select the axis and press **START TUNE**.
3. After the move, the help line shows the median and a suggested `D`.
4. Copy that into **DeadTime (cycles)**, or toggle `_00_Main.UseSuggestedD[axis]`.
5. Press **WRITE PARAMETERS**.
6. Raise `Kp` slowly until the trace just starts to oscillate, then drop it about 10%.

`D` must be in `(0, 20]`. SoftMotion rejects `0` and anything above `20` (`SMC_INVALID_PARAMETER`: "The value of one of the inputs is invalid."). The tuner clamps to `20` and still shows the raw median.

| Variable | Meaning |
|---|---|
| `_00_Main.EstimateDeadTime[axis]` | Official FB instance |
| `_00_Main.fCycleTimeMs` | MainTask cycle, ms |
| `_00_Main.fEstimatedDeadTimeRaw[axis]` | Unclamped median, cycles |
| `_00_Main.fSuggestedDeadTime[axis]` | Clamped `D` for PosControl |
| `_00_Main.nDeadTimeSamples[axis]` | How many valid samples were used |
| `_00_Main.sDeadTimeHelp[axis]` | One-line result |

## HMI bindings to add

The PLC variables are ready. Bind these on the Tuning visu if they are not already on the screen:

| Element | Variable |
|---|---|
| Help line | `_00_Main.sDeadTimeHelp[AxisNumber-1]` |
| Cycle time (ms) | `_00_Main.fCycleTimeMs` |
| Raw median (cycles) | `_00_Main.fEstimatedDeadTimeRaw[AxisNumber-1]` |
| Suggested D | `_00_Main.fSuggestedDeadTime[AxisNumber-1]` |
| Sample count | `_00_Main.nDeadTimeSamples[AxisNumber-1]` |
| Use suggested D | `_00_Main.UseSuggestedD[AxisNumber-1]` |

WRITE PARAMETERS still pushes `PersistentVars.PositionControllers[].fDeadTime` through `SMC_SetPosControlParams`. Suggested `D` is never applied by itself.

## Why the graph alone is confusing

The Tuning trace plots `fSetPosition` (red) and `fActPosition` (blue) against wall-clock time. Those traces still show the **physical** delay even after `D` is set, because `D` only delays the setpoint inside the lag calculation.

So:

- Use the graph (or the PLC suggestion) to **measure** deadtime.
- After `D` is correct, the controller should track better, but the two traces may still be shifted by that same physical lag.

## Official Kp pass

With deadtime set:

1. Start `Kp` small (for example `0.0001` to `0.01`).
2. Increase until the trace oscillates.
3. Reduce about 10% and write parameters.
