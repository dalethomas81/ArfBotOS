# Robot Animator (CODESYS WebVisu HTML5 control)

A generic industrial 6-axis body (joint housings, arm beams, spherical wrist, tool flange) that follows `J1..J6` actual positions on the ArfBotOS WebVisu. Geometry is primitives, not AR4 CAD.

HTML5 controls render **only in WebVisu** (the Pi browser HMI). They need **Support client animations and overlay of native elements** on the Visualization Manager.

## Try it without CODESYS

From this folder:

```powershell
python -m http.server 8765
```

Open http://localhost:8765/TestControl.html — drag to orbit, wheel to zoom, sliders are joint degrees. MCS / PC1 / PC2 are X Y Z mm and A B C deg (Rx Ry Rz), all relative to **WCS** (SoftMotion). TCP is relative to the flange. **Demo offsets** places sample frames.

## Install into CODESYS

The descriptor `RobotAnimator.html5control.xml` must match the CODESYS Visualization 4.10 schema (official demo controls from forge.codesys.com). Do not hand-edit it into a simple `<Property>` list — the editor rejects that.

1. Close any open CODESYS project (repository install requires none).
2. Open **CODESYS HTML5 Control Editor**
   (`C:\Program Files\CODESYS 3.5.22.30\CODESYS\Common\Html5ControlEditor.exe`),
   or **Tools → Visualization Element Repository → HTML5 Controls → Open Editor**.
3. File → Open → `RobotAnimator.html5control.xml`.
4. Confirm General: Company `ArfBotOS`, Name `RobotAnimator` (no spaces), Category **Special Controls**, image `ElementImage.svg`, additional file `ElementWrapper.js`.
5. Control Properties should already list `J1`..`J6` and `Gripper` (`LREAL`, Variable editor, call methods `setJ1`..`setJ6` / `setGripper`, property type Initialize so live values stream).
6. **Save and Install**.

Alternatively skip the editor: **Tools → Visualization Element Repository → HTML5 Controls → Install** and pick the `.html5control.xml`.

Unsigned HTML5 controls warn on download. Trust the element when prompted (Visualization message category).

## Drop onto a visualization

An **Animator** visualization is already in `ArfBot.project` (placeholder rectangle). There is also a **RobotView** template under `Visu/templates`.

1. Visualization Manager → Settings: enable **Support client animations and overlay of native elements** (required for any HTML5 control). If you drop the control first, CODESYS shows *The overlay feature of the visualization manager is not enabled*. Click **OK**, not **Abort** (Abort has crashed the IDE). Then enable the checkbox and drop the control again.
2. Open **Animator** (or **Jogging** / **RobotView**).
3. Toolbox → Special Controls → **RobotAnimator**.
4. Size it (e.g. 480×400).
5. Bind:
   - J1 = `IoConfig_Globals.J1.fActPosition`
   - … through J6
   - Gripper = 0–100 (open %) or 0–1. BOOL also works (0 closed, 1 open).

HTML5 controls are not a `VisualElementType` enum member, so they cannot be inserted by the usual `add_element` scripting API. Install + drag in the IDE.

Debug in the browser: `http://<pi>:8080/webvisu.htm?CFG_DebugHTML5=true` then DevTools → Sources → `elementwrapper*.js`.

If Process View is a blank dark rectangle: the overlay slot is there but the canvas had no size (WebVisu iframe). Use control **0.0.0.2**, Save and Install, download the application, then hard-refresh the browser. Unsigned HTML5 files also stay blank until you **trust** the element (Visualization messages → signature warning → Yes).

## Kinematics

Default DH is AR4 millimetres in the SoftMotion 6-DOF convention (`d1` negative). J2 has a built-in +90° DH offset. J1–J6 inputs are kinematic joint angles — if this robot’s `fActPosition` differs (J4 reversed, J6 flange +90°), map that on the visu bindings or in the axis group, not inside the control.
