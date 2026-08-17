# ArfBot Night — CODESYS Visualization Style

A dark, flat visualization style for the ArfBotOS WebVisu. It is derived from the stock **Flat style 4.9.0.0 (CODESYS)** so lamps, switches, and other image-based controls keep working. Only colors, fonts, and a few chrome settings are overridden.

Open `mockup/index.html` in a browser to preview Main, Jog, and the PackML dialog.

## Palette

| Token | Hex | Used for |
|---|---|---|
| Canvas | `#0E141B` | Visualization background |
| Surface | `#171F2A` | Group boxes, text fields, dialogs |
| Button | `#243041` | Default buttons, combos, spin boxes |
| Border | `#2A3646` / `#3A4A5C` | Frames and group-box lines |
| Text | `#E8EEF5` | Default and button text |
| Muted | `#8B9BB0` | Labels, scales, secondary text |
| Accent | `#2EE6D6` | Selection, meters, highlight |
| Success | `#3DDC84` | Start / healthy |
| Warning | `#FFB020` | Held / complete / caution |
| Danger | `#FF4D6A` | Abort / alarm |

Named colors `Canvas`, `Surface`, `Accent`, `Success`, `Warning`, `Danger`, and `Muted` also appear in the element color dropdown after the style is applied.

## Install

1. In CODESYS: **Tools → Visualization Style Repository…**
2. **Install…** and select `ArfBot Night/1.0.0.0/styledef.xml`  
   (or the folder `ArfBot Night/1.0.0.0`)
3. Open **Visualization Manager → Settings → Style Settings**
4. Choose **ArfBot Night** and rebuild / download

To edit later, open `ArfBot Night.visustyle.xml` in the Visualization Style Editor (**Visualization Manager → Settings → Open Style Editor**), or use **Create and Edit Derived Style** and pick this style as the base.

Manual install (same result):

```
%ProgramData%\CODESYS\Visualization Styles\ArfBotOS\ArfBot Night\1.0.0.0\styledef.xml
```

Copy `1.0.0.0/styledef.xml` to that path, then restart CODESYS so the repository rescans.

## What the style changes vs. what it does not

CODESYS styles recolor **style-bound** properties: visualization background, default buttons, group boxes, text fields, tables, dialogs, fonts, and new elements.

They do **not** override colors that were set explicitly on an element. On the current ArfBotOS visu that includes:

- START (lime) and ABORT (pink)
- Lamp / limit fills
- The vision image frame
- Some PackML state fills

After applying the style, point those leftover elements at the new named colors (`Success`, `Danger`, `Accent`, `Surface`) so they match the mockup. No layout changes are required.

## Design notes

- Chrome is `STYLE9_FLAT` — no beveled Windows 7 look.
- Rounded-rect radius is 6 px. CODESYS cannot do CSS-style cards or drop shadows; the mockup stays inside those limits so the installed theme does not over-promise.
- Segoe UI is already used by Flat style. WebVisu on Raspberry Pi falls back to a sans-serif if Segoe is missing.
- A light sibling can be derived from this file later by swapping the canvas/surface/text tokens.
