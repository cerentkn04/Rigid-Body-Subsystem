# Runic Classifier — Integration Guide

## Files

| File | Purpose |
|---|---|
| `runic_classifier.h` | Classifier — the only file needed for inference |
| `runic_weights.bin` | Weight data, loaded at runtime |
| `canvas_iface.h` | Pluggable canvas — `canvas_bind`, `canvas_paint`, `canvas_to_input` |
| `simple_canvas.h` | Float array canvas backend (standalone / prototyping) |

---

## Classifier only

```cpp
#include "runic_classifier.h"

RunicWeights weights;
runic_load_weights(weights, "runic_weights.bin");  // once at startup

int result = runic_classify(weights, img32);  // img32: float[1024], 32x32 grayscale
                                              // 1.0=background, 0.0=ink
```

**Returns:** `1–6` symbol, `0` low confidence, `-1` null input.

---

## With the canvas subsystem

### Standalone (float array backend)

```cpp
#include "simple_canvas.h"
#include "runic_classifier.h"

SimpleCanvas<64, 64> cv;

canvas_paint(cv, cx, cy, 3, 0.f);   // draw ink (0.0) with radius 3

float img32[1024];
canvas_to_input(cv, img32);          // crop + scale to 32x32
int result = runic_classify(weights, img32);
```

### Engine / game world (plug in your pixel cells)

```cpp
#include "canvas_iface.h"

auto cv = canvas_bind({
    world_w, world_h,
    [&](int x, int y)          { return world[y][x].ink; },
    [&](int x, int y, float v) { world[y][x].ink = v; },
    [&](float fill)            { /* clear your cells */ }
});

canvas_paint(cv, cx, cy, 3, 0.f);
canvas_to_input(cv, img32);
int result = runic_classify(weights, img32);
```

`canvas_to_input` finds the bounding box of drawn strokes, adds padding, and bilinear-scales to 32×32 — no manual preprocessing needed.

---

## RGB / RGBA inputs (no canvas)

```cpp
int result = runic_classify_rgb (weights, pixels, width, height);  // 3 bytes/px
int result = runic_classify_rgba(weights, pixels, width, height);  // 4 bytes/px, alpha ignored
```

Both resize to 32×32 internally.

---

## Notes

- **Thread safety:** `runic_classify` uses static buffers — not thread-safe. Add a mutex if calling from multiple threads.
- **Confidence threshold:** hardcoded at `0.85` in `runic_classifier.h`. Lower for more hits, raise for stricter matching.
- **Inverted colors** (white stroke on black): `img[i] = 1.f - img[i]` before classifying.
