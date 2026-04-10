#pragma once
#include "canvas_iface.h"

// SimpleCanvas<W, H> — flat float array backend for IDrawCanvas.
//
// Use this in the standalone draw tool or for testing.
//
// In a real engine, implement IDrawCanvas against your PixelCell world instead:
//
//   class EngineWorldCanvas : public IDrawCanvas {
//       PixelCell** world; int w, h;
//   public:
//       EngineWorldCanvas(PixelCell** world, int w, int h)
//           : world(world), w(w), h(h) {}
//       int   width()  const override { return w; }
//       int   height() const override { return h; }
//       float get(int x, int y) const override { return world[y][x].ink; }
//       void  set(int x, int y, float v) override { world[y][x].ink = v; }
//       void  clear(float fill) override {
//           for (int y = 0; y < h; y++)
//               for (int x = 0; x < w; x++)
//                   world[y][x].ink = fill;
//       }
//   };

template<int W, int H>
class SimpleCanvas : public IDrawCanvas {
    float _cells[H][W];
public:
    SimpleCanvas() { clear(); }

    int   width()  const override { return W; }
    int   height() const override { return H; }
    float get(int x, int y) const override { return _cells[y][x]; }
    void  set(int x, int y, float v) override { _cells[y][x] = v; }
    void  clear(float fill = 1.f) override {
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                _cells[y][x] = fill;
    }
};
