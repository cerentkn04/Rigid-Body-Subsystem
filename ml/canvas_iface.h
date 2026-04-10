#pragma once
#include <functional>

// IDrawCanvas — pluggable pixel canvas interface.
//
// Pixel convention: 1.0 = background (white/empty), 0.0 = full ink (black/stroke).
//
// Implement this to connect to any backend:
//   - SimpleCanvas<W,H>   : flat float array (this tool, prototyping)
//   - EngineWorldCanvas   : wraps PixelCell[][] world data in a real engine
//
// The free functions below (canvas_paint, canvas_to_input) operate purely
// through this interface — swap the backend without touching them.
//
// You don't need to subclass IDrawCanvas manually.
// Use canvas_bind() to plug in any backend with three lambdas:
//
//   auto cv = canvas_bind({world_w, world_h,
//       [&](int x, int y)          { return world[y][x].ink; },
//       [&](int x, int y, float v) { world[y][x].ink = v; },
//       [&](float fill)            { /* clear */ }
//   });

class IDrawCanvas {
public:
    virtual int   width()  const = 0;
    virtual int   height() const = 0;
    virtual float get(int x, int y) const = 0;
    virtual void  set(int x, int y, float value) = 0;
    virtual void  clear(float fill = 1.f) = 0;
    virtual ~IDrawCanvas() = default;
};

// Paint a circular brush at canvas cell (cx, cy) with given radius.
inline void canvas_paint(IDrawCanvas& c, int cx, int cy, int radius, float value) {
    int W = c.width(), H = c.height();
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
            if (dx*dx + dy*dy <= radius*radius) {
                int x = cx + dx, y = cy + dy;
                if (x >= 0 && x < W && y >= 0 && y < H)
                    c.set(x, y, value);
            }
}

// Sample canvas → 32x32 float array suitable for runic_classify().
// Finds bounding box of ink pixels (value < 0.5), adds ~15% padding,
// then bilinear-scales that region to 32x32.
// If nothing is drawn, fills output with 1.0 (blank).
inline void canvas_to_input(const IDrawCanvas& c, float out[32 * 32]) {
    int W = c.width(), H = c.height();

    int minx = W, maxx = -1, miny = H, maxy = -1;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (c.get(x, y) < 0.5f) {
                if (x < minx) minx = x;
                if (x > maxx) maxx = x;
                if (y < miny) miny = y;
                if (y > maxy) maxy = y;
            }

    if (maxx < 0) {
        for (int i = 0; i < 32 * 32; i++) out[i] = 1.f;
        return;
    }

    int span = (maxx - minx + 1 > maxy - miny + 1) ? maxx - minx + 1 : maxy - miny + 1;
    int pad  = (int)(0.15f * (float)span);
    if (pad < 2) pad = 2;

    int x0 = minx - pad; if (x0 < 0)    x0 = 0;
    int y0 = miny - pad; if (y0 < 0)    y0 = 0;
    int x1 = maxx + pad; if (x1 >= W)   x1 = W - 1;
    int y1 = maxy + pad; if (y1 >= H)   y1 = H - 1;

    float rw = (float)(x1 - x0) / 31.f;
    float rh = (float)(y1 - y0) / 31.f;

    for (int oy = 0; oy < 32; oy++) {
        for (int ox = 0; ox < 32; ox++) {
            float sx = x0 + ox * rw;
            float sy = y0 + oy * rh;
            int   ix  = (int)sx, iy  = (int)sy;
            float fx  = sx - ix, fy  = sy - iy;
            int   ix1 = (ix + 1 < W) ? ix + 1 : W - 1;
            int   iy1 = (iy + 1 < H) ? iy + 1 : H - 1;
            out[oy * 32 + ox] =
                c.get(ix,  iy)  * (1-fx) * (1-fy) +
                c.get(ix1, iy)  * fx     * (1-fy) +
                c.get(ix,  iy1) * (1-fx) * fy     +
                c.get(ix1, iy1) * fx     * fy;
        }
    }
}

// --- canvas_bind — plug in any backend without subclassing ---

struct CanvasDesc {
    int width, height;
    std::function<float(int x, int y)>       get;
    std::function<void(int x, int y, float)> set;
    std::function<void(float fill)>          clear;
};

class BoundCanvas : public IDrawCanvas {
    CanvasDesc _d;
public:
    BoundCanvas(CanvasDesc d) : _d(std::move(d)) {}
    int   width()  const override { return _d.width; }
    int   height() const override { return _d.height; }
    float get(int x, int y) const override { return _d.get(x, y); }
    void  set(int x, int y, float v) override { _d.set(x, y, v); }
    void  clear(float fill = 1.f) override { _d.clear(fill); }
};

// Returns a canvas backed by your lambdas.
// Use it anywhere IDrawCanvas& is expected.
//
// Example:
//   auto cv = canvas_bind({world_w, world_h,
//       [&](int x, int y)          { return world[y][x].ink; },
//       [&](int x, int y, float v) { world[y][x].ink = v; },
//       [&](float fill)            { for_each_cell([&](auto& c){ c.ink = fill; }); }
//   });
//   canvas_paint(cv, cx, cy, 3, 0.f);
//   canvas_to_input(cv, img32);
inline BoundCanvas canvas_bind(CanvasDesc desc) {
    return BoundCanvas(std::move(desc));
}
