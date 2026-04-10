#pragma once
#include <cmath>
#include <cstdio>
#include <cstring>

// Hand-rolled CNN inference for runic symbol classification.
// Architecture: Conv2D(32,3x3) -> MaxPool(2x2) -> Conv2D(64,3x3) -> MaxPool(2x2)
//               -> Dense(128,relu) -> Dense(6,softmax)
// Input: 32x32 grayscale image normalized to [0,1]
// Output: 1-6 (symbol class), 0 if confidence < 0.85, -1 on error

struct RunicWeights {
    float conv1_k[3*3*1*32];   // (3,3,1,32)
    float conv1_b[32];
    float conv2_k[3*3*32*64];  // (3,3,32,64)
    float conv2_b[64];
    float d1_w[2304*128];      // (2304,128)
    float d1_b[128];
    float d2_w[128*6];         // (128,6)
    float d2_b[6];
};

inline bool runic_load_weights(RunicWeights& w, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    bool ok =
        fread(w.conv1_k, sizeof(float), 3*3*1*32,   f) == 3*3*1*32   &&
        fread(w.conv1_b, sizeof(float), 32,          f) == 32          &&
        fread(w.conv2_k, sizeof(float), 3*3*32*64,   f) == 3*3*32*64  &&
        fread(w.conv2_b, sizeof(float), 64,          f) == 64          &&
        fread(w.d1_w,    sizeof(float), 2304*128,    f) == 2304*128    &&
        fread(w.d1_b,    sizeof(float), 128,         f) == 128         &&
        fread(w.d2_w,    sizeof(float), 128*6,       f) == 128*6       &&
        fread(w.d2_b,    sizeof(float), 6,           f) == 6;

    fclose(f);
    return ok;
}

// --- Forward pass primitives ---

// Conv2D valid padding, 3x3 kernel, ReLU activation
// Input/output layout: [H][W][C] (row-major, channels last — matches Keras)
static void conv2d_relu(
    const float* in, int H, int W, int Cin,
    const float* kernel, const float* bias, int Cout,
    float* out)
{
    int OH = H - 2, OW = W - 2;
    for (int oc = 0; oc < Cout; oc++) {
        for (int oh = 0; oh < OH; oh++) {
            for (int ow = 0; ow < OW; ow++) {
                float s = bias[oc];
                for (int ic = 0; ic < Cin; ic++)
                    for (int kh = 0; kh < 3; kh++)
                        for (int kw = 0; kw < 3; kw++)
                            s += in[(oh+kh)*W*Cin + (ow+kw)*Cin + ic]
                               * kernel[kh*3*Cin*Cout + kw*Cin*Cout + ic*Cout + oc];
                out[oh*OW*Cout + ow*Cout + oc] = s > 0.f ? s : 0.f;
            }
        }
    }
}

// MaxPool 2x2 stride 2 (odd dimensions: last row/col dropped, same as Keras)
static void maxpool2d(const float* in, int H, int W, int C, float* out) {
    int OH = H / 2, OW = W / 2;
    for (int c = 0; c < C; c++)
        for (int oh = 0; oh < OH; oh++)
            for (int ow = 0; ow < OW; ow++) {
                float m = -1e30f;
                for (int ph = 0; ph < 2; ph++)
                    for (int pw = 0; pw < 2; pw++) {
                        float v = in[(oh*2+ph)*W*C + (ow*2+pw)*C + c];
                        if (v > m) m = v;
                    }
                out[oh*OW*C + ow*C + c] = m;
            }
}

// Dense layer. relu=true applies ReLU, false leaves raw (use for final logits)
// Keras dense weight layout: [in][out]
static void dense_layer(
    const float* in, int Cin,
    const float* w, const float* b, int Cout,
    float* out, bool relu)
{
    for (int o = 0; o < Cout; o++) {
        float s = b[o];
        for (int i = 0; i < Cin; i++)
            s += in[i] * w[i*Cout + o];
        out[o] = (relu && s < 0.f) ? 0.f : s;
    }
}

// --- Main classify function ---
// img_norm: flat 32*32 floats, normalized [0,1], row-major GRAYSCALE
//           Input MUST be grayscale. If your canvas is RGBA/RGB, convert first:
//           gray = 0.299f * R + 0.587f * G + 0.114f * B  (all channels normalized 0-1)
// Returns: 1-6 (symbol), 0 (low confidence), -1 (null input)
// NOTE: uses static intermediate buffers — not thread-safe as-is
inline int runic_classify(const RunicWeights& wts, const float* img_norm) {
    if (!img_norm) return -1;

    // Intermediate buffers sized for worst-case output at each stage
    static float buf_conv1[30*30*32];  // after conv1:  30x30x32
    static float buf_pool1[15*15*32];  // after pool1:  15x15x32
    static float buf_conv2[13*13*64];  // after conv2:  13x13x64
    static float buf_pool2[6*6*64];    // after pool2:   6x6x64
    static float buf_d1[128];
    static float logits[6];

    // img_norm is 32x32x1 — channels-last with C=1 matches our layout
    conv2d_relu(img_norm, 32, 32, 1,  wts.conv1_k, wts.conv1_b, 32, buf_conv1);
    maxpool2d  (buf_conv1, 30, 30, 32, buf_pool1);
    conv2d_relu(buf_pool1, 15, 15, 32, wts.conv2_k, wts.conv2_b, 64, buf_conv2);
    maxpool2d  (buf_conv2, 13, 13, 64, buf_pool2);
    // flatten is implicit — buf_pool2 is already contiguous
    dense_layer(buf_pool2, 6*6*64, wts.d1_w, wts.d1_b, 128, buf_d1,  true);
    dense_layer(buf_d1,    128,    wts.d2_w, wts.d2_b,  6,  logits, false);

    // Softmax
    float max_v = logits[0];
    for (int i = 1; i < 6; i++) if (logits[i] > max_v) max_v = logits[i];
    float sum = 0.f;
    for (int i = 0; i < 6; i++) { logits[i] = expf(logits[i] - max_v); sum += logits[i]; }
    for (int i = 0; i < 6; i++) logits[i] /= sum;

    int best = 0;
    for (int i = 1; i < 6; i++) if (logits[i] > logits[best]) best = i;

    return logits[best] < 0.85f ? 0 : best + 1; // +1 because classes are 1-indexed
}

// --- Convenience wrappers for color inputs ---
// These convert to grayscale internally then call runic_classify.
// src: flat pixel array, w*h pixels, values 0-255 (bytes)

// RGB — 3 bytes per pixel
inline int runic_classify_rgb(const RunicWeights& wts,
    const unsigned char* src, int w, int h)
{
    if (!src) return -1;
    float gray[w * h];
    for (int i = 0; i < w * h; i++)
        gray[i] = (0.299f * src[i*3+0] + 0.587f * src[i*3+1] + 0.114f * src[i*3+2]) / 255.f;

    // Resize to 32x32 via bilinear if needed, then classify
    if (w == 32 && h == 32)
        return runic_classify(wts, gray);

    float img32[1024];
    float rw = (float)(w - 1) / 31.f, rh = (float)(h - 1) / 31.f;
    for (int oy = 0; oy < 32; oy++)
        for (int ox = 0; ox < 32; ox++) {
            float sx = ox * rw, sy = oy * rh;
            int ix = (int)sx, iy = (int)sy;
            float fx = sx - ix, fy = sy - iy;
            int ix1 = (ix+1 < w) ? ix+1 : ix, iy1 = (iy+1 < h) ? iy+1 : iy;
            img32[oy*32+ox] =
                gray[iy*w+ix]  *(1-fx)*(1-fy) + gray[iy*w+ix1]  *fx*(1-fy) +
                gray[iy1*w+ix] *(1-fx)*fy     + gray[iy1*w+ix1] *fx*fy;
        }
    return runic_classify(wts, img32);
}

// RGBA — 4 bytes per pixel (alpha ignored)
inline int runic_classify_rgba(const RunicWeights& wts,
    const unsigned char* src, int w, int h)
{
    if (!src) return -1;
    float gray[w * h];
    for (int i = 0; i < w * h; i++)
        gray[i] = (0.299f * src[i*4+0] + 0.587f * src[i*4+1] + 0.114f * src[i*4+2]) / 255.f;

    if (w == 32 && h == 32)
        return runic_classify(wts, gray);

    float img32[1024];
    float rw = (float)(w - 1) / 31.f, rh = (float)(h - 1) / 31.f;
    for (int oy = 0; oy < 32; oy++)
        for (int ox = 0; ox < 32; ox++) {
            float sx = ox * rw, sy = oy * rh;
            int ix = (int)sx, iy = (int)sy;
            float fx = sx - ix, fy = sy - iy;
            int ix1 = (ix+1 < w) ? ix+1 : ix, iy1 = (iy+1 < h) ? iy+1 : iy;
            img32[oy*32+ox] =
                gray[iy*w+ix]  *(1-fx)*(1-fy) + gray[iy*w+ix1]  *fx*(1-fy) +
                gray[iy1*w+ix] *(1-fx)*fy     + gray[iy1*w+ix1] *fx*fy;
        }
    return runic_classify(wts, img32);
}
