#include <SDL3/SDL.h>
#include <iostream>
#include <cstring>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "runic_classifier.h"
#include "canvas_iface.h"
#include "simple_canvas.h"

static const int CANVAS  = 64;
static const int SCALE   = 8;
static const int BRUSH_R = 3;
static const int INFO_H  = 48;
static const int REF_W   = 96;
static const int REF_SZ  = 32;
static const int WIN_W   = CANVAS * SCALE + REF_W;
static const int WIN_H   = CANVAS * SCALE + INFO_H;

static unsigned char ref_pixels[6][32 * 32];
static bool          ref_loaded[6] = {};

void load_references() {
    for (int s = 0; s < 6; s++) {
        char path[64];
        snprintf(path, sizeof(path), "dataset/%d/0.png", s + 1);
        int w, h, ch;
        unsigned char* data = stbi_load(path, &w, &h, &ch, 1);
        if (data) {
            for (int ry = 0; ry < 32; ry++)
                for (int rx = 0; rx < 32; rx++) {
                    int sy = (ry * h) / 32;
                    int sx = (rx * w) / 32;
                    ref_pixels[s][ry * 32 + rx] = data[sy * w + sx];
                }
            stbi_image_free(data);
            ref_loaded[s] = true;
        }
    }
}

void draw_reference_panel(SDL_Renderer* renderer, int predicted) {
    int panel_x = CANVAS * SCALE;

    SDL_SetRenderDrawColor(renderer, 25, 25, 25, 255);
    SDL_FRect bg = {(float)panel_x, 0, (float)REF_W, (float)WIN_H};
    SDL_RenderFillRect(renderer, &bg);

    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_RenderLine(renderer, panel_x, 0, panel_x, WIN_H);

    int spacing = (CANVAS * SCALE) / 6;

    for (int s = 0; s < 6; s++) {
        int sym_num  = s + 1;
        int center_y = s * spacing + spacing / 2;
        int img_x    = panel_x + (REF_W - 32) / 2;
        int img_y    = center_y - 16;

        if (sym_num == predicted) {
            SDL_SetRenderDrawColor(renderer, 80, 160, 80, 255);
            SDL_FRect hl = {(float)(img_x - 3), (float)(img_y - 3), 38, 38};
            SDL_RenderFillRect(renderer, &hl);
        }

        if (ref_loaded[s]) {
            for (int ry = 0; ry < 32; ry++) {
                for (int rx = 0; rx < 32; rx++) {
                    Uint8 v = ref_pixels[s][ry * 32 + rx];
                    SDL_SetRenderDrawColor(renderer, v, v, v, 255);
                    SDL_FRect px = {(float)(img_x + rx), (float)(img_y + ry), 1, 1};
                    SDL_RenderFillRect(renderer, &px);
                }
            }
        }
    }
}

int main() {
    RunicWeights weights;
    if (!runic_load_weights(weights, "runic_weights.bin")) {
        std::cerr << "Failed to load runic_weights.bin\n";
        return 1;
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window*   window   = SDL_CreateWindow("Runic Draw", WIN_W, WIN_H, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    // --- Canvas backend: swap this line to plug in a different backend ---
    SimpleCanvas<CANVAS, CANVAS> canvas;
    IDrawCanvas& cv = canvas;
    // In your engine: EngineWorldCanvas cv(world, W, H);
    // ---------------------------------------------------------------------

    load_references();

    int  result  = 0;
    bool drawing = false;
    bool running = true;

    SDL_FRect done_btn  = {10,  (float)(CANVAS * SCALE + 8), 80, 32};
    SDL_FRect clear_btn = {100, (float)(CANVAS * SCALE + 8), 80, 32};

    auto do_classify = [&]() {
        float input[32 * 32];
        canvas_to_input(cv, input);
        float dark = 0.f;
        for (int i = 0; i < 32 * 32; i++) dark += (1.f - input[i]);
        result = (dark < 1.f) ? 0 : runic_classify(weights, input);
        if (result == 0) std::cout << "No confident prediction.\n";
        else             std::cout << "Prediction: Symbol " << result << "\n";
    };

    SDL_Event e;
    while (running) {
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_EVENT_QUIT:
                running = false;
                break;

            case SDL_EVENT_MOUSE_BUTTON_DOWN: {
                float mx = e.button.x, my = e.button.y;
                if (mx >= clear_btn.x && mx <= clear_btn.x + clear_btn.w &&
                    my >= clear_btn.y && my <= clear_btn.y + clear_btn.h) {
                    cv.clear();
                    result = 0;
                } else if (mx >= done_btn.x && mx <= done_btn.x + done_btn.w &&
                           my >= done_btn.y && my <= done_btn.y + done_btn.h) {
                    do_classify();
                } else if (mx < CANVAS * SCALE && my < CANVAS * SCALE) {
                    drawing = true;
                    float val = (e.button.button == SDL_BUTTON_LEFT) ? 0.f : 1.f;
                    canvas_paint(cv, (int)mx / SCALE, (int)my / SCALE, BRUSH_R, val);
                }
                break;
            }

            case SDL_EVENT_MOUSE_BUTTON_UP:
                drawing = false;
                break;

            case SDL_EVENT_MOUSE_MOTION:
                if (drawing && e.motion.x < CANVAS * SCALE && e.motion.y < CANVAS * SCALE) {
                    SDL_MouseButtonFlags btns = SDL_GetMouseState(nullptr, nullptr);
                    float val = (btns & SDL_BUTTON_LMASK) ? 0.f : 1.f;
                    canvas_paint(cv, (int)e.motion.x / SCALE, (int)e.motion.y / SCALE, BRUSH_R, val);
                }
                break;

            case SDL_EVENT_KEY_DOWN:
                if (e.key.key == SDLK_RETURN || e.key.key == SDLK_KP_ENTER) {
                    do_classify();
                } else if (e.key.key == SDLK_C || e.key.key == SDLK_SPACE) {
                    cv.clear();
                    result = 0;
                }
                break;
            }
        }

        // --- Render ---
        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);

        // Canvas cells — read through interface
        for (int y = 0; y < CANVAS; y++)
            for (int x = 0; x < CANVAS; x++) {
                Uint8 v = (Uint8)(cv.get(x, y) * 255.f);
                SDL_SetRenderDrawColor(renderer, v, v, v, 255);
                SDL_FRect r = {(float)(x * SCALE), (float)(y * SCALE), (float)SCALE, (float)SCALE};
                SDL_RenderFillRect(renderer, &r);
            }

        // Subtle grid
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 30);
        for (int i = 0; i <= CANVAS; i++) {
            SDL_RenderLine(renderer, i * SCALE, 0, i * SCALE, CANVAS * SCALE);
            SDL_RenderLine(renderer, 0, i * SCALE, CANVAS * SCALE, i * SCALE);
        }

        // Info bar
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_FRect bar = {0, (float)(CANVAS * SCALE), (float)WIN_W, (float)INFO_H};
        SDL_RenderFillRect(renderer, &bar);

        // Done button
        SDL_SetRenderDrawColor(renderer, 60, 120, 60, 255);
        SDL_RenderFillRect(renderer, &done_btn);
        SDL_SetRenderDrawColor(renderer, 100, 200, 100, 255);
        SDL_RenderRect(renderer, &done_btn);

        // Clear button
        SDL_SetRenderDrawColor(renderer, 100, 60, 60, 255);
        SDL_RenderFillRect(renderer, &clear_btn);
        SDL_SetRenderDrawColor(renderer, 180, 80, 80, 255);
        SDL_RenderRect(renderer, &clear_btn);

        draw_reference_panel(renderer, result);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
