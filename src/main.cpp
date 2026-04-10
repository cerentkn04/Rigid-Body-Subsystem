#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <random>
#include <algorithm>
#include "RigidPixelSystem.hpp"
#include "RigidPixelGrid.hpp"
#include "runic_classifier.h"
#include "canvas_iface.h"
#include "simple_canvas.h"

constexpr int WINDOW_WIDTH  = 800;
constexpr int WINDOW_HEIGHT = 600;
constexpr int CELL_SIZE     = 4;
constexpr int GRID_WIDTH    = WINDOW_WIDTH  / CELL_SIZE;
constexpr int GRID_HEIGHT   = WINDOW_HEIGHT / CELL_SIZE;

enum class CellType : uint8_t { Empty = 0, Sand, Water, Wood, Rock };
struct Cell {
    CellType type          = CellType::Empty;
    bool     updated       = false;
    uint8_t  colorVariation = 0;
    uint32_t object_id     = 0;
};

// ── Sand simulation ──────────────────────────────────────────────────────────
class SandSimulation {
public:
    rigid::RigidPixelGrid<Cell> grid;
    std::mt19937 rng;

    SandSimulation() : grid(GRID_WIDTH, GRID_HEIGHT), rng(std::random_device{}()) {}

    Cell& at(int x, int y)             { return grid.at(x, y); }
    const Cell& at(int x, int y) const { return grid.at(x, y); }
    bool inBounds(int x, int y) const  { return grid.in_bounds(x, y); }
    bool isEmpty(int x, int y) const   { return inBounds(x, y) && at(x, y).type == CellType::Empty; }

    bool canDisplace(int x, int y, CellType displacing) const {
        if (!inBounds(x, y)) return false;
        CellType t = at(x, y).type;
        return t == CellType::Empty || (displacing == CellType::Sand && t == CellType::Water);
    }

    void swap(int x1, int y1, int x2, int y2) {
        std::swap(at(x1, y1), at(x2, y2));
        at(x1, y1).updated = at(x2, y2).updated = true;
    }

    void updateSand(int x, int y) {
        if (at(x, y).updated) return;
        if (canDisplace(x, y+1, CellType::Sand)) { swap(x, y, x, y+1); return; }
        bool L = canDisplace(x-1, y+1, CellType::Sand), R = canDisplace(x+1, y+1, CellType::Sand);
        if (L && R) swap(x, y, x + ((rng()%2) ? -1 : 1), y+1);
        else if (L) swap(x, y, x-1, y+1);
        else if (R) swap(x, y, x+1, y+1);
    }

    void updateWater(int x, int y) {
        if (isEmpty(x, y+1)) { swap(x, y, x, y+1); return; }
        bool dL = isEmpty(x-1, y+1), dR = isEmpty(x+1, y+1);
        if (dL && dR) { swap(x, y, x + ((rng()%2) ? -1:1), y+1); return; }
        if (dL) { swap(x, y, x-1, y+1); return; }
        if (dR) { swap(x, y, x+1, y+1); return; }
        bool L = isEmpty(x-1, y), R = isEmpty(x+1, y);
        if (L && R) swap(x, y, x + ((rng()%2) ? -1:1), y);
        else if (L) swap(x, y, x-1, y);
        else if (R) swap(x, y, x+1, y);
    }

    void update() {
        for (auto& c : grid.cells) c.updated = false;
        for (int y = GRID_HEIGHT-1; y >= 0; --y) {
            bool ltr = (rng()%2) == 0;
            for (int i = 0; i < GRID_WIDTH; ++i) {
                int x = ltr ? i : (GRID_WIDTH-1-i);
                switch (at(x, y).type) {
                    case CellType::Sand:  updateSand(x, y);  break;
                    case CellType::Water: updateWater(x, y); break;
                    default: break;
                }
            }
        }
    }

    void placeCell(int x, int y, CellType type, int brushSize = 1, uint32_t instanceId = 0) {
        std::uniform_int_distribution<uint8_t> dist(0, 30);
        bool isSolid = (type == CellType::Rock || type == CellType::Wood);
        for (int dy = -brushSize+1; dy < brushSize; ++dy) {
            for (int dx = -brushSize+1; dx < brushSize; ++dx) {
                int nx = x+dx, ny = y+dy;
                if (!inBounds(nx, ny)) continue;
                CellType old = at(nx, ny).type;
                if (old == type) continue;
                at(nx, ny).type           = type;
                at(nx, ny).colorVariation = dist(rng);
                at(nx, ny).object_id      = isSolid ? instanceId : 0;
                bool solid_change = old == CellType::Rock || old == CellType::Wood
                                 || type == CellType::Rock || type == CellType::Wood;
                if (solid_change) grid.mark_solid_changed(nx, ny);
            }
        }
    }

    void clear() { grid.clear(Cell{}); }
};

// ── Helpers ──────────────────────────────────────────────────────────────────
SDL_Color get_id_color(rigid::RegionID id) {
    if (id == rigid::InvalidRegionID) return { 50, 50, 50, 255 };
    uint32_t h = id;
    h = ((h>>16)^h)*0x45d9f3b; h = ((h>>16)^h)*0x45d9f3b; h = (h>>16)^h;
    return { uint8_t(h), uint8_t(h>>8), uint8_t(h>>16), 255 };
}

// ── Entry point ──────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO)) return 1;
    SDL_Window*   window   = SDL_CreateWindow("Sand Falling Simulation", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    SandSimulation sim;

    // ── WorldView — 3 simple cell-level predicates, nothing else needed ───
    world::WorldView view = sim.grid.make_view(
        [](const Cell& c) { return c.type == CellType::Rock || c.type == CellType::Wood; },
        [](const Cell& c) -> uint32_t {
            if (c.type == CellType::Wood) return 1;
            if (c.type == CellType::Rock) return 2;
            return 0;
        },
        [](const Cell& c) -> uint32_t { return c.object_id; }
    );

    rigid::RigidPixelSystem rigidSystem;
    rigid::RigidPixelConfig cfg;
    cfg.linear_damping = 0.1f;   // nearly no air resistance so the block reaches the target
    cfg.gravity_y      = 0.0f;   // horizontal-only test
    rigidSystem.init(GRID_WIDTH, GRID_HEIGHT, cfg);

    // ── Initial object: dagger (object_id = 1) ───────────────────────────
    // Blade (Rock) — tapers from single-pixel tip down to a 9-pixel base
    for (int y = 18; y <= 58; ++y) {
        int half = (y - 18) * 4 / 40;
        for (int x = 100 - half; x <= 100 + half; ++x) {
            sim.placeCell(x, y, CellType::Rock, 1);
            sim.at(x, y).object_id = 1;
        }
    }
    // Crossguard (Wall)
    for (int y = 59; y <= 62; ++y)
        for (int x = 88; x <= 112; ++x) {
            sim.placeCell(x, y, CellType::Wood, 1);
            sim.at(x, y).object_id = 1;
        }
    // Grip (Wall)
    for (int y = 63; y <= 80; ++y)
        for (int x = 97; x <= 103; ++x) {
            sim.placeCell(x, y, CellType::Wood, 1);
            sim.at(x, y).object_id = 1;
        }
    // Pommel (Wall)
    for (int y = 81; y <= 86; ++y)
        for (int x = 94; x <= 106; ++x) {
            sim.placeCell(x, y, CellType::Wood, 1);
            sim.at(x, y).object_id = 1;
        }
    // ── Test: two blocks — static (id=2) on the right, moving (id=3) on the left ─
    // Placed at y=105-120, safely below the dagger (y=18-86)
    // Static block
    for (int y = 105; y <= 120; ++y)
        for (int x = 145; x <= 165; ++x) {
            sim.placeCell(x, y, CellType::Rock, 1);
            sim.at(x, y).object_id = 2;
        }
    // Moving block
    for (int y = 105; y <= 120; ++y)
        for (int x = 30; x <= 50; ++x) {
            sim.placeCell(x, y, CellType::Rock, 1);
            sim.at(x, y).object_id = 3;
        }
    // ── Rotation test block (id=4) — isolated, upper-right area ─────────────
    // Rectangular block so rotation is visually obvious (not square)
    for (int y = 20; y <= 30; ++y)
        for (int x = 155; x <= 185; ++x) {
            sim.placeCell(x, y, CellType::Rock, 1);
            sim.at(x, y).object_id = 4;
        }
    // ── X-blade (id=5) — upper-left, two diagonal arms crossing at center ──
    {
        const int bx = 50, by = 45, arm = 14, half_w = 1;
        for (int dy = -arm; dy <= arm; ++dy) {
            for (int dx = -arm; dx <= arm; ++dx) {
                if (dx*dx + dy*dy > arm*arm) continue;          // clip to circle
                bool on_d1 = std::abs(dx - dy) <= half_w;       // \ arm
                bool on_d2 = std::abs(dx + dy) <= half_w;       // / arm
                if (!on_d1 && !on_d2) continue;
                int x = bx + dx, y = by + dy;
                if (!sim.inBounds(x, y)) continue;
                sim.placeCell(x, y, CellType::Rock, 1);
                sim.at(x, y).object_id = 5;
            }
        }
    }

    // ── Runic classifier ─────────────────────────────────────────────────
    RunicWeights runic_weights;
    bool runic_loaded = runic_load_weights(runic_weights, RUNIC_WEIGHTS_PATH);
    SimpleCanvas<64, 64> runic_canvas;
    int runic_result = -2; // -2 = nothing classified yet
  //-----------------------------------------------------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    bool  velocity_kicked   = false;
    float kick_velocity     = 0.0f;   // exposed in UI
    float spin_velocity     = 10.0f;   // rad/s, exposed in UI
    bool showPhysicsHulls = false;
    bool colorByRegion    = true;
    bool running = true, paused = false, mouseDown = false;
    int  selectedMaterial = 0, brushSize = 3;
    uint32_t nextInstanceId = 2; // 1 is reserved for the authored dagger
    uint32_t currentStrokeId = 0;
    const char* materials[] = { "Sand", "Water", "Wood", "Rock", "Eraser" };

    SDL_Texture* gridTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, GRID_WIDTH, GRID_HEIGHT);
    SDL_SetTextureScaleMode(gridTexture, SDL_SCALEMODE_NEAREST);
    std::vector<uint32_t> pixels(GRID_WIDTH * GRID_HEIGHT);

    Uint64 lastTime = SDL_GetTicks();
    float  accumulator    = 0.0f;
    const float fixedDt   = 1.0f / 60.0f;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) running = false;
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && !io.WantCaptureMouse) {
                mouseDown = true;
                currentStrokeId = nextInstanceId++;
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP)   mouseDown = false;
            if (event.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard) {
                switch (event.key.key) {
                    case SDLK_1: selectedMaterial = 0; break;
                    case SDLK_2: selectedMaterial = 1; break;
                    case SDLK_3: selectedMaterial = 2; break;
                    case SDLK_4: selectedMaterial = 3; break;
                    case SDLK_5: selectedMaterial = 4; break;
                    case SDLK_SPACE: paused = !paused; break;
                    case SDLK_C: sim.clear(); break;
                }
            }
        }
        if (mouseDown && !io.WantCaptureMouse) {
            float mx, my; SDL_GetMouseState(&mx, &my);
            int winW, winH; SDL_GetWindowSize(window, &winW, &winH);
            int gx = int((mx / winW) * GRID_WIDTH);
            int gy = int((my / winH) * GRID_HEIGHT);
            CellType type = selectedMaterial == 0 ? CellType::Sand  :
                            selectedMaterial == 1 ? CellType::Water :
                            selectedMaterial == 2 ? CellType::Wood  :
                            selectedMaterial == 3 ? CellType::Rock  : CellType::Empty;
            bool isSolid = (type == CellType::Rock || type == CellType::Wood);
            uint32_t instanceId = (isSolid && !rigidSystem.extractor.merge_same_type) ? currentStrokeId : 0;
            sim.placeCell(gx, gy, type, brushSize, instanceId);
        }

        Uint64 now = SDL_GetTicks();
        float dt = (now - lastTime) / 1000.0f; lastTime = now;
        if (!paused) {
            accumulator += dt;
            while (accumulator >= fixedDt) {
                rigidSystem.step(fixedDt);
                sim.update();
                accumulator -= fixedDt;
            }
        }

        // ── Rigid system ──────────────────────────────────────────────────
        rigidSystem.update(view);

        // One-shot: kick the moving block (object_id=3) rightward
        if (!velocity_kicked) {
            for (uint32_t i = 0; i < (uint32_t)rigidSystem.body_store.ids.size(); ++i) {
                b2BodyId bid = rigidSystem.body_store.body_ids[i];
                if (!b2Body_IsValid(bid)) continue;
                auto it = rigidSystem.tracker.active_regions.find(rigidSystem.body_store.ids[i]);
                if (it == rigidSystem.tracker.active_regions.end()) continue;
                const auto& b = it->second.bounds;
                if (b.min_x >= 25 && b.max_x <= 60 && b.min_y >= 100 && b.max_y <= 125) {
                    b2Body_SetLinearVelocity(bid, { kick_velocity, 0.0f });
                    velocity_kicked = true;
                    break;
                }
            }
        }
        // Every frame: keep spinning both the test block and the X blade.
        for (uint32_t i = 0; i < (uint32_t)rigidSystem.body_store.ids.size(); ++i) {
            b2BodyId bid = rigidSystem.body_store.body_ids[i];
            if (!b2Body_IsValid(bid)) continue;
            if (b2Body_GetType(bid) != b2_dynamicBody) continue;
            auto it = rigidSystem.tracker.active_regions.find(rigidSystem.body_store.ids[i]);
            if (it == rigidSystem.tracker.active_regions.end()) continue;
            const auto& b = it->second.bounds;
            int cx = (b.min_x + b.max_x) / 2;
            int cy = (b.min_y + b.max_y) / 2;
            bool is_rect_block = (cx > 140 && cx < 200 && cy > 10  && cy < 50);
            bool is_x_blade    = (cx > 25  && cx < 75  && cy > 25  && cy < 65);
            if (is_rect_block || is_x_blade)
                b2Body_SetAngularVelocity(bid, spin_velocity);
        }
        rigidSystem.apply_motion(sim.grid.data(), Cell{ CellType::Empty }, sim.grid.world_revision);

        // ── Render ────────────────────────────────────────────────────────
        const auto& index_to_id  = rigidSystem.tracker.index_to_id;
        const size_t num_mappings = index_to_id.size();
        static std::vector<uint32_t> color_cache;
        if (color_cache.size() < num_mappings) color_cache.resize(num_mappings);
        for (size_t i = 0; i < num_mappings; ++i) {
            SDL_Color c = get_id_color(index_to_id[i]);
            color_cache[i] = (c.r << 24) | (c.g << 16) | (c.b << 8) | 255;
        }

        uint32_t*    __restrict pixel_ptr  = pixels.data();
        const Cell*  __restrict cell_ptr   = sim.grid.data();
        const rigid::RegionIndex* __restrict label_ptr = rigidSystem.extractor.label_grid.data();
        const uint32_t* __restrict cached_colors = color_cache.data();

        for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) {
            const Cell& cell = cell_ptr[i];
            if (cell.type >= CellType::Wood) {
                if (colorByRegion) {
                    rigid::RegionIndex rIdx = label_ptr[i];
                    pixel_ptr[i] = (rIdx < num_mappings) ? cached_colors[rIdx] : 0x646464FF;
                } else {
                    uint8_t v = cell.colorVariation;
                    pixel_ptr[i] = cell.type == CellType::Wood
                        ? ((139-v)<<24)|((90-v)<<16)|(43<<8)|0xFF
                        : ((120-v)<<24)|((120-v)<<16)|((120-v)<<8)|0xFF;
                }
            } else {
                uint8_t v = cell.colorVariation;
                if      (cell.type == CellType::Sand)  pixel_ptr[i] = ((220-v)<<24)|((180-v)<<16)|((80-v)<<8)|0xFF;
                else if (cell.type == CellType::Water) pixel_ptr[i] = ((30+v)<<24)|((100+v)<<16)|((200+v)<<8)|0xC8;
                else                                   pixel_ptr[i] = 0x14141EFA;
            }
        }

        SDL_UpdateTexture(gridTexture, nullptr, pixels.data(), GRID_WIDTH * sizeof(uint32_t));
        SDL_RenderTexture(renderer, gridTexture, nullptr, nullptr);

        if (showPhysicsHulls) {
            int winW, winH; SDL_GetWindowSize(window, &winW, &winH);
            float sx = float(winW)/GRID_WIDTH, sy = float(winH)/GRID_HEIGHT;
            for (auto const& [id, geo] : rigidSystem.geometry_cache) {
                auto it = rigidSystem.tracker.active_regions.find(id);
                if (it == rigidSystem.tracker.active_regions.end()) continue;
                float cx = std::round(it->second.center_f.x); 
        float cy = std::round(it->second.center_f.y);
                SDL_SetRenderDrawColor(renderer, 255, 100, 0, 255);
                for (const auto& piece : geo.convex_pieces) {
                    const auto& pts = piece.points;
                    for (size_t i = 0; i < pts.size(); ++i) {
                        const auto& p1 = pts[i], &p2 = pts[(i+1)%pts.size()];
                        SDL_RenderLine(renderer, (p1.x+cx)*sx, (p1.y+cy)*sy, (p2.x+cx)*sx, (p2.y+cy)*sy);
                    }
                }
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                SDL_RenderPoint(renderer, cx*sx, cy*sy);
            }
        }

        // ── ImGui ─────────────────────────────────────────────────────────
        ImGui_ImplSDLRenderer3_NewFrame(); ImGui_ImplSDL3_NewFrame(); ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        // ── Runic Classifier ─────────────────────────────────────────────
        ImGui::Text("Runic Classifier:");
        if (!runic_loaded) {
            ImGui::TextColored(ImVec4(1.f, 0.3f, 0.3f, 1.f), "runic_weights.bin not found");
        } else {
            constexpr int CW = 64, CH = 64, CPX = 2;
            ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size(CW * CPX, CH * CPX);

            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(canvas_origin,
                ImVec2(canvas_origin.x + canvas_size.x, canvas_origin.y + canvas_size.y),
                IM_COL32(255, 255, 255, 255));
            for (int y = 0; y < CH; ++y) {
                for (int x = 0; x < CW; ++x) {
                    float v = runic_canvas.get(x, y);
                    if (v < 1.f) {
                        uint8_t c = (uint8_t)(v * 255.f);
                        dl->AddRectFilled(
                            ImVec2(canvas_origin.x + x * CPX, canvas_origin.y + y * CPX),
                            ImVec2(canvas_origin.x + (x+1) * CPX, canvas_origin.y + (y+1) * CPX),
                            IM_COL32(c, c, c, 255));
                    }
                }
            }
            dl->AddRect(canvas_origin,
                ImVec2(canvas_origin.x + canvas_size.x, canvas_origin.y + canvas_size.y),
                IM_COL32(160, 160, 160, 255));

            ImGui::InvisibleButton("runic_canvas", canvas_size);
            if (ImGui::IsItemActive() && io.MouseDown[0]) {
                ImVec2 mouse = io.MousePos;
                int mx = (int)((mouse.x - canvas_origin.x) / CPX);
                int my = (int)((mouse.y - canvas_origin.y) / CPX);
                canvas_paint(runic_canvas, mx, my, 2, 0.f);
            }

            if (ImGui::Button("Done")) {
                float img32[1024];
                canvas_to_input(runic_canvas, img32);
                runic_result = runic_classify(runic_weights, img32);
                if (runic_result >= 1 && runic_result <= 6)
                    printf("[Runic] Symbol: %d\n", runic_result);
                else if (runic_result == 0)
                    printf("[Runic] Low confidence\n");
                else
                    printf("[Runic] Nothing drawn\n");
                fflush(stdout);
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear")) {
                runic_canvas.clear();
                runic_result = -2;
            }

            if (runic_result >= 1 && runic_result <= 6)
                ImGui::TextColored(ImVec4(0.2f, 1.f, 0.4f, 1.f), "Symbol: %d", runic_result);
            else if (runic_result == 0)
                ImGui::TextColored(ImVec4(1.f, 0.8f, 0.f, 1.f), "Low confidence");
            else
                ImGui::TextDisabled("Draw a symbol above");
        }
        //till here was ai prediction window and debug
        //
        //
        //

        ImGui::Separator();

        ImGui::Text("Material (1-4):");
        for (int i = 0; i < 5; ++i) {
            if (i > 0) ImGui::SameLine();
            if (ImGui::RadioButton(materials[i], selectedMaterial == i)) selectedMaterial = i;
        }
        ImGui::SliderInt("Brush Size", &brushSize, 1, 10);
        if (ImGui::Button("Select Eraser")) selectedMaterial = 4;
        ImGui::Checkbox("Paused (Space)", &paused);
        ImGui::Checkbox("Color by Region ID", &colorByRegion);
        ImGui::Checkbox("Show Physics Hulls (Debug)", &showPhysicsHulls);
        ImGui::Checkbox("Merge Same Type", &rigidSystem.extractor.merge_same_type);
        if (ImGui::Button("Clear (C)")) sim.clear();
        ImGui::Text("Active Regions: %zu", rigidSystem.tracker.active_regions.size());
        ImGui::Text("Geometry Cache: %zu", rigidSystem.geometry_cache.size());
        ImGui::Separator();
        ImGui::Text("Physics Debug:");
        if (b2World_IsValid(rigidSystem.body_store.world_id)) {
            ImGui::Text("Active Bodies: %zu", rigidSystem.body_store.ids.size());
            ImGui::Text("World ID: %llu", (unsigned long long)rigidSystem.body_store.world_id.index1);
        }
        ImGui::Text("FPS: %.1f", io.Framerate);
        ImGui::Separator();
        ImGui::Text("Physics Test:");
        ImGui::SliderFloat("Kick Speed (m/s)", &kick_velocity, 0.5f, 30.0f);
        ImGui::SliderFloat("Spin Speed (rad/s)", &spin_velocity, 0.1f, 20.0f);
        if (ImGui::Button("Re-spin Block")) {
            for (uint32_t i = 0; i < (uint32_t)rigidSystem.body_store.ids.size(); ++i) {
                b2BodyId bid = rigidSystem.body_store.body_ids[i];
                if (!b2Body_IsValid(bid)) continue;
                auto it = rigidSystem.tracker.active_regions.find(rigidSystem.body_store.ids[i]);
                if (it == rigidSystem.tracker.active_regions.end()) continue;
                const auto& bnd = it->second.bounds;
                int cx = (bnd.min_x + bnd.max_x) / 2;
                int cy = (bnd.min_y + bnd.max_y) / 2;
                if (cx > 140 && cx < 200 && cy > 10 && cy < 50 &&
                    b2Body_GetType(bid) == b2_dynamicBody) {
                    b2Body_SetAngularVelocity(bid, spin_velocity);
                    break;
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Re-launch Block")) {
            for (uint32_t i = 0; i < (uint32_t)rigidSystem.body_store.ids.size(); ++i) {
                b2BodyId bid = rigidSystem.body_store.body_ids[i];
                if (!b2Body_IsValid(bid)) continue;
                auto it = rigidSystem.tracker.active_regions.find(rigidSystem.body_store.ids[i]);
                if (it == rigidSystem.tracker.active_regions.end()) continue;
                const auto& bnd = it->second.bounds;
                int cx = (bnd.min_x + bnd.max_x) / 2;
                int cy = (bnd.min_y + bnd.max_y) / 2;
                // Identify the moving block by its approximate bounding region
                if (cx > 0 && cx < 100 && cy > 100 && cy < 130 &&
                    b2Body_GetType(bid) == b2_dynamicBody) {
                    b2Body_SetLinearVelocity(bid, { kick_velocity, 0.0f });
                    break;
                }
            }
        }
        ImGui::End();

        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    rigidSystem.shutdown();
    SDL_DestroyTexture(gridTexture);
    ImGui_ImplSDLRenderer3_Shutdown(); ImGui_ImplSDL3_Shutdown(); ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer); SDL_DestroyWindow(window); SDL_Quit();
    return 0;
}
