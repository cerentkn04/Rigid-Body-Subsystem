#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <random>
#include <algorithm>
#include <regionScratch.hpp>
#include "RigidPixelSystem.hpp"
constexpr int WINDOW_WIDTH = 800;
constexpr int WINDOW_HEIGHT = 600;
constexpr int CELL_SIZE = 4;
constexpr int GRID_WIDTH = WINDOW_WIDTH / CELL_SIZE;
constexpr int GRID_HEIGHT = WINDOW_HEIGHT / CELL_SIZE;

enum class CellType : uint8_t { Empty = 0, Sand, Water, Wall,Rock };
struct Cell { CellType type = CellType::Empty; bool updated = false; uint8_t colorVariation = 0; };

class SandSimulation {
public:
    std::vector<Cell> grid;
    std::vector<uint64_t> region_revisions; 
    uint64_t world_revision_counter = 1;
    std::mt19937 rng;

    SandSimulation() : grid(GRID_WIDTH * GRID_HEIGHT), 
                       region_revisions(GRID_WIDTH * GRID_HEIGHT, 0),
                       rng(std::random_device{}()) {}

    Cell& at(int x, int y) { return grid[y * GRID_WIDTH + x]; }
    const Cell& at(int x, int y) const { return grid[y * GRID_WIDTH + x]; }
    bool inBounds(int x, int y) const { return x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT; }
    bool isEmpty(int x, int y) const { return inBounds(x, y) && at(x, y).type == CellType::Empty; }

    bool canDisplace(int x, int y, CellType displacing) const {
        if (!inBounds(x, y)) return false;
        CellType target = at(x, y).type;
        if (target == CellType::Empty) return true;
        if (displacing == CellType::Sand && target == CellType::Water) return true;
        return false;
    }

    void swap(int x1, int y1, int x2, int y2) {
        std::swap(at(x1, y1), at(x2, y2));
        at(x1, y1).updated = true;
        at(x2, y2).updated = true;
    }

    void updateSand(int x, int y) {
        if (at(x, y).updated) return;
        if (canDisplace(x, y + 1, CellType::Sand)) { swap(x, y, x, y + 1); return; }
        bool canLeft = canDisplace(x - 1, y + 1, CellType::Sand);
        bool canRight = canDisplace(x + 1, y + 1, CellType::Sand);
        if (canLeft && canRight) swap(x, y, x + ((rng() % 2) ? -1 : 1), y + 1);
        else if (canLeft) swap(x, y, x - 1, y + 1);
        else if (canRight) swap(x, y, x + 1, y + 1);
    }

    void updateWater(int x, int y) {
        if (at(x, y).updated) return;
        if (isEmpty(x, y + 1)) { swap(x, y, x, y + 1); return; }
        bool cDL = isEmpty(x - 1, y + 1), cDR = isEmpty(x + 1, y + 1);
        if (cDL && cDR) { swap(x, y, x + ((rng() % 2) ? -1 : 1), y + 1); return; }
        else if (cDL) { swap(x, y, x - 1, y + 1); return; }
        else if (cDR) { swap(x, y, x + 1, y + 1); return; }
        bool cL = isEmpty(x - 1, y), cR = isEmpty(x + 1, y);
        if (cL && cR) swap(x, y, x + ((rng() % 2) ? -1 : 1), y);
        else if (cL) swap(x, y, x - 1, y);
        else if (cR) swap(x, y, x + 1, y);
    }

    void update() {
        world_revision_counter++;
        for (auto& cell : grid) cell.updated = false;
        for (int y = GRID_HEIGHT - 1; y >= 0; --y) {
            bool leftToRight = (rng() % 2) == 0;
            for (int i = 0; i < GRID_WIDTH; ++i) {
                int x = leftToRight ? i : (GRID_WIDTH - 1 - i);
                switch (at(x, y).type) {
                    case CellType::Sand: updateSand(x, y); break;
                    case CellType::Water: updateWater(x, y); break;
                    case CellType::Wall: break;
                    case CellType::Rock:  break;    
                    case CellType::Empty: break;

                }
            }
        }
    }

    void placeCell(int x, int y, CellType type, int brushSize = 1) {
        std::uniform_int_distribution<uint8_t> dist(0, 30);
        bool topology_changed = false;
        for (int dy = -brushSize + 1; dy < brushSize; ++dy) {
            for (int dx = -brushSize + 1; dx < brushSize; ++dx) {
                int nx = x + dx, ny = y + dy;
                if (!inBounds(nx, ny)) continue;
                CellType old = at(nx, ny).type;
                if (old == type) continue;
                at(nx, ny).type = type;
                at(nx, ny).colorVariation = dist(rng);
                if (old == CellType::Rock ||old == CellType::Wall || type == CellType::Rock|| type == CellType::Wall) {
                    topology_changed = true;
                    region_revisions[ny * GRID_WIDTH + nx] = world_revision_counter;
                }
            }
        }
        if (topology_changed) world_revision_counter++;
    }

    void clear() { for (auto& cell : grid) { cell.type = CellType::Empty; cell.colorVariation = 0; } }
};

static SandSimulation* g_pixel_sim_ptr = nullptr;
world::CellSolidity solidity_callback(int x, int y) {
    return (g_pixel_sim_ptr && g_pixel_sim_ptr->inBounds(x, y) && g_pixel_sim_ptr->at(x, y).type == CellType::Wall  ) ? world::CellSolidity::Solid : g_pixel_sim_ptr && g_pixel_sim_ptr->inBounds(x, y) && g_pixel_sim_ptr->at(x, y).type == CellType::Rock ? world::CellSolidity::Solid :  world::CellSolidity::Empty;
}
world::WorldRevision revision_callback() { return g_pixel_sim_ptr ? g_pixel_sim_ptr->world_revision_counter : 0; }
world::WorldRevision region_rev_callback(int x, int y) {
    return (g_pixel_sim_ptr && g_pixel_sim_ptr->inBounds(x, y)) ? g_pixel_sim_ptr->region_revisions[y * GRID_WIDTH + x] : 0;
}
world::GroupID group_id_callback(int x, int y) {
    if (!g_pixel_sim_ptr || !g_pixel_sim_ptr->inBounds(x, y)) return world::InvalidGroupID;
    
    CellType t = g_pixel_sim_ptr->at(x, y).type;
    if (t == CellType::Wall) return 1; // Group 1 for Walls
    if (t == CellType::Rock) return 2; // Group 2 for Rocks
    
    return world::InvalidGroupID;
}

SDL_Color get_id_color(rigid::RegionID id) {
    if (id == rigid::InvalidRegionID) return { 50, 50, 50, 255 };
    uint32_t h = id;
    h = ((h >> 16) ^ h) * 0x45d9f3b; h = ((h >> 16) ^ h) * 0x45d9f3b; h = (h >> 16) ^ h;
    return { static_cast<uint8_t>(h & 0xFF), static_cast<uint8_t>((h >> 8) & 0xFF), static_cast<uint8_t>((h >> 16) & 0xFF), 255 };
}

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO)) return 1;
    SDL_Window* window = SDL_CreateWindow("Sand Falling Simulation", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    
    SandSimulation sim;
    g_pixel_sim_ptr = &sim;

    rigid::RigidPixelSystem rigidSystem;

    world::WorldView view;
    view.width = GRID_WIDTH; view.height = GRID_HEIGHT;
    view.solidity_at = solidity_callback;
    view.world_revision = revision_callback;
    view.region_revision = region_rev_callback;
    view.group_id_at = group_id_callback;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);
    bool showPhysicsHulls = false;
    bool running = true, paused = false, mouseDown = false;
    int selectedMaterial = 0, brushSize = 3;
    const char* materials[] = {"Sand", "Water", "Wall", "Rock","Eraser"};

    SDL_Texture* gridTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, GRID_WIDTH, GRID_HEIGHT);
    SDL_SetTextureScaleMode(gridTexture, SDL_SCALEMODE_NEAREST);
    std::vector<uint32_t> pixels(GRID_WIDTH * GRID_HEIGHT);

    Uint64 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;
    const float fixedDeltaTime = 1.0f / 60.0f;


    b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.gravity = { 0.0f, 9.8f }; // Standard gravity
b2WorldId worldId = b2CreateWorld(&worldDef);

// Link it to your system
rigidSystem.init_physics(worldId);
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) running = false;
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && !io.WantCaptureMouse) mouseDown = true;
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) mouseDown = false;
            if (event.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard) {
                switch (event.key.key) {
                    case SDLK_1: selectedMaterial = 0; break;
                    case SDLK_2: selectedMaterial = 1; break;
                    case SDLK_3: selectedMaterial = 2; break;
                    case SDLK_4: selectedMaterial = 3; break;
                    case SDLK_5: selectedMaterial = 4; break;
                    case SDLK_SPACE: paused = !paused; break;
                    case SDLK_C: sim.clear();break;
                }
            }
        }

        if (mouseDown && !io.WantCaptureMouse) {
            float mx, my; SDL_GetMouseState(&mx, &my);
            int winW, winH; SDL_GetWindowSize(window, &winW, &winH);
            int gx = static_cast<int>((mx / winW) * GRID_WIDTH);
            int gy = static_cast<int>((my / winH) * GRID_HEIGHT);
            CellType type = (selectedMaterial == 0) ? CellType::Sand : (selectedMaterial == 1) ? CellType::Water : (selectedMaterial == 2) ? CellType::Wall : (selectedMaterial == 3) ? CellType::Rock :CellType::Empty;
            sim.placeCell(gx, gy, type, brushSize);
        }

        Uint64 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - lastTime) / 1000.0f; lastTime = currentTime;
        if (!paused) {
            accumulator += deltaTime;
            while (accumulator >= fixedDeltaTime) { 
                sim.update(); 
                b2World_Step(worldId, fixedDeltaTime, 4); 
                
                accumulator -= fixedDeltaTime; 
            }
        }

        // --- RIGID SYSTEM ---
        rigidSystem.update(view);

        // --- RENDER (Restored Tight Loop) ---
        const auto& index_to_id = rigidSystem.tracker.get_index_mapping();
        const size_t num_mappings = index_to_id.size();
        static std::vector<uint32_t> color_cache;
        if (color_cache.size() < num_mappings) color_cache.resize(num_mappings);
        for (size_t i = 0; i < num_mappings; ++i) {
            SDL_Color c = get_id_color(index_to_id[i]);
            color_cache[i] = (c.r << 24) | (c.g << 16) | (c.b << 8) | 255;
        }

        uint32_t* __restrict pixel_ptr = pixels.data();
        const Cell* __restrict cell_ptr = sim.grid.data();
        const rigid::RegionIndex* __restrict label_ptr = rigidSystem.extractor.label_grid().data();
        const uint32_t* __restrict cached_colors = color_cache.data();

        for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) {
            const Cell& cell = cell_ptr[i];
            if (cell.type == CellType::Wall) {
                const rigid::RegionIndex rIdx = label_ptr[i];
                pixel_ptr[i] = (rIdx < num_mappings) ? cached_colors[rIdx] : 0x646464FF;
            } else if(cell.type == CellType::Rock){
                const rigid::RegionIndex rIdx = label_ptr[i];
                pixel_ptr[i] = (rIdx < num_mappings) ? cached_colors[rIdx] : 0x646450FF;

            }
            else {
                const uint8_t v = cell.colorVariation;
                if (cell.type == CellType::Sand) pixel_ptr[i] = ((220-v) << 24) | ((180-v) << 16) | ((80-v) << 8) | 0xFF;
                else if (cell.type == CellType::Water) pixel_ptr[i] = ((30+v) << 24) | ((100+v) << 16) | ((200+v) << 8) | 0xC8;
                else pixel_ptr[i] = 0x14141EFA; 
            }
        }


// --- RENDER (Pixel Texture) ---
        SDL_UpdateTexture(gridTexture, nullptr, pixels.data(), GRID_WIDTH * sizeof(uint32_t));
        SDL_RenderTexture(renderer, gridTexture, nullptr, nullptr);

        // --- RENDER STEP: GEOMETRY (Optimized Debug View) ---
        // ONLY this block should exist for contours
        if (showPhysicsHulls) {
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
            
            int winW, winH;
            SDL_GetWindowSize(window, &winW, &winH);
            float scaleX = (float)winW / GRID_WIDTH;
            float scaleY = (float)winH / GRID_HEIGHT;
            float visualOffsetX = 0.5f * scaleX; 
            float visualOffsetY = 0.5f * scaleY;

            for (auto const& [id, geo] : rigidSystem.geometry_cache) {
                for (const auto& contour : geo.contours) {
                    const auto& points = contour.points;
                    if (points.size() < 2) continue;

                    for (size_t i = 0; i < points.size(); ++i) {
                        const auto& p1 = points[i];
                        const auto& p2 = points[(i + 1) % points.size()];

                        float x1 = (p1.x * scaleX) + visualOffsetX;
                        float y1 = (p1.y * scaleY) + visualOffsetY;
                        float x2 = (p2.x * scaleX) + visualOffsetX; // Fixed typo here (was scaleY)
                        float y2 = (p2.y * scaleY) + visualOffsetY;

                        SDL_RenderLine(renderer, x1, y1, x2, y2);
                    }
                }
            }
        }
        // DELETE the naked loop that was here!


 

            // --- IMGUI (Original Controls) ---
        ImGui_ImplSDLRenderer3_NewFrame(); ImGui_ImplSDL3_NewFrame(); ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Material (1-4):");
     
        for (int i = 0; i < 5; ++i) {
            if (i > 0) ImGui::SameLine();
            if (ImGui::RadioButton(materials[i], selectedMaterial == i)) selectedMaterial = i;
        }
        ImGui::SliderInt("Brush Size", &brushSize, 1, 10);
        if (ImGui::Button("Select Eraser")) selectedMaterial = 4;
        ImGui::Checkbox("Paused (Space)", &paused);
        ImGui::Checkbox("Show Physics Hulls (Debug)", &showPhysicsHulls);
        if (ImGui::Button("Clear (C)")) sim.clear();
        ImGui::Text("Total Active Regions: %zu", rigidSystem.tracker.get_active_regions().size());
        ImGui::Text("Geometry Cache: %zu", rigidSystem.geometry_cache.size());
        ImGui::Separator();
        ImGui::Text("FPS: %.1f", io.Framerate);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(gridTexture);
    ImGui_ImplSDLRenderer3_Shutdown(); ImGui_ImplSDL3_Shutdown(); ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer); SDL_DestroyWindow(window); SDL_Quit();
    return 0;
}
