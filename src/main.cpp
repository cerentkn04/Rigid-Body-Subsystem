
#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <vector>
#include <random>
#include <algorithm>
#include <RigidPixelWorldView.hpp>
#include <RigidPixelSystem.hpp>
#include <RigidPixelTypes.hpp>
#include <regionScratch.hpp>
#include <RegionTracker.hpp>
#include <RegionType.hpp>
#include <RegionExtractor.hpp>
constexpr int WINDOW_WIDTH = 800;
constexpr int WINDOW_HEIGHT = 600;
constexpr int CELL_SIZE = 4;
constexpr int GRID_WIDTH = WINDOW_WIDTH / CELL_SIZE;
constexpr int GRID_HEIGHT = WINDOW_HEIGHT / CELL_SIZE;

enum class CellType : uint8_t {
    Empty = 0,
    Sand,
    Water,
    Wall
};

struct Cell {
    CellType type = CellType::Empty;
    bool updated = false;
    uint8_t colorVariation = 0;
};

class SandSimulation {
public:
    std::vector<Cell> grid;
    std::mt19937 rng;

    SandSimulation() : grid(GRID_WIDTH * GRID_HEIGHT), rng(std::random_device{}()) {}

    Cell& at(int x, int y) { return grid[y * GRID_WIDTH + x]; }
    const Cell& at(int x, int y) const { return grid[y * GRID_WIDTH + x]; }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT;
    }

    bool isEmpty(int x, int y) const {
        return inBounds(x, y) && at(x, y).type == CellType::Empty;
    }

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

        // Try to fall straight down
        if (canDisplace(x, y + 1, CellType::Sand)) {
            swap(x, y, x, y + 1);
            return;
        }

        // Try to fall diagonally
        bool canLeft = canDisplace(x - 1, y + 1, CellType::Sand);
        bool canRight = canDisplace(x + 1, y + 1, CellType::Sand);

        if (canLeft && canRight) {
            int dir = (rng() % 2) ? -1 : 1;
            swap(x, y, x + dir, y + 1);
        } else if (canLeft) {
            swap(x, y, x - 1, y + 1);
        } else if (canRight) {
            swap(x, y, x + 1, y + 1);
        }
    }

    void updateWater(int x, int y) {
        if (at(x, y).updated) return;

        // Try to fall straight down
        if (isEmpty(x, y + 1)) {
            swap(x, y, x, y + 1);
            return;
        }

        // Try to fall diagonally
        bool canDownLeft = isEmpty(x - 1, y + 1);
        bool canDownRight = isEmpty(x + 1, y + 1);

        if (canDownLeft && canDownRight) {
            int dir = (rng() % 2) ? -1 : 1;
            swap(x, y, x + dir, y + 1);
            return;
        } else if (canDownLeft) {
            swap(x, y, x - 1, y + 1);
            return;
        } else if (canDownRight) {
            swap(x, y, x + 1, y + 1);
            return;
        }

        // Try to spread horizontally
        bool canLeft = isEmpty(x - 1, y);
        bool canRight = isEmpty(x + 1, y);

        if (canLeft && canRight) {
            int dir = (rng() % 2) ? -1 : 1;
            swap(x, y, x + dir, y);
        } else if (canLeft) {
            swap(x, y, x - 1, y);
        } else if (canRight) {
            swap(x, y, x + 1, y);
        }
    }

    void update() {
        // Reset update flags
        for (auto& cell : grid) {
            cell.updated = false;
        }

        // Update from bottom to top, alternating left-right direction
        for (int y = GRID_HEIGHT - 1; y >= 0; --y) {
            bool leftToRight = (rng() % 2) == 0;

            for (int i = 0; i < GRID_WIDTH; ++i) {
                int x = leftToRight ? i : (GRID_WIDTH - 1 - i);

                switch (at(x, y).type) {
                    case CellType::Sand:
                        updateSand(x, y);
                        break;
                    case CellType::Water:
                        updateWater(x, y);
                        break;
                    default:
                        break;
                }
            }
        }
    }

    void placeCell(int x, int y, CellType type, int brushSize = 1) {
        std::uniform_int_distribution<uint8_t> dist(0, 30);

        for (int dy = -brushSize + 1; dy < brushSize; ++dy) {
            for (int dx = -brushSize + 1; dx < brushSize; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (inBounds(nx, ny)) {
                    if (type == CellType::Empty || at(nx, ny).type == CellType::Empty) {
                        at(nx, ny).type = type;
                        at(nx, ny).colorVariation = dist(rng);
                    }
                }
            }
        }
    }

    void clear() {
        for (auto& cell : grid) {
            cell.type = CellType::Empty;
            cell.colorVariation = 0;
        }
    }
};

SDL_Color getCellColor(const Cell& cell) {
    int v = cell.colorVariation;
    switch (cell.type) {
        case CellType::Sand:
            return {static_cast<uint8_t>(220 - v), static_cast<uint8_t>(180 - v), static_cast<uint8_t>(80 - v), 255};
        case CellType::Water:
            return {static_cast<uint8_t>(30 + v), static_cast<uint8_t>(100 + v), static_cast<uint8_t>(200 + v), 200};
        case CellType::Wall:
            return {static_cast<uint8_t>(100 - v), static_cast<uint8_t>(100 - v), static_cast<uint8_t>(100 - v), 255};
        default:
            return {20, 20, 30, 255};
    }
}

static SandSimulation* g_pixel_sim_ptr = nullptr;

world::CellSolidity solidity_callback(int x, int y) {
    if (g_pixel_sim_ptr && g_pixel_sim_ptr->inBounds(x, y)) {
        // Only "Wall" counts as Solid for our Rigid Body system
        return g_pixel_sim_ptr->at(x, y).type == CellType::Wall ? 
               world::CellSolidity::Solid : world::CellSolidity::Empty;
    }
    return world::CellSolidity::Empty;
}
world::WorldRevision revision_callback() { return 0; }
world::WorldRevision region_rev_callback(int x, int y) { return 0; }


int main(int argc, char* argv[]) {  if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Sand Falling Simulation",
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_RESIZABLE
    );

    if (!window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SandSimulation sim;
    g_pixel_sim_ptr = &sim;

rigid::RegionExtractor extractor;
rigid::RegionTracker tracker;
std::vector<rigid::RegionBuildRecord> build_records;

world::WorldView view;
view.width = GRID_WIDTH;
view.height = GRID_HEIGHT;
view.solidity_at = solidity_callback;
view.world_revision = revision_callback;
view.region_revision = region_rev_callback;

// For debug visualization: a unique color for each persistent RegionID
std::unordered_map<rigid::RegionID, SDL_Color> region_colors;
auto get_id_color = [&](rigid::RegionID id) -> SDL_Color {
    if (id == rigid::InvalidRegionID) return { 50, 50, 50, 255 };

    // Simple deterministic hash to get unique colors per ID
    uint32_t h = id;
    h = ((h >> 16) ^ h) * 0x45d9f3b;
    h = ((h >> 16) ^ h) * 0x45d9f3b;
    h = (h >> 16) ^ h;

    return {
        static_cast<uint8_t>((h >> 0) & 0xFF),
        static_cast<uint8_t>((h >> 8) & 0xFF),
        static_cast<uint8_t>((h >> 16) & 0xFF),
        255
    };
};

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);


    bool running = true;
    bool paused = false;
    int selectedMaterial = 0;
    const char* materials[] = {"Sand", "Water", "Wall", "Eraser"};
    int brushSize = 3;
    bool mouseDown = false;

    SDL_Texture* gridTexture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGBA8888,
        SDL_TEXTUREACCESS_STREAMING,
        GRID_WIDTH, GRID_HEIGHT
    );

    // Set nearest-neighbor scaling for crisp pixels
    SDL_SetTextureScaleMode(gridTexture, SDL_SCALEMODE_NEAREST);

    std::vector<uint32_t> pixels(GRID_WIDTH * GRID_HEIGHT);

    Uint64 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;
    const float fixedDeltaTime = 1.0f / 60.0f;

    while (running) {

SDL_Event event;
        // 1. Process Input Events
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
                    case SDLK_SPACE: paused = !paused; break;
                    case SDLK_C: sim.clear(); break;
                }
            }
        }

        // 2. Handle Mouse Drawing (MUTATE WORLD)
        if (mouseDown && !io.WantCaptureMouse) {
            float mx, my;
            SDL_GetMouseState(&mx, &my);
            int winW, winH;
            SDL_GetWindowSize(window, &winW, &winH);
            int gx = static_cast<int>((mx / winW) * GRID_WIDTH);
            int gy = static_cast<int>((my / winH) * GRID_HEIGHT);

            CellType type = (selectedMaterial == 0) ? CellType::Sand : 
                            (selectedMaterial == 1) ? CellType::Water : 
                            (selectedMaterial == 2) ? CellType::Wall : CellType::Empty;
            sim.placeCell(gx, gy, type, brushSize);
        }

        // 3. Update Simulation (MUTATE WORLD)
        Uint64 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;
        if (!paused) {
            accumulator += deltaTime;
            while (accumulator >= fixedDeltaTime) {
                sim.update();
                accumulator -= fixedDeltaTime;
            }
        }

        // 4. TOPOLOGY PHASE (READ WORLD)
        // Move extraction here so it sees the results of Sim and Mouse placement
        extractor.extract(view, build_records);
        tracker.process_frame(extractor.label_grid(), build_records, GRID_WIDTH, GRID_HEIGHT);
        
        const auto& label_grid = extractor.label_grid();

        // 5. RENDER PHASE (CONVERT TO PIXELS)
        // const auto& label_grid = extractor.label_grid();
const auto& index_to_id = tracker.get_index_mapping();
        size_t num_regions = index_to_id.size();
        
        // Pre-calculate colors for the few regions we have
        std::vector<uint32_t> color_cache(num_regions);
        for (size_t i = 0; i < num_regions; ++i) {
            SDL_Color c = get_id_color(index_to_id[i]);
            color_cache[i] = (c.r << 24) | (c.g << 16) | (c.b << 8) | c.a;
        }

        uint32_t* pixel_ptr = pixels.data();
        const Cell* cell_ptr = sim.grid.data();
        const rigid::RegionIndex* label_ptr = label_grid.data();
        int total_cells = GRID_WIDTH * GRID_HEIGHT;

        for (int i = 0; i < total_cells; ++i) {
            if (cell_ptr[i].type == CellType::Wall) {
                rigid::RegionIndex rIdx = label_ptr[i];
                if (rIdx < num_regions) {
                    pixel_ptr[i] = color_cache[rIdx];
                } else {
                    pixel_ptr[i] = 0x646464FF; // Default Gray
                }
            } else {
                // Inline the getCellColor logic to avoid function call overhead
                const Cell& cell = cell_ptr[i];
                int v = cell.colorVariation;
                if (cell.type == CellType::Sand) 
                    pixel_ptr[i] = ((220-v) << 24) | ((180-v) << 16) | ((80-v) << 8) | 0xFF;
                else if (cell.type == CellType::Water)
                    pixel_ptr[i] = ((30+v) << 24) | ((100+v) << 16) | ((200+v) << 8) | 0xC8;
                else 
                    pixel_ptr[i] = 0x14141EFA; // Empty/Background
            }
        }

/*for (int y = 0; y < GRID_HEIGHT; ++y) {
  
    for (int x = 0; x < GRID_WIDTH; ++x) {
        int pixelIdx = y * GRID_WIDTH + x;
        const auto& cell = sim.at(x, y);
        
        // Default color for empty space/sand/water
        SDL_Color finalColor = getCellColor(cell);

        if (cell.type == CellType::Wall) {
            rigid::RegionIndex rIdx = label_grid[pixelIdx];
            rigid::RegionID pId = rigid::InvalidRegionID;

            // Method A: Check the fast mapping vector
            if (rIdx != rigid::InvalidRegionIndex && rIdx < index_to_id.size()) {
                pId = index_to_id[rIdx];
            }

            // Method B: Emergency Fallback (If Mapping is empty/broken)
            // If the fast map failed but we have an index, find who owns it
            if (pId == rigid::InvalidRegionID && rIdx != rigid::InvalidRegionIndex) {
                // We know this pixel belongs to a region, let's find which one
                // based on the ID sequence (rIdx + 1 is a common sequence start)
                pId = rIdx + 1; 
            }

            finalColor = get_id_color(pId);
        }

        pixels[pixelIdx] = (finalColor.r << 24) | (finalColor.g << 16) | (finalColor.b << 8) | finalColor.a;
    }
} */
               // 6. DRAW TO SCREEN
        SDL_UpdateTexture(gridTexture, nullptr, pixels.data(), GRID_WIDTH * sizeof(uint32_t));
        SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255);
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, gridTexture, nullptr, nullptr);
               // Start ImGui frame
        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // Control panel
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("Material (1-4):");
        for (int i = 0; i < 4; ++i) {
            if (i > 0) ImGui::SameLine();
            if (ImGui::RadioButton(materials[i], selectedMaterial == i)) {
                selectedMaterial = i;
            }
        }

        ImGui::SliderInt("Brush Size", &brushSize, 1, 10);

        ImGui::Checkbox("Paused (Space)", &paused);

        if (ImGui::Button("Clear (C)")) {
            sim.clear();
        }
        const auto& active = tracker.get_active_regions();
ImGui::Text("Total Active Regions: %zu", active.size());


        ImGui::Separator();
        ImGui::Text("FPS: %.1f", io.Framerate);

        ImGui::End();


 



        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

        SDL_RenderPresent(renderer);
    }


    // Cleanup
    SDL_DestroyTexture(gridTexture);
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

