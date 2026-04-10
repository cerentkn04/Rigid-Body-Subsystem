#include <iostream>
#include <string>
#include <ctime>
#include <cstdlib>

// Single-header PNG loader — grab stb_image.h from github.com/nothings/stb
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "runic_classifier.h"

// Loads a PNG as a flat 32x32 normalized float array [0,1].
// out_pixels must be at least 1024 floats.
// Returns true on success.
bool load_image(const char* path, float out_pixels[1024]) {
    int w, h, channels;
    unsigned char* data = stbi_load(path, &w, &h, &channels, 1); // force grayscale
    if (!data) return false;

    // Resize to 32x32 by nearest-neighbor if needed
    // (dataset images are already 32x32, but guard anyway)
    if (w == 32 && h == 32) {
        for (int i = 0; i < 1024; i++)
            out_pixels[i] = data[i] / 255.f;
    } else {
        for (int py = 0; py < 32; py++)
            for (int px = 0; px < 32; px++) {
                int sy = (py * h) / 32;
                int sx = (px * w) / 32;
                out_pixels[py*32+px] = data[sy*w+sx] / 255.f;
            }
    }

    stbi_image_free(data);
    return true;
}

int main() {
    RunicWeights weights;
    if (!runic_load_weights(weights, "runic_weights.bin")) {
        std::cerr << "Failed to load runic_weights.bin\n"
                  << "Run export_weights.py first.\n";
        return 1;
    }

    float pixels[1024];
    int success_count = 0;
    srand(static_cast<unsigned int>(time(0)));

    std::cout << "--- Runic Sembol Tanima Testi Basliyor ---\n\n";

    for (int i = 0; i < 15000; i++) {
        int random_symbol  = (rand() % 6) + 1;
        int random_img_idx = rand() % 180;

        std::string path = "dataset/" + std::to_string(random_symbol)
                         + "/" + std::to_string(random_img_idx) + ".png";

        std::cout << "Deneme " << i + 1 << ":\n";
        std::cout << "Secilen Dosya: " << path << "\n";

        if (!load_image(path.c_str(), pixels)) {
            std::cout << "Hata: Dosya okunamadi!\n";
            std::cout << "------------------------------------------\n";
            continue;
        }

        int result = runic_classify(weights, pixels);

        std::cout << "Gercek Sembol   : " << random_symbol << "\n";
        std::cout << "Modelin Tahmini : " << result << "\n";

        if (result == random_symbol) {
            success_count++;
            std::cout << "DURUM: BASARILI [V]\n";
        } else if (result == 0) {
            std::cout << "DURUM: DUSUK GUVEN\n";
        } else {
            std::cout << "DURUM: HATALI [X]\n";
        }
        std::cout << "------------------------------------------\n";
    }

    std::cout << "\nToplam Basari Orani: "
              << (success_count / 15000.0) * 100 << "%\n";
    return 0;
}
