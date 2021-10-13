#define OLC_PGE_APPLICATION

#include "olcPixelGameEngine.h"
#include <memory>


class MainSimulation: public olc::PixelGameEngine {
    std::unique_ptr<olc::Sprite> carSprite;

    public:
        MainSimulation() {
            sAppName = "Local path planning";
        }

        bool OnUserCreate() override {
            carSprite = std::make_unique<olc::Sprite>("./resources/car.png");
            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {
            Clear(olc::DARK_BLUE);
            DrawSprite(olc::vi2d(20,20), carSprite.get());

            return true;
        }
};

int main() {
    MainSimulation main;
    if (main.Construct(256, 256, 1, 1)) {
        main.Start();
    }
    return 0;
}
