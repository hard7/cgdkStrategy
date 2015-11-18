#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace model;
using namespace std;

struct MyStrategy::Impl {
    World const* world;
    Impl() {}
    void updateWorld(World const* world_) { if(world != world_) world = world_; }

    void showTilesXY() const {
        cout << "TilesXY" << endl;
        auto const& tilesXY = world->getTilesXY();
        for(int j=0; j<world->getHeight(); j++) {
            for(int i=0; i<world->getWidth(); i++) {
                cout << tilesXY[i][j];
            }
            cout << endl;
        }
        cout << endl;

//        for(auto const& tile : world->getWaypoints())
    }
};


void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    impl->updateWorld(&world);
    impl->showTilesXY();

    Car const& car = self;
    int px = static_cast<int>(car.getX() / 800);
    int py = static_cast<int>(car.getY() / 800);
    cout << px << " " << py << endl;
    cout << world.getTilesXY()[px][py+1] << endl;

    exit(1);
//    move.setSpeedUp(-1.0);
//    move.setTurn(PI);
//    move.setAction(STRIKE);
}

MyStrategy::MyStrategy() : impl(new Impl) { }