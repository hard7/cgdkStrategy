#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <map>
#include <stack>
#include <stdexcept>
#include <cassert>


struct TileNode {
    int x, y;
    std::vector<unsigned> nb;
    TileNode(int x_, int y_) : x(x_), y(y_) {}
    TileNode(std::pair<int, int> const& p) : x(p.first), y(p.second) {}
    void appendLink(int i) { nb.push_back(static_cast<unsigned>(i)); }
};

struct MyStrategy::Impl {
    model::World const* world;
    std::vector<TileNode> tileGraph;

    Impl() : world(nullptr) { }
    void updateWorld(model::World const* world_) {
        if(world == nullptr) tileGraph = makeTileGraph(world_->getTilesXY());
        if(world != world_) world = world_;

    }

    void showTilesXY() const {
        using namespace std;
        cout << "TilesXY" << endl;
        auto const& tilesXY = world->getTilesXY();
        for(int j=0; j<world->getHeight(); j++) {
            for(int i=0; i<world->getWidth(); i++) {
                cout << tilesXY[i][j];
            }
            cout << endl;
        }
        cout << endl;
    }

    void showWaypoints() const {
        using namespace std;
        cout << "Waypoints" << endl;
        for(auto const& tile : world->getWaypoints()) {
            cout << "[" << tile[0] << ", " << tile[1] << "] ";
        }
        cout << endl << endl;
    }

private:
    static std::vector<TileNode> makeTileGraph(std::vector<std::vector<model::TileType> > const& tiles) {
        std::vector<TileNode> tileGraph;
        int dimY = static_cast<int>(tiles.size());
        int dimX = static_cast<int>(tiles.at(0).size());

        typedef std::pair<int, int> Point;
        std::map<Point, int> visited;
        for(int j=0; j<dimY; j++) {
            for(int i=0; i<dimX; i++) {
                visited[{i, j}] = -1;
            }
        }

        auto getNeighbors = [](Point const& p, model::TileType t) -> std::vector<Point>  {
            using namespace model;
            int x = p.first, y = p.second;
            switch(t) {
                case EMPTY: return {};
                case VERTICAL:   return {{x, y-1}, {x, y+1}};
                case HORIZONTAL: return {{x-1, y}, {x+1, y}};
                case LEFT_TOP_CORNER:     return {{x, y+1}, {x+1, y}};
                case RIGHT_TOP_CORNER:    return {{x, y+1}, {x-1, y}};
                case LEFT_BOTTOM_CORNER:  return {{x, y-1}, {x+1, y}};
                case RIGHT_BOTTOM_CORNER: return {{x, y-1}, {x-1, y}};
                case LEFT_HEADED_T:   return {{x, y-1}, {x, y+1}, {x-1, y}};
                case RIGHT_HEADED_T:  return {{x, y-1}, {x, y+1}, {x+1, y}};
                case TOP_HEADED_T:    return {{x-1, y}, {x+1, y}, {x, y-1}};
                case BOTTOM_HEADED_T: return {{x-1, y}, {x+1, y}, {x, y+1}};
                case CROSSROADS: return {{x, y-1}, {x, y+1}, {x-1, y}, {x+1, y}};
                case UNKNOWN: return {}; //WARNING
                default: throw std::logic_error("Error in MyStrategy::Impl::getNeighbors: _UNKNOWN_TILE_TYPE_");
            }
        };

        int x, y;
        int ccnum = 0;
        int p_idx, n_idx;
        std::stack<Point> stack_;
        for(int j=0; j<dimY; j++) {
            for(int i=0; i<dimX; i++) {
                const Point BASE(i, j);

                if(visited[BASE] == -1) {
                    stack_.push(BASE);
                    ccnum++;
                }
                while(not stack_.empty()) {
                    Point p = stack_.top(); stack_.pop();
                    tileGraph.push_back(TileNode(p));
                    visited[p] = p_idx = static_cast<int>(tileGraph.size() - 1);
//                    TileNode& parent = tileGraph.back();
                    const std::vector<Point> neighbors
                            = getNeighbors(p, tiles[p.first][p.second]);
                    for(Point const& n : neighbors) {
                        x = n.first; y = n.second;
                        assert(x >= 0 and x < dimX);
                        assert(y >= 0 and y < dimY);
                        n_idx = visited[n];
                        if(n_idx < 0) stack_.push(n);
                        else {
                            tileGraph[p_idx].appendLink(n_idx);
                            tileGraph[n_idx].appendLink(p_idx);
                        }
                    }
                }
            }
        }
        return std::move(tileGraph);
    }

};

using namespace model;
using namespace std;

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    impl->updateWorld(&world);
    impl->showTilesXY();
    impl->showWaypoints();

    Car const& car = self;
    int px = static_cast<int>(car.getX() / 800);
    int py = static_cast<int>(car.getY() / 800);
    cout << car.getAngle() / (2 * M_PI) * 360 << endl;

    cout << impl->tileGraph[0].x << endl;
    cout << impl->tileGraph[0].y << endl;
    cout << impl->tileGraph[0].nb[0] << endl;
    cout << impl->tileGraph[0].nb[1] << endl;

    exit(1);
}

MyStrategy::MyStrategy() : impl(new Impl) { }