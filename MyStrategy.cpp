#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <stack>
#include <queue>
#include <stdexcept>
#include <cassert>
#include <tuple>


struct TileNode {
    typedef std::pair<int, int> Point;
    int x, y;
    std::vector<unsigned> nb_RM;
    std::vector<TileNode*> neighbors;
    TileNode() : x(-1), y(-1) {}
    TileNode(int x_, int y_) : x(x_), y(y_) {}
    TileNode(std::pair<int, int> const& p) : x(p.first), y(p.second) {}
    void appendLink(int i) { nb_RM.push_back(static_cast<unsigned>(i)); }
    void appendLink(TileNode* node) { neighbors.push_back((node)); }
    operator Point() const { return std::make_pair(x, y); };
};

std::ostream& operator<<(std::ostream& os, TileNode const& node) {
    os << "( [" << node.x << ", " << node.y << "] -> ";
    for(TileNode* nearest : node.neighbors) {
        os <<" [" << nearest->x << ", " << nearest->y << "]";
    }
    os << " )";
    return os;
}

struct MyStrategy::Impl {
    typedef std::pair<int, int> Point;

    model::World const* world;
    std::map<Point, TileNode> tileGraph;

    Impl() : world(nullptr) { }
    void updateWorld(model::World const* world_) {
        if(world == nullptr) tileGraph = makeTileGraph(world_->getTilesXY());
        if(world != world_) world = world_;
    }

    std::vector<Point> shortestPath(Point const& from, Point const& to) const {
        std::vector<Point> path;
        std::set<Point> visited;
        auto fromIt = tileGraph.find(from);
        auto toIt = tileGraph.find(to);
        assert(fromIt != tileGraph.end() and toIt != tileGraph.end());
        if(fromIt == toIt) return std::move(path);
        TileNode const* target = &fromIt->second;
        std::queue<TileNode const*> queue_;
        queue_.push(&fromIt->second);
        bool inserted;
        while(not queue_.empty()) {
            TileNode const* current = queue_.front(); queue_.pop();
            std::tie(std::ignore, inserted) = visited.insert(*current);
            if(inserted) {
                for(TileNode const* nearest : current->neighbors) {
                    queue_.push(nearest);
                }
            }
        }
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
    static std::map<Point, TileNode> makeTileGraph(std::vector<std::vector<model::TileType> > const& tiles) {
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
        using namespace std;
        std::map<Point, TileNode> tileGraph;
        int dimY = static_cast<int>(tiles.size());
        int dimX = static_cast<int>(tiles.at(0).size());

        std::map<Point, int> visited;
        for(int j=0; j<dimY; j++) {
            for(int i=0; i<dimX; i++) {
                visited[{i, j}] = -1;
            }
        }

        int x, y;
        int ccNum = 0;
        std::stack<Point> stack_;
        for(int j=0; j<dimY; j++) {
            for(int i=0; i<dimX; i++) {
                const Point BASE(i, j);
                if(tileGraph.find(BASE) == tileGraph.end()) {
                    stack_.push(BASE);
                    ccNum++;
                }
                while(not stack_.empty()) {
                    Point p = stack_.top(); stack_.pop();
                    if(tileGraph.find(p) != tileGraph.end()) {
                        continue;
                    }

                    tileGraph[p] = TileNode(p);
                    const std::vector<Point> neighbors
                            = getNeighbors(p, tiles[p.first][p.second]);
                    for(Point const& n : neighbors) {
                        x = n.first; y = n.second;
                        assert(x >= 0 and x < dimX and y >= 0 and y < dimY);
//                        n_idx = visited[n];
                        if(tileGraph.find(n) == tileGraph.end()) {
                            stack_.push(n);
                        }
                        else {
                            tileGraph[p].appendLink(&tileGraph[n]);
                            tileGraph[n].appendLink(&tileGraph[p]);
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
    cout << car.getAngle() / (2 * PI) * 360 << endl;

    cout << impl->tileGraph[{0, 0}] << endl;

    exit(1);
}

MyStrategy::MyStrategy() : impl(new Impl) { }
