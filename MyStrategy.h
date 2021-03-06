#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <memory>

class MyStrategy : public Strategy {
public:
    class Impl;
    std::shared_ptr<Impl> impl;

    MyStrategy();

    void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& move) override;
};

#endif
