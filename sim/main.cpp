#include "environment/environment.hpp"
#include "simulation/initialization.hpp"
#include <iostream>
#include <memory>

int main() {
    std::unique_ptr<aocs::simulation::Simulation> simulation =
        aocs::simulation::initializeSimulation();
    if (!simulation) {
        return 1;
    }

    return aocs::simulation::runEnvironment(*simulation);
}
