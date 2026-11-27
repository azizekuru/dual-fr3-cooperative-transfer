#include <iostream>
#include <limits>
#include <stdexcept>
#include "fr3_cooperative_control/simulation_hold_clock.hpp"

using fr3_cooperative_control::HoldClockResult;
using fr3_cooperative_control::SimulationHoldClock;

void check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main()
{
  try {
    SimulationHoldClock normal;
    check(normal.observe(0.0, 0.0, 1.0) == HoldClockResult::waiting, "normal start");
    check(normal.observe(0.5, 0.7, 1.0) == HoldClockResult::waiting, "normal progress");
    check(normal.observe(1.0, 1.4, 1.0) == HoldClockResult::complete, "normal completion");

    SimulationHoldClock slow;
    for (int wall = 0; wall < 50; ++wall) {
      check(slow.observe(wall * 0.02, wall, 1.0) == HoldClockResult::waiting,
        "slow live clock must keep waiting beyond 30 wall seconds");
    }
    check(slow.observe(1.0, 50.0, 1.0) == HoldClockResult::complete, "slow completion");

    SimulationHoldClock frozen;
    check(frozen.observe(0.2, 2.0, 1.0) == HoldClockResult::waiting, "freeze start");
    check(frozen.observe(0.2, 31.9, 1.0) == HoldClockResult::waiting, "freeze below limit");
    check(frozen.observe(0.2, 32.0, 1.0) == HoldClockResult::stalled, "frozen clock limit");

    SimulationHoldClock reset;
    check(reset.observe(0.8, 1.0, 1.0) == HoldClockResult::waiting, "reset start");
    check(reset.observe(0.1, 2.0, 1.0) == HoldClockResult::regressed, "backward reset");

    SimulationHoldClock budget;
    for (int wall = 0; wall < 120; ++wall) {
      check(budget.observe(wall * 0.001, wall, 1.0) == HoldClockResult::waiting,
        "very slow clock before hard budget");
    }
    check(budget.observe(0.12, 120, 1.0) == HoldClockResult::wall_limit, "hard budget");

    SimulationHoldClock invalid;
    check(invalid.observe(std::numeric_limits<double>::quiet_NaN(), 0, 1) ==
      HoldClockResult::invalid, "nonfinite time");
    check(invalid.observe(0, -1, 1) == HoldClockResult::invalid, "negative wall time");
    std::cout << "Simulation hold clock checks passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';return 1;
  }
  return 0;
}
