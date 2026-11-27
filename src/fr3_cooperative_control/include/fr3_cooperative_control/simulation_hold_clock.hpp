#pragma once

#include <cmath>

namespace fr3_cooperative_control
{
enum class HoldClockResult {waiting, complete, stalled, regressed, wall_limit, invalid};

// A slow but advancing simulation differs from a stopped or reset clock.
class SimulationHoldClock
{
public:
  HoldClockResult observe(double sim_elapsed, double wall_elapsed, double required)
  {
    if (!std::isfinite(sim_elapsed) || !std::isfinite(wall_elapsed) ||
      !std::isfinite(required) || wall_elapsed < 0 || required <= 0)
    {
      return HoldClockResult::invalid;
    }
    if (sim_elapsed < last_sim_ - 1e-9) {return HoldClockResult::regressed;}
    if (sim_elapsed > last_sim_ + 1e-9) {last_progress_wall_ = wall_elapsed;}
    last_sim_ = sim_elapsed;
    idle_wall_ = wall_elapsed - last_progress_wall_;
    if (wall_elapsed >= 120.0) {return HoldClockResult::wall_limit;}
    if (idle_wall_ >= 30.0) {return HoldClockResult::stalled;}
    return sim_elapsed >= required ? HoldClockResult::complete : HoldClockResult::waiting;
  }

  double idleWallSeconds() const {return idle_wall_;}
  double simElapsedSeconds() const {return last_sim_;}

  static const char * name(HoldClockResult result)
  {
    switch (result) {
      case HoldClockResult::waiting: return "waiting";
      case HoldClockResult::complete: return "complete";
      case HoldClockResult::stalled: return "clock_stalled";
      case HoldClockResult::regressed: return "clock_regressed";
      case HoldClockResult::wall_limit: return "wall_budget_exhausted";
      default: return "invalid_time";
    }
  }

private:
  double last_sim_ = 0.0;
  double last_progress_wall_ = 0.0;
  double idle_wall_ = 0.0;
};
}  // namespace fr3_cooperative_control
