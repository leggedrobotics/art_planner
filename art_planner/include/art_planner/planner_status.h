#pragma once



namespace art_planner {



enum PlannerStatus {
  UNKNOWN = 0,
  INVALID_START,
  INVALID_GOAL,
  NO_MAP,
  NOT_SOLVED,
  SOLVED
};



}
