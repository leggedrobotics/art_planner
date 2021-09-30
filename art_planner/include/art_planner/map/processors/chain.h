#pragma once

#include <memory>

#include <grid_map_core/GridMap.hpp>




namespace art_planner {

namespace processors {

using GridMapPtr = std::unique_ptr<grid_map::GridMap>;



template <typename ...Args>
class Chain {

  using ProcFunc = std::function<void (Args...)>;

  std::vector<ProcFunc> functions_;

  public:

  void process(Args... args) {
    for (auto&& func: functions_) {
      func(args...);
    }
  }

  void appendFunction(const ProcFunc& func) {
    functions_.push_back(func);
  }

  void appendFunction(ProcFunc&& func) {
    functions_.push_back(std::move(func));
  }

};

// Args passed = (map_new).
//using ChainNewMap = Chain<const GridMapPtr&>;
using ChainNewMap = Chain<const GridMapPtr&>;

// Args passed = (map_new, map_old).
using ChainOldMap = Chain<const GridMapPtr&, const GridMapPtr&>;



} // namespace processors

} // namespace art_planner
