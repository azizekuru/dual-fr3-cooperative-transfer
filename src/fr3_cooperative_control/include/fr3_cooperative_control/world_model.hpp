// World geometry (table, shelves, boards, posts) read from the MoveIt robot model, plus the
// task cells derived from it. No coordinates are hard-coded in the task nodes.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fr3_cooperative_control
{

// Axis-aligned box in the world frame.
struct WorldBox
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  double top() const {return center.z() + 0.5 * size.z();}
  double bottom() const {return center.z() - 0.5 * size.z();}
};

struct Shelf
{
  std::string name;                 // e.g. "shelf_2" (link "shelf_2_link")
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  std::vector<WorldBox> boards;     // ordered bottom (level 1) to top
  std::vector<WorldBox> posts;
  std::string link() const {return name + "_link";}
  // Collision name of a board as it appears in Gazebo contact messages.
  std::string boardCollision(int level) const
  {
    return name + "_level_" + std::to_string(level) + "_collision";
  }
  const WorldBox & board(int level) const;
  // +1 if the shelf interior lies towards +y from the robots (y = 0), else -1.
  double inward() const {return origin.y() >= 0 ? 1.0 : -1.0;}
  // Board edge facing the robots.
  double frontY() const;
};

// A place where the box rests: its centre and the supporting surface height.
struct Cell
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // resting box centre
  double surface = 0.0;                              // supporting surface (board/table top)
};

struct TaskConfig
{
  std::string source_shelf = "shelf_1";
  int source_level = 2;
  std::string target_shelf = "shelf_2";
  int target_level = 2;
  double handoff_x = 0.0;    // box centre on the table, world frame
  double handoff_y = 0.05;
  double cell_inset = 0.03;  // box centre behind the board's front edge
  double box_size = 0.04;
  double gazebo_z_offset = 0.05;  // Gazebo world z minus MoveIt world z
  double speed_scale = 1.0;       // test mode: multiplies every motion speed (capped at 1.0)
  bool rigid_grasp = false;       // test mode: weld the box to the hand while grasped

  static TaskConfig fromParameters(const rclcpp::Node & node);
};

class WorldModel
{
public:
  WorldModel(const moveit::core::RobotModelConstPtr & model, const TaskConfig & task);

  const TaskConfig & task() const {return task_;}
  const WorldBox & table() const {return table_;}
  const std::vector<Shelf> & shelves() const {return shelves_;}
  const Shelf & shelf(const std::string & name) const;

  double boxHalf() const {return 0.5 * task_.box_size;}
  double boxHalfDiagonal() const;

  Cell shelfCell(const std::string & shelf, int level) const;
  // Cell `dx` metres sideways from the board centre and `inset` metres behind the front edge.
  Cell shelfCell(const std::string & shelf, int level, double dx, double inset) const;
  Cell tableCell(double x, double y) const;
  Cell sourceCell() const {return shelfCell(task_.source_shelf, task_.source_level);}
  Cell targetCell() const {return shelfCell(task_.target_shelf, task_.target_level);}
  Cell handoffCell() const;
  const Shelf & targetShelf() const {return shelf(task_.target_shelf);}

  // Table footprint grown by `margin` (x/y only).
  bool overTable(const Eigen::Vector3d & p, double margin) const;

private:
  TaskConfig task_;
  WorldBox table_;
  std::vector<Shelf> shelves_;
};

}  // namespace fr3_cooperative_control
