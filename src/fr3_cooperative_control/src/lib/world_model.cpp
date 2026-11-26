#include "fr3_cooperative_control/world_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometric_shapes/shapes.h>
#include <moveit/robot_state/robot_state.hpp>

namespace fr3_cooperative_control
{
namespace
{
// Every box collision body of a link, as world-aligned boxes (fixed environment links only).
std::vector<WorldBox> linkBoxes(
  const moveit::core::RobotState & state, const std::string & link_name)
{
  const auto * link = state.getRobotModel()->getLinkModel(link_name);
  if (!link) {throw std::runtime_error("World model: link '" + link_name + "' not found");}
  const Eigen::Isometry3d & link_pose = state.getGlobalLinkTransform(link);
  const auto & shapes = link->getShapes();
  const auto & origins = link->getCollisionOriginTransforms();
  std::vector<WorldBox> boxes;
  for (size_t i = 0; i < shapes.size(); ++i) {
    if (shapes[i]->type != shapes::BOX) {continue;}
    const auto * box = static_cast<const shapes::Box *>(shapes[i].get());
    const Eigen::Isometry3d pose = link_pose * origins[i];
    WorldBox world;
    world.center = pose.translation();
    world.size = pose.linear().cwiseAbs() * Eigen::Vector3d(box->size[0], box->size[1], box->size[2]);
    boxes.push_back(world);
  }
  return boxes;
}
}  // namespace

const WorldBox & Shelf::board(int level) const
{
  if (level < 1 || level > static_cast<int>(boards.size())) {
    throw std::runtime_error(name + ": no board level " + std::to_string(level));
  }
  return boards[level - 1];
}

double Shelf::frontY() const
{
  const WorldBox & any = boards.front();
  return any.center.y() - inward() * 0.5 * any.size.y();
}

TaskConfig TaskConfig::fromParameters(const rclcpp::Node & node)
{
  TaskConfig c;
  node.get_parameter_or("source_shelf", c.source_shelf, c.source_shelf);
  node.get_parameter_or("source_level", c.source_level, c.source_level);
  node.get_parameter_or("target_shelf", c.target_shelf, c.target_shelf);
  node.get_parameter_or("target_level", c.target_level, c.target_level);
  node.get_parameter_or("source_dx", c.source_dx, c.source_dx);
  node.get_parameter_or("target_dx", c.target_dx, c.target_dx);
  node.get_parameter_or("handoff_x", c.handoff_x, c.handoff_x);
  node.get_parameter_or("handoff_y", c.handoff_y, c.handoff_y);
  node.get_parameter_or("cell_inset", c.cell_inset, c.cell_inset);
  node.get_parameter_or("box_size", c.box_size, c.box_size);
  node.get_parameter_or("gazebo_z_offset", c.gazebo_z_offset, c.gazebo_z_offset);
  node.get_parameter_or("speed_scale", c.speed_scale, c.speed_scale);
  node.get_parameter_or("rigid_grasp", c.rigid_grasp, c.rigid_grasp);
  node.get_parameter_or("verify", c.verify, c.verify);
  return c;
}

WorldModel::WorldModel(const moveit::core::RobotModelConstPtr & model, const TaskConfig & task)
: task_(task)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();

  // Tabletop: the widest box of table_link.
  const auto table = linkBoxes(state, "table_link");
  if (table.empty()) {throw std::runtime_error("World model: table_link has no box collision");}
  table_ = *std::max_element(table.begin(), table.end(), [](const auto & a, const auto & b) {
      return a.size.x() * a.size.y() < b.size.x() * b.size.y();
    });

  for (const std::string name : {"shelf_1", "shelf_2"}) {
    Shelf shelf;
    shelf.name = name;
    shelf.origin = state.getGlobalLinkTransform(shelf.link()).translation();
    for (const auto & box : linkBoxes(state, shelf.link())) {
      if (box.size.z() < 0.05 && box.size.x() > 0.2 && box.size.y() > 0.2) {
        shelf.boards.push_back(box);
      } else if (box.size.z() > 0.5) {
        shelf.posts.push_back(box);
      }
    }
    std::sort(shelf.boards.begin(), shelf.boards.end(), [](const auto & a, const auto & b) {
        return a.center.z() < b.center.z();
      });
    if (shelf.boards.empty()) {throw std::runtime_error("World model: " + name + " has no boards");}
    shelves_.push_back(shelf);
  }
}

const Shelf & WorldModel::shelf(const std::string & name) const
{
  for (const auto & s : shelves_) {
    if (s.name == name) {return s;}
  }
  throw std::runtime_error("World model: unknown shelf '" + name + "'");
}

double WorldModel::boxHalfDiagonal() const {return boxHalf() * std::sqrt(2.0);}

Cell WorldModel::shelfCell(const std::string & name, int level) const
{
  return shelfCell(name, level, 0.0, task_.cell_inset);
}

Cell WorldModel::shelfCell(const std::string & name, int level, double dx, double inset) const
{
  const Shelf & s = shelf(name);
  const WorldBox & board = s.board(level);
  Cell cell;
  cell.surface = board.top();
  cell.center = Eigen::Vector3d(
    board.center.x() + dx, s.frontY() + s.inward() * inset, cell.surface + boxHalf());
  return cell;
}

Cell WorldModel::tableCell(double x, double y) const
{
  Cell cell;
  cell.surface = table_.top();
  cell.center = Eigen::Vector3d(x, y, cell.surface + boxHalf());
  return cell;
}

Cell WorldModel::handoffCell() const {return tableCell(task_.handoff_x, task_.handoff_y);}

bool WorldModel::overTable(const Eigen::Vector3d & p, double margin) const
{
  return std::abs(p.x() - table_.center.x()) <= 0.5 * table_.size.x() + margin &&
         std::abs(p.y() - table_.center.y()) <= 0.5 * table_.size.y() + margin;
}

}  // namespace fr3_cooperative_control
