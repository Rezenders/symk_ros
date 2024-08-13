// Copyright 2024 KAS Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_symk_planner/symk_planner.hpp"

namespace plansys2
{

SymkPlanner::SymkPlanner()
{
}

void SymkPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & lc_node,
  const std::string & plugin_name)
{
  parameter_name_ = plugin_name + ".arguments";
  lc_node_ = lc_node;
  lc_node_->declare_parameter<std::string>(parameter_name_, "");
}

std::optional<plansys2_msgs::msg::Plan>
SymkPlanner::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    std::filesystem::path tp = std::filesystem::temp_directory_path();
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        tp /= p;
      }
    }
    std::filesystem::create_directories(tp);
  }

  plansys2_msgs::msg::Plan ret;
  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();

  system(
    ("ros2 run symk_ros fast-downward.py /tmp/" +
    node_namespace + "/domain.pddl /tmp/" + node_namespace +
    "/problem.pddl --search 'sym_bd()'" + lc_node_->get_parameter(parameter_name_).value_to_string()+
    " > /tmp/" + node_namespace + "/plan")
    .c_str());

  std::string line;
  std::ifstream plan_file("/tmp/" + node_namespace + "/plan");
  bool solution = false;
  float time = 0;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Best plan") != std::string::npos) {
          solution = true;
        }
      } else if (line.front() != '[') {
        plansys2_msgs::msg::PlanItem item;
        size_t bracket_pos = line.find("(");

        std::string action = "(" + line.substr(0, bracket_pos-1)+")";

        item.time = time;
        item.action = action;
        time += 1.0;

        ret.items.push_back(item);
      } else if (line.front() == '[') {
        break;
      }
    }
    plan_file.close();
  }

  if (ret.items.empty()) {
    return {};
  } else {
    return ret;
  }
}

bool
SymkPlanner::is_valid_domain(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
  }

  std::ofstream domain_out("/tmp/" + node_namespace + "/check_domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/check_problem.pddl");
  problem_out << "(define (problem void) (:domain simple) (:objects) (:init) (:goal))";
  problem_out.close();

  system(
    ("ros2 run symk_ros fast-downward.py /tmp/" +
    node_namespace + "/check_domain.pddl /tmp/" +
    node_namespace + "/check_problem.pddl --search 'sym_bd()' > /tmp/" +
    node_namespace + "/check.out").c_str());

  std::ifstream plan_file("/tmp/" + node_namespace + "/check.out");

  std::string result((std::istreambuf_iterator<char>(plan_file)),
    std::istreambuf_iterator<char>());

  return result.find("Could not parse domain file") == result.npos;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::SymkPlanner, plansys2::PlanSolverBase);
