# rosservice call /rosplan_problem_interface/problem_generation_server
# rosservice call /rosplan_planner_interface/planning_server

# printf "------PROBLEM:\n"
# rostopic echo /rosplan_problem_interface/problem_instance -n 1
# printf "\n\n------PLAN:\n"
# rostopic echo /rosplan_planner_interface/planner_output -n 1
# printf "\n"


echo "Generating a Problem"
rosservice call /rosplan_problem_interface/problem_generation_server

echo "Planning"
rosservice call /rosplan_planner_interface/planning_server

echo "Executing the Plan"
rosservice call /rosplan_parsing_interface/parse_plan
rosservice call /rosplan_plan_dispatcher/dispatch_plan