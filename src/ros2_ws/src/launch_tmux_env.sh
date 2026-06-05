#!/bin/bash

# Define the sessions
sessions="sim rviz other debug dubins"

for session in $sessions; do
    # Create a new detached session
    tmux new-session -d -s "$session"

    # Configure the session based on its name
    case "$session" in
        sim)
            tmux send-keys -t "$session" "ros2 launch linc_gz docking_harner.launch.py"
            ;;
        other)
            # First command in the initial window
            #tmux send-keys -t "$session" "ros2 run frontseat twist_to_diff_drive" C-m
            
            # Second command in a new vertical split
            tmux split-window -v -t "$session"
            tmux send-keys -t "$session" "ros2 run frontseat gps_center_offset_node" C-m
                        
            # Adjust layout to be even vertical
            tmux select-layout -t "$session" even-vertical
            ;;
        rviz)
            # First command
            config_file="rviz2 -d /workspace/codebase/mini-bream/src/ros2_ws/src/visualization_tools/visualizers_config/rviz_config/default_dubins.rviz"
            tmux send-keys -t "$session" "rviz2 -d $config_file" C-m
            
            # Second command in a new vertical split
            tmux split-window -v -t "$session"
            tmux send-keys -t "$session" "ros2 launch visualization_tools visuals.launch.py" C-m
            
            # Adjust layout
            tmux select-layout -t "$session" even-vertical
            ;;
        dubins)
            tmux send-keys -t "$session" "ros2 launch backseat mission.launch.py" C-m
            
            tmux split-window -v -t "$session"
            tmux send-keys -t "$session" "ros2 launch mission_planner action_client.launch.py" C-m
            tmux split-window -v -t "$session"
            tmux send-keys -t "$session" "ros2 launch mission_planner moloplanner.launch.py" C-m
            ;;
        debug)
            # No specific commands for debug session
            ;;
    esac
done
# Attach to the dubins session
tmux new -As dubins
