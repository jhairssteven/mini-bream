#!/bin/bash

SESSION="dubins_planner"

# Kill existing session if it exists
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t $SESSION
fi


# Start new tmux session
tmux new-session -d -s $SESSION -n main

# Rename initial pane to "top-left" (Pane 0)
tmux send-keys -t $SESSION 'cd /workspace/codebase/linc_ws' C-m
tmux send-keys -t $SESSION 'ros2 launch mission_planner action_client.launch.py'

# Split pane 0 vertically into left and right (Pane 1 becomes the right half)
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION.1 'cd /workspace/codebase' C-m
tmux send-keys -t $SESSION.1 'ros2 service call /rvizvisualsadapter/rviz_path std_srvs/srv/Trigger'

# Split left pane (Pane 0) horizontally to make middle-left (Pane 2)
tmux split-window -v -t $SESSION.0
tmux send-keys -t $SESSION.2 'cd /workspace/codebase/linc_ws' C-m
tmux send-keys -t $SESSION.2 'ros2 launch backseat mission.launch.py'

# Split again the top-left pane (now Pane 0) to make bottom-left (Pane 3)
tmux split-window -v -t $SESSION.0
tmux send-keys -t $SESSION.3 'cd /workspace/codebase/linc_ws' C-m
tmux send-keys -t $SESSION.3 'ros2 launch visualization_tools visuals.launch.py'

# Optional: select the main right pane at the end
tmux select-pane -t $SESSION.1

# Attach to session
tmux attach-session -t $SESSION
