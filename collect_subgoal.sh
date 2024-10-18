#!/bin/bash

# Create session
session_name="vnav_subgoal_collector_$(date +%s)"
tmux new-session -d -s $session_name

# Split into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# [0] Roslaunch
tmux select-pane -t 0
tmux send-keys "roslaunch rover.launch" Enter

# [1] gtpose_publisher.py
tmux select-pane -t 1
tmux send-keys "python gtpose_publisher.py" Enter

# [2] rover_controller.py
tmux select-pane -t 2
tmux send-keys "python rover_controller.py" Enter

# [3] collect_subgoal.py
tmux select-pane -t 3
tmux send-keys "python collect_subgoal.py" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
