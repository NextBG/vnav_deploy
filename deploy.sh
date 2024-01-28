#!/bin/bash

# Create session
session_name="vnav_$(date +%s)"
tmux new-session -d -s $session_name

# Split into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# [1] Roslaunch
tmux select-pane -t 0
tmux send-keys "roslaunch rover.launch" Enter

# [2] rover_controller.py
tmux select-pane -t 1
tmux send-keys "python rover_controller.py" Enter

# [3] gtpose_publisher.py
tmux select-pane -t 2
tmux send-keys "python gtpose_publisher.py" Enter

# [4] 
tmux select-pane -t 3

# Attach to the tmux session
tmux -2 attach-session -t $session_name
