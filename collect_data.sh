#!/bin/bash

# Create session
session_name="vnav_$(date +%s)"
tmux new-session -d -s $session_name

# Split into four panes
tmux selectp -t 0    
tmux splitw -h -p 50 
tmux selectp -t 0    
tmux splitw -v -p 50 
tmux selectp -t 1    
tmux splitw -v -p 50 

# [0] Roslaunch
tmux select-pane -t 0
tmux send-keys "roslaunch rover.launch" Enter

# [1] gtpose_publisher.py
tmux select-pane -t 1
tmux send-keys "python gtpose_publisher.py" Enter

# [2] Data collector
tmux select-pane -t 2
tmux send-keys "python data_collector.py" Enter

# [3] T265 tracker
tmux select-pane -t 3
tmux send-keys roslaunch realsense2_camera rs_t265.launch

# Attach to the tmux session
tmux -2 attach-session -t $session_name
