#!/bin/bash

# Create session
session_name="vnav_$(date +%s)"
tmux new-session -d -s $session_name

# Split into four panes
tmux selectp -t 0    
tmux splitw -h -p 50 
tmux selectp -t 0    
tmux splitw -v -p 50 
tmux selectp -t 2    
tmux splitw -v -p 50 

# [0] Roslaunch
tmux select-pane -t 0
tmux send-keys "roslaunch rover.launch" Enter

# Delay for roslaunch to start
sleep 3

# [1] Rover control
tmux select-pane -t 1
tmux send-keys "python rover_controller_rconly.py" Enter

# [2] Data collector
tmux select-pane -t 2
tmux send-keys "python data_collector.py --interval 0.5 --n_points 1000" Enter

# [1]
tmux select-pane -t 3

# Attach to the tmux session
tmux -2 attach-session -t $session_name
