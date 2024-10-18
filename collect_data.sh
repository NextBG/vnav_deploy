#!/bin/bash

# Create session
session_name="vnav_$(date +%s)"
tmux new-session -d -s $session_name

# Split into four panes
tmux selectp -t 0    
tmux splitw -h -p 40 
tmux selectp -t 0    
tmux splitw -v -p 50 
tmux selectp -t 0    
tmux splitw -v -p 50 
tmux selectp -t 2    
tmux splitw -v -p 50 
tmux selectp -t 4  
tmux splitw -v -p 80

# [0] Roslaunch
tmux select-pane -t 0
tmux send-keys "roslaunch rover.launch" Enter

# [1] rover_controller.py # NOT NEED
tmux select-pane -t 1
tmux send-keys "python rover_controller.py" Enter

# [2] gtpose_publisher.py
tmux select-pane -t 2
tmux send-keys "python gtpose_publisher.py" Enter

# [3] subgoal_publisher.py  # NOT NEED
tmux select-pane -t 3
tmux send-keys "python subgoal_publisher.py" Enter

# [4] plot_subgoals.py # NOT NEED
tmux select-pane -t 4
tmux send-keys "python plot_subgoals.py" Enter
 
# [5] vnav_inferencer.py # NOT NEED
tmux select-pane -t 5
tmux send-keys "python vnav_inferencer.py" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
