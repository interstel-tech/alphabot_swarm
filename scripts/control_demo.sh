#!/bin/bash
# Control demo script for testing the swarm controller with the AlphaBot2 robots
# Usage: ./control_demo.sh
# Make sure to give execute permissions: chmod +x control_demo.sh
# This script assumes that the swarm controller and robots are set up and the
# `agent` binary is available in the PATH.

pause_for_key() {
    read -n 1 -s -r -p "Press any key to continue..."
    echo
}

# Start from vertical line formation
echo "Starting control demo..."
echo "Starting in line formation"
agent main controller set_swarm_formation '{"shape":"line","sep":0.75}'
agent main controller set_swarm_position '{"angle":-90,"pos":{"x":0.5,"y":0.25}}'
pause_for_key
# Move forward in a straight line
echo "Moving forward in line formation"
agent main controller set_swarm_position '{"angle":-90,"pos":{"x":1.5,"y":0.25}}'
pause_for_key
# Move to triangle formation
echo "Switching to triangle formation"
agent main controller set_swarm_formation '{"shape":"ngon","sep":0.4}'
agent main controller set_swarm_position '{"angle":-120,"pos":{"x":2.5,"y":1.0}}'
pause_for_key
# Move forward
echo "Moving forward in triangle formation"
agent main controller set_swarm_position '{"angle":-120,"pos":{"x":3.0,"y":1.0}}'
pause_for_key
# Move backward
echo "Moving backward in triangle formation"
agent main controller set_swarm_position '{"angle":-120,"pos":{"x":2.5,"y":1.0}}'
pause_for_key
# Move into line formation
echo "Switching back to line formation"
agent main controller set_swarm_formation '{"shape":"line","sep":0.75}'
agent main controller set_swarm_position '{"angle":-90,"pos":{"x":1.5,"y":0.25}}'
pause_for_key
# Move backward in line formation
echo "Moving backward in line formation"
agent main controller set_swarm_position '{"angle":-90,"pos":{"x":0.5,"y":0.25}}'
