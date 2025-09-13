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

agent main controller set_swarm_formation '{"shape":"line","sep":1.0}'
agent main controller set_swarm_position '{"angle":0,"pos":{"x":2.0,"y":1.25}}'
pause_for_key
agent main controller set_swarm_position '{"angle":0,"pos":{"x":1.5,"y":0.75}}'
agent main controller set_swarm_formation '{"shape":"ngon","sep":0.75}'
pause_for_key
agent main controller set_swarm_position '{"angle":60,"pos":{"x":1.5,"y":0.75}}'
pause_for_key
agent main controller set_swarm_position '{"angle":120,"pos":{"x":1.5,"y":0.75}}'
pause_for_key
agent main controller set_swarm_position '{"angle":180,"pos":{"x":1.5,"y":0.75}}'
pause_for_key
agent main controller set_swarm_position '{"angle":180,"pos":{"x":2.0,"y":0.75}}'
pause_for_key
agent main controller set_swarm_formation '{"shape":"line","sep":1.0}'
agent main controller set_swarm_position '{"angle":0,"pos":{"x":2.0,"y":1.25}}'
