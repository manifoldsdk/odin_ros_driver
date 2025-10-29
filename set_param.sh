#!/bin/bash

# Usage: ./set_param.sh <parameter_name> <value>
# Example: ./set_param.sh save_map 1

if [ $# -ne 2 ]; then
    echo "Usage: $0 <parameter_name> <value>"
    echo "Example: $0 save_map 1"
    exit 1
fi

PARAM_NAME=$1
VALUE=$2

COMMAND_FILE="/tmp/odin_command.txt"

# Create the command file with the parameter
echo "set $PARAM_NAME $VALUE" > "$COMMAND_FILE"

echo "Command sent: set $PARAM_NAME $VALUE"
echo "Command file: $COMMAND_FILE"
