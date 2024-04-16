#!/bin/bash

# Help message function
print_help() {
    echo "Usage: $0 [INTERFACE] [BITRATE] [LAUNCH_JOY]"
    echo "Activate a CAN interface with optional INTERFACE name and BITRATE."
    echo "If INTERFACE and BITRATE are not provided, defaults to 'can0' and '500000' respectively."
    echo "Options:"
    echo "  INTERFACE  Name of the CAN interface (default: can0)"
    echo "  BITRATE    Bitrate of the CAN interface (default: 500000)"
    echo "  LAUNCH_JOY Whether to launch joy stack or not (default: true)"
    echo ""
    echo "After setting up the network interface, this script searches for the 'setup.bash' script"
    echo "in parent directories recursively. Once found, it sources the 'setup.bash' script and"
    echo "launches the 'ros2 launch leodrive_gatevcu_driver driver.launch.xml' command."
    exit 0
}

# Check if help flag is provided
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    print_help
fi

# Default values
interface="can0"
bitrate="500000"
launch_joy="true"

# Check if interface and bitrate are specified as arguments
if [ $# -ge 1 ]; then
    interface="$1"
fi

if [ $# -ge 2 ]; then
    bitrate="$2"
fi

if [ $# -ge 3 ]; then
    launch_joy="$3"
fi

# Check if the specified interface exists
if ! ip link show "$interface" &>/dev/null; then
    echo "Error: Interface '$interface' does not exist."
    exit 1
fi

# Check if the interface is already up
if ip link show "$interface" | grep -q "state UP"; then
    echo "Interface '$interface' is already up."
else
    # Try to activate the interface
    if sudo ip link set "$interface" up type can bitrate "$bitrate"; then
        echo "Interface '$interface' activated successfully with bitrate $bitrate."
    else
        echo "Error: Failed to activate interface '$interface'."
        exit 1
    fi
fi

# Find and source setup.bash
found=false
dir="$(pwd)"
while [ "$dir" != "/" ]; do
    setup_script="${dir}/install/setup.bash"
    if [ -f "$setup_script" ]; then
        found=true
        break
    fi
    dir="$(dirname "$dir")"
done

if [ "$found" = false ]; then
    echo "Error: setup.bash script not found."
    exit 1
fi

# shellcheck source=../../install/setup.bash
source "$setup_script"

# Run the ROS 2 command
ros2 launch leodrive_gatevcu_driver driver.launch.xml can_interface:="$interface" launch_joy:="$launch_joy"
