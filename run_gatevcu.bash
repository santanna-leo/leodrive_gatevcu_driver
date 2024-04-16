#!/bin/bash

# Help message function
print_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Activate a CAN interface and launch a ROS 2 command."
    echo ""
    echo "Options:"
    echo "  -h, --help            Show this help message and exit"
    echo "  -i, --interface       Name of the CAN interface (default: can0)"
    echo "  -b, --bitrate         Bitrate of the CAN interface (default: 500000)"
    echo "  -j, --launch-joy      Whether to launch joy stack or not (default: true)"
    echo ""
    echo "After setting up the network interface, this script searches for the 'setup.bash' script"
    echo "in parent directories recursively. Once found, it sources the 'setup.bash' script and"
    echo "launches the 'ros2 launch leodrive_gatevcu_driver driver.launch.xml' command."
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -h|--help)
            print_help
            ;;
        -i|--interface)
            interface="$2"
            shift
            ;;
        -b|--bitrate)
            bitrate="$2"
            shift
            ;;
        -j|--launch-joy)
            launch_joy="$2"
            shift
            ;;
        *)
            echo "Unknown option: $key"
            exit 1
            ;;
    esac
    shift
done

# Default values
interface="${interface:-can0}"
bitrate="${bitrate:-500000}"
launch_joy="${launch_joy:-true}"

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
