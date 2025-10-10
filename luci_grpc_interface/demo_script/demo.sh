#!/bin/bash

# Demo script for LUCI gRPC interface
# This script provides a menu-driven interface to interact with the LUCI gRPC interface
# through ROS2 publish commands and service calls

# Colors for better visualization
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Store PID of background joystick process
JOYSTICK_PID=""
GRPC_PID=""

# Check if ROS2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then
    echo -e "${RED}Error: ROS2 environment is not sourced. Please source ROS2 first.${NC}"
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${BLUE}==================================================${NC}"
echo -e "${GREEN}LUCI gRPC Interface Demo Script${NC}"
echo -e "${BLUE}==================================================${NC}"

# Function to start gRPC node
start_grpc_node() {
    if [[ -n "$GRPC_PID" ]]; then
        echo -e "${YELLOW}gRPC node is already running with PID: $GRPC_PID${NC}"
        return
    fi

    read -p "Enter IP address for gRPC node [default: 10.2.10.3]: " grpc_ip
    grpc_ip=${grpc_ip:-10.2.10.3}

    echo -e "${YELLOW}Starting LUCI gRPC node at IP: $grpc_ip...${NC}"
    # Launch the node in the background
    ros2 run luci_grpc_interface grpc_interface_node -a "$grpc_ip" >/dev/null 2>&1 &
    GRPC_PID=$!

    echo -e "${GREEN}gRPC node started with PID: $GRPC_PID${NC}"
    sleep 2  # Give the node time to initialize
}

# Function to stop gRPC node
stop_grpc_node() {
    if [[ -z "$GRPC_PID" ]]; then
        echo -e "${YELLOW}No gRPC node is currently running from this script.${NC}"
        return
    fi
    
    echo -e "${YELLOW}Stopping gRPC node (PID: $GRPC_PID)...${NC}"
    kill $GRPC_PID 2>/dev/null || true
    GRPC_PID=""
    echo -e "${GREEN}gRPC node stopped.${NC}"
}

# Function to stop any currently running joystick command
stop_joystick_publishing() {
    if [[ -n "$JOYSTICK_PID" ]]; then
        echo -e "${YELLOW}Stopping current joystick command...${NC}"
        kill $JOYSTICK_PID 2>/dev/null || true
        JOYSTICK_PID=""
        
        # Send a zero command to stop the robot
        ros2 topic pub --once /luci/remote_joystick luci_messages/msg/LuciJoystick "{forward_back: 0, left_right: 0, input_source: 1}"
    fi
}

# Function to publish joystick command continuously
publish_joystick_continuous() {
    local forward_back=$1
    local left_right=$2
    local input_source=$3

    # Stop any existing joystick command
    stop_joystick_publishing
    
    echo -e "${YELLOW}Starting continuous joystick command:${NC}"
    echo "Forward/Back: $forward_back, Left/Right: $left_right, Input Source: $input_source"
    
    # Start continuous publishing in background with rate flag
    (
        ros2 topic pub -r 10 /luci/remote_joystick luci_messages/msg/LuciJoystick "{forward_back: $forward_back, left_right: $left_right, input_source: $input_source}" >/dev/null 2>&1
    ) &
    
    JOYSTICK_PID=$!
    echo -e "${GREEN}Continuous joystick command started (PID: $JOYSTICK_PID)${NC}"
    echo -e "${YELLOW}Press option 5 to stop movement${NC}"
}

# Function to publish joystick command once
publish_joystick_once() {
    local forward_back=$1
    local left_right=$2
    local input_source=$3

    echo -e "${YELLOW}Publishing one-time joystick command:${NC}"
    echo "Forward/Back: $forward_back, Left/Right: $left_right, Input Source: $input_source"
    
    ros2 topic pub --once /luci/remote_joystick luci_messages/msg/LuciJoystick "{forward_back: $forward_back, left_right: $left_right, input_source: $input_source}"
}

# Function to call a service
call_service() {
    local service_name=$1
    
    echo -e "${YELLOW}Calling service:${NC} $service_name"
    ros2 service call $service_name std_srvs/srv/Empty
}

# Function to display active topics and their types
show_topics() {
    echo -e "${YELLOW}Active LUCI Topics:${NC}"
    ros2 topic list | grep luci | sort
    echo ""
    echo -e "${YELLOW}To view messages on a topic, use:${NC}"
    echo -e "ros2 topic echo /topic_name"
}

# Function to display available services
show_services() {
    echo -e "${YELLOW}Available LUCI Services:${NC}"
    ros2 service list | grep luci | sort
}

# Cleanup function to ensure we don't leave processes running
cleanup() {
    echo -e "${YELLOW}Cleaning up...${NC}"
    stop_joystick_publishing
    stop_grpc_node
    exit 0
}

# Set up trap to catch ctrl+c and other termination signals
trap cleanup EXIT INT TERM

# Main menu loop
while true; do
    movement_status=""
    if [[ -n "$JOYSTICK_PID" ]]; then
        movement_status="${YELLOW}[ROBOT MOVING]${NC} "
    fi
    
    echo -e "\n${BLUE}=== LUCI Interface Demo Menu ===${NC} $movement_status"
    
    echo -e "${BLUE}--- gRPC Node Control ---${NC}"
    echo "S. Start gRPC node"
    echo "T. Stop gRPC node"
    
    echo -e "${BLUE}--- Joystick Commands ---${NC}"
    echo "1. Move FORWARD (continuous until stopped)"
    echo "2. Move BACKWARD (continuous until stopped)"
    echo "3. Move LEFT (continuous until stopped)"
    echo "4. Move RIGHT (continuous until stopped)"
    echo "5. STOP movement"
    
    echo -e "${BLUE}--- Service Calls ---${NC}"
    echo "6. Set shared remote input source"
    echo "7. Remove shared remote input source"
    echo "8. Set auto remote input source"
    echo "9. Remove auto remote input source"
    
    echo -e "${BLUE}--- Information ---${NC}"
    echo "10. Show active LUCI topics"
    echo "11. Show available LUCI services"
    echo "12. Custom joystick command (one-time)"
    echo "13. Custom continuous joystick command"
    echo "0. Exit"
    
    read -p "Choose an option: " choice
    
    case $choice in
        S|s)
            start_grpc_node
            ;;
        T|t)
            stop_grpc_node
            ;;
        1)
            publish_joystick_continuous 40 0 1  # forward, no left/right, shared remote
            ;;
        2)
            publish_joystick_continuous -40 0 1  # backward, no left/right, shared remote
            ;;
        3)
            publish_joystick_continuous 0 -40 1  # no forward/back, left, shared remote
            ;;
        4)
            publish_joystick_continuous 0 40 1  # no forward/back, right, shared remote
            ;;
        5)
            stop_joystick_publishing
            echo -e "${GREEN}Robot stopped${NC}"
            ;;
        6)
            call_service /luci/set_shared_remote_input
            ;;
        7)
            call_service /luci/remove_shared_remote_input
            ;;
        8)
            call_service /luci/set_auto_remote_input
            ;;
        9)
            call_service /luci/remove_auto_remote_input
            ;;
        10)
            show_topics
            ;;
        11)
            show_services
            ;;
        12)
            echo "Custom joystick command (one-time):"
            read -p "Forward/Back (-100 to 100): " fb
            read -p "Left/Right (-100 to 100): " lr
            read -p "Input Source (1=SharedRemote, 2=AutonomousRemote): " src
            publish_joystick_once $fb $lr $src
            ;;
        13)
            echo "Custom continuous joystick command:"
            read -p "Forward/Back (-100 to 100): " fb
            read -p "Left/Right (-100 to 100): " lr
            read -p "Input Source (1=SharedRemote, 2=AutonomousRemote): " src
            publish_joystick_continuous $fb $lr $src
            ;;
        0)
            echo -e "${GREEN}Exiting demo script. Goodbye!${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option. Please try again.${NC}"
            ;;
    esac
    
    # Pause to see the output, but don't pause if we're in continuous mode
    if [[ -z "$JOYSTICK_PID" ]]; then
        read -p "Press Enter to continue..."
    fi
done