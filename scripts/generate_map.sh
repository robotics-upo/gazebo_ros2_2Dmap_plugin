#!/bin/bash
# Script to generate and save a 2D occupancy map from a Gazebo world file
#
# Usage: ./generate_map.sh <world_file> [output_directory]
#
# Example: ./generate_map.sh ~/my_world.sdf ./maps
#
# The output map will have the same name as the world file (without extension)

set -e

# Check arguments
if [ $# -lt 1 ]; then
    echo "Usage: $0 <world_file> [output_directory]"
    echo "Example: $0 ~/my_world.sdf ./maps"
    echo ""
    echo "The output map will automatically be named after the world file."
    echo "For example: my_world.sdf -> my_world.yaml + my_world.pgm"
    exit 1
fi

WORLD_FILE="$1"
OUTPUT_DIR="${2:-.}"  # Default to current directory if not specified

# Extract map name from world file (remove path and extension)
MAP_NAME=$(basename "$WORLD_FILE" | sed 's/\.[^.]*$//')

# Check if world file exists
if [ ! -f "$WORLD_FILE" ]; then
    echo "Error: World file '$WORLD_FILE' not found!"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Create a temporary world file with the plugin injected
TEMP_WORLD_FILE="/tmp/gazebo_map_gen_world_$(date +%s).world"
PLUGIN_INJECTED=false

# Check if the world file already has the plugin
if grep -q "libgazebo_2Dmap_plugin.so" "$WORLD_FILE"; then
    echo "Plugin already present in world file"
    TEMP_WORLD_FILE="$WORLD_FILE"
else
    echo "Injecting occupancy map plugin into world file..."
    
    # Minimal plugin configuration - uses all default parameters
    PLUGIN_XML="  <plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'></plugin>"
    
    # Find the last </world> tag and insert plugin before it
    # Using awk to handle the insertion properly
    awk -v plugin="$PLUGIN_XML" '
        /<\/world>/ && !done {
            print plugin
            done=1
        }
        { print }
    ' "$WORLD_FILE" > "$TEMP_WORLD_FILE"
    
    PLUGIN_INJECTED=true
    echo "✓ Plugin injected into temporary world file"
fi

# Check for missing models and download if needed
echo "Checking for missing models..."
ALL_MODELS=$(grep -oP '(?<=model://)[^<]+' "$TEMP_WORLD_FILE" | sort -u || true)
MODEL_PATH="${HOME}/.gazebo/models"
MISSING_MODELS=()

# Initial check for missing models
for model in $ALL_MODELS; do
    if [ ! -d "$MODEL_PATH/$model" ]; then
        MISSING_MODELS+=("$model")
    fi
done

# Function to download specific models from a git repo using sparse-checkout
download_from_git() {
    local REPO_URL="$1"
    local BRANCH="$2"
    local PREFIX="$3"  # e.g., "models/" for AWS, or empty for OSRF
    local REPO_NAME=$(basename "$REPO_URL" .git)
    local TEMP_REPO="/tmp/gazebo_model_dl_$REPO_NAME"
    
    if [ ${#MISSING_MODELS[@]} -eq 0 ]; then
        return
    fi
    
    echo "  → Searching in $REPO_NAME..."
    
    rm -rf "$TEMP_REPO"
    mkdir -p "$TEMP_REPO"
    cd "$TEMP_REPO"
    
    git init -q
    git remote add origin "$REPO_URL"
    git config core.sparseCheckout true
    
    # Add potential paths to sparse-checkout
    local FOUND_POTENTIAL=false
    for model in "${MISSING_MODELS[@]}"; do
        echo "${PREFIX}${model}" >> .git/info/sparse-checkout
        FOUND_POTENTIAL=true
    done
    
    if [ "$FOUND_POTENTIAL" = true ]; then
        # Try to pull
        if git pull --depth=1 origin "$BRANCH" >/dev/null 2>&1; then
            # Check what we actually got
            local REPO_ROOT="$TEMP_REPO"
            if [ ! -z "$PREFIX" ]; then
                REPO_ROOT="$TEMP_REPO/$PREFIX"
            fi

            if [ -d "$REPO_ROOT" ]; then
                # Move found models to ~/.gazebo/models
                local NEW_MODELS_FOUND=0
                for model in "${MISSING_MODELS[@]}"; do
                    if [ -d "$REPO_ROOT/$model" ]; then
                        # Verify it's a real model (has model.config or model.sdf)
                        if [ -f "$REPO_ROOT/$model/model.config" ] || [ -f "$REPO_ROOT/$model/model.sdf" ]; then
                            echo "    ✓ Found: $model"
                            mv "$REPO_ROOT/$model" "$MODEL_PATH/"
                            NEW_MODELS_FOUND=$((NEW_MODELS_FOUND + 1))
                        fi
                    fi
                done
                
                # Update MISSING_MODELS list
                local REMAINING_MODELS=()
                for model in "${MISSING_MODELS[@]}"; do
                    if [ ! -d "$MODEL_PATH/$model" ]; then
                        REMAINING_MODELS+=("$model")
                    fi
                done
                MISSING_MODELS=("${REMAINING_MODELS[@]}")
            fi
        fi
    fi
    
    # Cleanup
    cd "$OUTPUT_DIR" || cd /tmp
    rm -rf "$TEMP_REPO"
}

if [ ${#MISSING_MODELS[@]} -gt 0 ]; then
    echo "⚠ Missing models detected: ${#MISSING_MODELS[@]} model(s)"
    echo "Attempting to download missing models..."
    mkdir -p "$MODEL_PATH"
    
    # 1. Try OSRF Gazebo Models (standard)
    download_from_git "https://github.com/osrf/gazebo_models.git" "master" ""
    
    # 2. Try AWS Robomaker (residential) if still missing
    if [ ${#MISSING_MODELS[@]} -gt 0 ]; then
        download_from_git "https://github.com/aws-robotics/aws-robomaker-small-house-world.git" "ros2" "models/"
    fi
    
    # Report results
    if [ ${#MISSING_MODELS[@]} -eq 0 ]; then
        echo "✓ All models successfully downloaded."
    else
        echo "⚠ Could not find sources for: ${MISSING_MODELS[*]}"
        echo "  Check Fuel or generic sources manually."
    fi
else
    echo "✓ All required models are already installed"
fi

echo "================================================"
echo "Automated Gazebo 2D Map Generator"
echo "================================================"
echo "Processing: $(basename "$WORLD_FILE")"
echo "Output dir: $OUTPUT_DIR"
echo "================================================"

echo "[1/5] Starting Gazebo server..."
# Start Gazebo server in background (use temporary file if plugin was injected)
gzserver "$TEMP_WORLD_FILE" > /tmp/gazebo_map_gen.log 2>&1 &
GAZEBO_PID=$!

# Function to cleanup on exit
cleanup() {
    # Forcefully kill Gazebo and all child processes
    if [ ! -z "$GAZEBO_PID" ]; then
        # Kill the entire process group to ensure all child processes are terminated
        pkill -P $GAZEBO_PID 2>/dev/null || true
        kill -9 $GAZEBO_PID 2>/dev/null || true
    fi
    # Also kill any orphaned gzserver processes from this session
    killall -9 gzserver 2>/dev/null || true
    
    # Remove temporary world file if it was created
    if [ "$PLUGIN_INJECTED" = true ] && [ -f "$TEMP_WORLD_FILE" ]; then
        rm -f "$TEMP_WORLD_FILE" 2>/dev/null || true
    fi
    # Remove temporary log file
    rm -f /tmp/gazebo_map_gen.log 2>/dev/null || true
}

trap cleanup EXIT INT TERM

# Wait for Gazebo to start
echo "[2/5] Waiting for Gazebo initialization..."
echo "      (This may take a while if downloading models...)"
sleep 10

# Check if Gazebo started successfully (check for gzserver process, not just the initial PID)
if ! pgrep -f "gzserver.*$TEMP_WORLD_FILE" > /dev/null; then
    echo "ERROR: Gazebo failed to start!"
    echo "Check log: /tmp/gazebo_map_gen.log"
    exit 1
fi

# Wait for ROS topics to be available
echo "[3/5] Waiting for plugin..."
echo "      (Waiting up to 180 seconds for models to download and plugin to load...)"
for i in {1..360}; do
    if ros2 topic list 2>/dev/null | grep -q "/map2d\|/gazebo_2Dmap_plugin"; then
        break
    fi
    sleep 0.5
done

# Trigger map generation by calling the service
echo "[4/5] Generating occupancy map..."
if ros2 service list 2>/dev/null | grep -q "generate_map"; then
    ros2 service call /gazebo_2Dmap_plugin/generate_map std_srvs/srv/Empty >/dev/null 2>&1
    # Wait for map generation to complete
    sleep 0.5
else
    echo "WARNING: generate_map service not found, waiting..."
    sleep 3
fi

# Check if map topic is being published
if ! ros2 topic list 2>/dev/null | grep -q "/map2d"; then
    echo "ERROR: /map2d topic not found!"
    echo "Check log: /tmp/gazebo_map_gen.log"
    exit 1
fi

# Verify map has data 
if ! timeout 3 ros2 topic echo /map2d --once > /dev/null 2>&1; then
    echo "ERROR: Could not read from /map2d topic"
    echo "Check log: /tmp/gazebo_map_gen.log"
    exit 1
fi

# Save the map
echo "[5/5] Saving map files..."
cd "$OUTPUT_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" --ros-args -r map:=map2d --log-level error 2>&1 | grep -v "^$" || true

# Verify files were created
if [ -f "${MAP_NAME}.yaml" ] && [ -f "${MAP_NAME}.pgm" ]; then
    echo ""
    echo "================================================"
    echo "✓ SUCCESS: Map generated automatically"
    echo "================================================"
    echo "World:  $(basename "$WORLD_FILE")"
    echo "Output: $OUTPUT_DIR/$MAP_NAME.yaml"
    echo "        $OUTPUT_DIR/$MAP_NAME.pgm"
    echo "================================================"
    
    # Success - exit cleanly (cleanup trap will run automatically)
    exit 0
else
    echo "================================================"
    echo "ERROR: Map files were not created!"
    echo "Check /tmp/gazebo_map_gen.log for details"
    echo "================================================"
    exit 1
fi
