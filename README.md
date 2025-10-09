# Gazebo ROS 2 2D Map Plugin

**Automatically generate 2D occupancy maps from Gazebo Classic worlds for ROS 2 Humble.**

This plugin generates occupancy maps by slicing the Gazebo world at a specified height and performing wavefront exploration from an initial position. Perfect for creating navigation maps without running actual SLAM.

## Features

✅ **Automatic map generation** from any Gazebo world  
✅ **Auto-detection** of world bounds (no manual sizing needed)  
✅ **Origin-aligned** maps (centered at world origin 0,0)  
✅ **Fully automated** script (plugin injection, generation, cleanup)  
✅ **Subprocess-ready** for Python integration  
✅ **ROS 2 Humble** compatible with Gazebo 11 Classic

## Quick Start

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Miguesji/gazebo_ros2_2Dmap_plugin.git
cd ~/ros2_ws
colcon build --packages-select gazebo_ros2_2dmap_plugin
source install/setup.bash
```

### Automated Generation (Recommended)

The easiest way - automatically injects plugin, generates map, and cleans up:

```bash
# Basic usage
ros2 run gazebo_ros2_2dmap_plugin generate_map.sh <world_file> [output_dir]

# Example
ros2 run gazebo_ros2_2dmap_plugin generate_map.sh ~/my_world.sdf ~/maps
```

**Output:** Creates `my_world.yaml` and `my_world.pgm` (automatically named from world file)

**What it does:**

1. Detects if plugin is in world file (injects if missing)
2. Launches Gazebo in headless mode
3. Generates occupancy map
4. Saves map files
5. Cleans up automatically

### Manual Method

If you prefer manual control:

**1. Add plugin to your world file** (inside `<world>` tags):

```xml
<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
    <map_resolution>0.05</map_resolution>  <!-- 5cm per cell -->
    <map_height>0.3</map_height>           <!-- slice at 30cm -->
    <!-- map_size_x/y omitted for auto-detection -->
    <init_robot_x>0</init_robot_x>         <!-- start from origin -->
    <init_robot_y>0</init_robot_y>
</plugin>
```

**2. Launch Gazebo:**

```bash
gazebo my_world.sdf
```

**3. Generate map:**

```bash
ros2 service call /gazebo_2Dmap_plugin/generate_map std_srvs/srv/Empty
```

**4. Save map:**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -r map:=map2d
```

## Configuration

### Plugin Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_resolution` | 0.05 | Cell size in meters |
| `map_height` | 0.2 | Height to slice world (0.2-0.5m typical) |
| `map_size_x` | auto | Map width - **omit for auto-detection** |
| `map_size_y` | auto | Map height - **omit for auto-detection** |
| `map_margin` | 2.0 | Extra space around auto-detected bounds |
| `init_robot_x` | 0.0 | Start position for wavefront (must be free space!) |
| `init_robot_y` | 0.0 | Start position for wavefront |

### Auto-Detection

When you omit `map_size_x` and `map_size_y`:

- Plugin analyzes all models in the world
- Computes bounding box of all objects
- Centers map at world origin (0,0)
- Sizes map to fit all objects + margin

## Python Integration

Recommended for automated workflows:

```python
import subprocess
import os

def generate_map(world_file, output_dir="./maps"):
    """Generate map automatically."""
    os.makedirs(output_dir, exist_ok=True)
    
    result = subprocess.run([
        'ros2', 'run', 'gazebo_ros2_2dmap_plugin', 'generate_map.sh',
        world_file, output_dir
    ], capture_output=True, text=True, timeout=60)
    
    if result.returncode == 0:
        map_name = os.path.splitext(os.path.basename(world_file))[0]
        return {
            'yaml': f"{output_dir}/{map_name}.yaml",
            'pgm': f"{output_dir}/{map_name}.pgm"
        }
    raise RuntimeError(result.stderr)

# Usage
files = generate_map("/path/to/world.sdf", "./maps")
print(f"Generated: {files['yaml']}")
```

**Batch processing:**

```bash
for world in ~/worlds/*.sdf; do
    ros2 run gazebo_ros2_2dmap_plugin generate_map.sh "$world" ~/maps
done
```

## Visualization

View your map in RViz2:

```bash
ros2 run rviz2 rviz2
```

1. Add → By topic → `/map2d` → Map
2. Set Fixed Frame to `odom`
3. View your map!

**Colors:**

- **White** = Free space
- **Black** = Occupied (walls/obstacles)
- **Gray** = Unknown (not explored)

## Troubleshooting

**Map is all gray/unknown**

- Ensure `init_robot_x/y` is in free space (not inside a wall)
- Check that wavefront can reach all areas

**Map doesn't fit all objects**

- Increase `map_margin` parameter (try 3.0 or 5.0)
- Check auto-detection logs for computed bounds

**Walls too thick/thin**

- Adjust `map_height` (try 0.2-0.5m range)
- Verify collision geometries exist in models

**Script hangs/doesn't exit**

- The script should exit automatically
- Check `/tmp/gazebo_map_gen.log` for errors
- Ensure ROS 2 workspace is sourced

**Plugin not found**

- Verify build: `ls install/gazebo_ros2_2dmap_plugin/lib/libgazebo_2Dmap_plugin.so`
- Check plugin path: `echo $GAZEBO_PLUGIN_PATH`

## ROS 2 API

### Topics

- `/map2d` (`nav_msgs/msg/OccupancyGrid`) - Generated map (transient local QoS)

### Services  

- `/gazebo_2Dmap_plugin/generate_map` (`std_srvs/srv/Empty`) - Trigger generation

## Requirements

- ROS 2 Humble
- Gazebo 11 (Classic)
- `nav2_map_server` (for saving maps): `sudo apt install ros-humble-nav2-map-server`

## Notes

- **Remove robots** from world before generating map (they appear as obstacles)
- **Wavefront exploration** starts from `init_robot_x/y` - must be in free space
- **Frame ID** is set to `odom` (modify source if needed)
- **Map origin** is always at world origin (0,0) when using auto-detection

## License

MIT License

---

**Migrated from ROS 1 to ROS 2 Humble** | Based on [octomap plugin](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_gazebo_plugins) by ETH Zürich
