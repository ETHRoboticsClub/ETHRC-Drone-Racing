import xml.etree.ElementTree as ET
import numpy as np
import os
import re

MODEL_FILE = 'px4/PX4-Autopilot/Tools/simulation/gz/models/aruco_gate/model.sdf'
WORLD_FILE = 'px4/PX4-Autopilot/Tools/simulation/gz/worlds/aruco_gate_course.sdf'
OUTPUT_FILE = 'src/perception/ArUco_estimation/config/marker_map_aruco_gate_course.yaml'

# Default marker size
DEFAULT_MARKER_SIZE = 0.3

def get_rotation_matrix(roll, pitch, yaw):

    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    # R = Rz * Ry * Rx
    return Rz @ Ry @ Rx

def get_transform_matrix(x, y, z, roll, pitch, yaw):

    T = np.eye(4)
    T[:3, :3] = get_rotation_matrix(roll, pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def rotation_matrix_to_euler(R):

    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def parse_pose_string(pose_str):
    """Parses an SDF pose into a 4x4 matrix."""

    if not pose_str:
        return np.eye(4)
    vals = [float(x) for x in pose_str.split()]

    if len(vals) == 3:
        vals.extend([0, 0, 0])
    return get_transform_matrix(*vals)

def parse_model_markers(model_path):

    tree = ET.parse(model_path)
    root = tree.getroot()
    
    markers = []
    
    for visual in root.findall(".//visual"):
        name = visual.get('name', '')
        
        if 'marker' in name.lower():
            
            # Get Pose
            pose_elem = visual.find('pose')
            pose_str = pose_elem.text if pose_elem is not None else "0 0 0 0 0 0"
            local_tf = parse_pose_string(pose_str)
            
            # Get ID from texture path
            tex_path = ""
            albedo = visual.find(".//pbr/metal/albedo_map")
            if albedo is not None:
                tex_path = albedo.text
            else:
                print("Cannot extract ID")
            
            # Extract ID from filename (e.g. "aruco_marker_0.png" -> 0)
            match = re.search(r'marker_(\d+)\.', tex_path)
            marker_id = int(match.group(1)) if match else 0
            
            # Get marker size
            size_elem = visual.find(".//geometry/plane/size")
            size = DEFAULT_MARKER_SIZE
            if size_elem is not None:
                dims = [float(x) for x in size_elem.text.split()]
                size = dims[0]

            markers.append({
                'id': marker_id,
                'local_pose': local_tf,
                'size': size,
                'name': name
            })
            
    return markers

def parse_world_gates(world_path):

    tree = ET.parse(world_path)
    root = tree.getroot()
    
    gates = []
    
    # Find all <include> tags
    for include in root.findall(".//include"):
        uri = include.find('uri')
        if uri is not None and 'aruco_gate' in uri.text:
            name_elem = include.find('name')
            name = name_elem.text if name_elem is not None else "unknown_gate"
            
            pose_elem = include.find('pose')
            pose_str = pose_elem.text if pose_elem is not None else "0 0 0 0 0 0"
            world_tf = parse_pose_string(pose_str)
            
            gates.append({
                'name': name,
                'world_pose': world_tf
            })
            
    return gates


def generate_yaml(model_file, world_file):
    if not os.path.exists(model_file) or not os.path.exists(world_file):
        print("Error: Files not found.")
        return

    # Get local marker definitions
    model_markers = parse_model_markers(model_file)
    print(f"Found {len(model_markers)} markers in model definition.")

    # Get world gates
    world_gates = parse_world_gates(world_file)
    print(f"Found {len(world_gates)} gates in world definition.")

    final_markers = []
    
    for gate in world_gates:
        gate_name = gate['name']
        gate_tf = gate['world_pose']
        
        for marker in model_markers:
            # Transform: World_T_Marker = World_T_Gate * Gate_T_Marker
            local_tf = marker['local_pose']
            global_tf = gate_tf @ local_tf
            pos = global_tf[:3, 3]
            
            # Extract Rotation (Euler)
            rot_matrix = global_tf[:3, :3]
            euler = rotation_matrix_to_euler(rot_matrix)
            
            final_markers.append({
                'id': marker['id'],
                'size': marker['size'],
                'pos': pos,
                'rot': euler,
                'desc': f"Marker {marker['id']} on {gate_name} ({marker['name']})"
            })

    # 4. Generate YAML
    yaml_content = "%YAML:1.0\n---\n"
    yaml_content += "# Generated ArUco Map\n"
    yaml_content += "marker_dict: \"DICT_5X5_1000\"\n"
    yaml_content += "reference_frame: \"world\"\n"
    yaml_content += f"marker_size: \"{DEFAULT_MARKER_SIZE}\"\n\n"
    yaml_content += "markers:\n"

    for m in final_markers:
        p = np.round(m['pos'], 4)
        r = np.round(m['rot'], 4)
        
        yaml_content += f"  - id: {m['id']}\n"
        yaml_content += f"    size: {m['size']}\n"
        yaml_content += "    pose:\n"
        yaml_content += f"      position: [{p[0]}, {p[1]}, {p[2]}]\n"
        yaml_content += f"      orientation: [{r[0]}, {r[1]}, {r[2]}]\n"
        yaml_content += f"    description: \"{m['desc']}\"\n\n"

    return yaml_content


if __name__ == "__main__":
    result = generate_yaml(MODEL_FILE, WORLD_FILE)
    
    if result:
        with open(OUTPUT_FILE, "w") as f:
            f.write(result)
        print(f"Successfully generated {OUTPUT_FILE}")
        print(result[:500] + "\n...")