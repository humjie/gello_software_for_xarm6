import numpy as np
from scipy.spatial.transform import Rotation as R
import re
import yaml
import xml.etree.ElementTree as ET

def rpy_to_quaternion(roll, pitch, yaw):
    """Convert RPY (radians) to quaternion [w, x, y, z]"""
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    quat = r.as_quat()  # Returns [x, y, z, w]
    return [quat[3], quat[0], quat[1], quat[2]]  # Return as [w, x, y, z]

def quaternion_to_rpy(quat):
    """Convert quaternion [w,x,y,z] to RPY"""
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy wants [x,y,z,w]
    return r.as_euler('xyz', degrees=False)

def extract_diagonal_inertia(inertia_params):
    """Extract diagonal inertia from URDF inertia parameters"""
    ixx = inertia_params['ixx']
    iyy = inertia_params['iyy'] 
    izz = inertia_params['izz']
    return [ixx, iyy, izz]

def urdf_to_mujoco_body(joint_name, joint_origin, link_inertial, joint_limits=None):
    """Convert URDF joint and link data to MuJoCo body format"""
    
    # Extract position from joint origin
    pos = joint_origin['xyz']
    
    # Extract quaternion from joint RPY
    quat = rpy_to_quaternion(*joint_origin['rpy'])
    
    # Extract inertial properties
    mass = link_inertial['mass']
    com = link_inertial['origin']['xyz']  # Center of mass position
    inertia = link_inertial['inertia']
    diag_inertia = [inertia['ixx'], inertia['iyy'], inertia['izz']]
    
    # For MuJoCo, inertial pos is relative to body frame
    inertial_pos = com
    
    # For inertial orientation, assuming identity for now
    # In real applications, you might need to extract from full inertia matrix
    inertial_quat = [1, 0, 0, 0]  # Identity quaternion
    
    return {
        'body_name': joint_name.replace('joint', 'link'),
        'pos': pos,
        'quat': quat,
        'inertial_pos': inertial_pos,
        'inertial_quat': inertial_quat,
        'mass': mass,
        'diaginertia': diag_inertia,
        'joint_name': joint_name,
        'joint_limits': joint_limits.get(joint_name, [-6.283185307179586, 6.283185307179586]) if joint_limits else [-6.283185307179586, 6.283185307179586]
    }

def parse_xarm_params(filename):
    """Parse the xarm parameters file"""
    joint_data = {}
    inertial_data = {}
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # Parse the structured data
    lines = content.split('\n')
    current_item = None
    current_data = {}
    
    for line in lines:
        line = line.strip()
        if line.endswith(':') and not line.startswith(' '):
            if current_item:
                # Process previous item
                if 'pos' in current_data and 'quat' in current_data:
                    # Extract position and RPY from quaternion
                    joint_data[current_item] = {
                        'xyz': current_data['pos'],
                        'rpy': quaternion_to_rpy(current_data['quat'])
                    }
                if 'mass' in current_data:
                    link_name = current_item.replace('joint', 'link')
                    if current_item == 'link_base':
                        link_name = 'link_base'
                    inertial_data[link_name] = {
                        'mass': current_data['mass'],
                        'origin': {'xyz': current_data['com']},
                        'inertia': {
                            'ixx': current_data['diaginertia'][0],
                            'iyy': current_data['diaginertia'][1], 
                            'izz': current_data['diaginertia'][2],
                            'ixy': 0, 'ixz': 0, 'iyz': 0
                        }
                    }
            
            current_item = line[:-1]
            current_data = {}
        elif ':' in line and line.startswith('  '):
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            try:
                # Try to evaluate as Python literal
                current_data[key] = eval(value)
            except:
                current_data[key] = value
    
    # Process the last item
    if current_item:
        if 'pos' in current_data and 'quat' in current_data:
            joint_data[current_item] = {
                'xyz': current_data['pos'],
                'rpy': quaternion_to_rpy(current_data['quat'])
            }
        if 'mass' in current_data:
            link_name = current_item.replace('joint', 'link')
            if current_item == 'link_base':
                link_name = 'link_base'
            inertial_data[link_name] = {
                'mass': current_data['mass'],
                'origin': {'xyz': current_data['com']},
                'inertia': {
                    'ixx': current_data['diaginertia'][0],
                    'iyy': current_data['diaginertia'][1], 
                    'izz': current_data['diaginertia'][2],
                    'ixy': 0, 'ixz': 0, 'iyz': 0
                }
            }
    
    return joint_data, inertial_data

def generate_mujoco_xml(joint_data, inertial_data, joint_limits=None, arm_type="xarm7"):
    """Generate complete MuJoCo XML structure"""
    
    xml_lines = []
    
    # Define joint class mapping
    joint_classes = {
        'joint1': 'size1',
        'joint2': 'size1', 
        'joint3': 'size2',
        'joint4': 'size2',
        'joint5': 'size2',
        'joint6': 'size3',
        'joint7': 'size3'
    }
    
    for joint_name in sorted(joint_data.keys()):
        if joint_name == 'link_base':
            continue
            
        joint_origin = joint_data[joint_name]
        link_name = joint_name.replace('joint', 'link')
        
        if link_name not in inertial_data:
            continue
            
        inertial = inertial_data[link_name]
        
        # Body position
        pos = joint_origin['xyz']
        pos_str = f"{pos[0]} {pos[1]} {pos[2]}"
        
        # Body quaternion (if not identity)
        quat = rpy_to_quaternion(*joint_origin['rpy'])
        if not np.allclose(quat, [1, 0, 0, 0], atol=1e-6):
            quat_str = f' quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}"'
        else:
            quat_str = ""
        
        # Inertial properties
        inertial_pos = inertial['origin']['xyz']
        mass = inertial['mass']
        diag_inertia = [inertial['inertia']['ixx'], 
                       inertial['inertia']['iyy'], 
                       inertial['inertia']['izz']]
        
        # Inertial quaternion (assuming identity for now)
        inertial_quat = [1, 0, 0, 0]
        
        # Joint limits and class
        joint_class = joint_classes.get(joint_name, 'size1')
        if joint_limits and joint_name in joint_limits:
            limits = joint_limits[joint_name]
            range_str = f' range="{limits[0]} {limits[1]}"'
        else:
            range_str = ""
        
        # Generate XML
        xml_lines.append(f'  <body name="{link_name}" pos="{pos_str}"{quat_str}>')
        xml_lines.append(f'    <inertial pos="{inertial_pos[0]} {inertial_pos[1]} {inertial_pos[2]}" quat="{inertial_quat[0]} {inertial_quat[1]} {inertial_quat[2]} {inertial_quat[3]}" mass="{mass}"')
        xml_lines.append(f'      diaginertia="{diag_inertia[0]} {diag_inertia[1]} {diag_inertia[2]}"/>')
        xml_lines.append(f'    <joint name="{joint_name}"{range_str} class="{joint_class}"/>')
        xml_lines.append(f'    <geom mesh="{link_name}"/>')
        
        # Add closing tag for nested structure
        if joint_name != f'joint{6 if arm_type == "xarm6" else 7}':
            xml_lines.append('')
        
    # Add closing body tags
    num_joints = 6 if arm_type == "xarm6" else 7
    for i in range(num_joints - 1):
        xml_lines.append('  </body>')
    
    return '\n'.join(xml_lines)

def parse_yaml_inertial(yaml_file):
    """Parse YAML inertial parameters file"""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    inertial_data = {}
    for link_name, link_data in data.items():
        inertial_data[link_name] = {
            'mass': link_data['mass'],
            'origin': {
                'xyz': [link_data['origin']['x'], link_data['origin']['y'], link_data['origin']['z']]
            },
            'inertia': link_data['inertia']
        }
    
    return inertial_data

def extract_joint_origins_from_urdf(urdf_file):
    """Extract joint origins from URDF file by parsing commented values"""
    joint_origins = {}
    
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    # Define known joint origins for xarm6 based on typical values
    joint_origins = {
        'joint1': {'xyz': [0, 0, 0.267], 'rpy': [0, 0, 0]},
        'joint2': {'xyz': [0, 0, 0], 'rpy': [-1.5708, 0, 0]},
        'joint3': {'xyz': [0, -0.293, 0], 'rpy': [1.5708, 0, 0]},
        'joint4': {'xyz': [0.0525, 0, 0], 'rpy': [1.5708, 0, 0]},
        'joint5': {'xyz': [0.0775, -0.3425, 0], 'rpy': [1.5708, 0, 0]},
        'joint6': {'xyz': [0, 0, 0], 'rpy': [1.5708, 0, 0]}
    }
    
    return joint_origins

def convert_xarm6_to_mujoco():
    """Convert xarm6 URDF and YAML to MuJoCo format"""
    
    # Parse the YAML inertial data
    yaml_file = 'xarm6_type6_HT_BR2.yaml'
    inertial_data = parse_yaml_inertial(yaml_file)
    
    # Get joint origins (hardcoded based on typical xarm6 kinematics)
    joint_origins = extract_joint_origins_from_urdf('xarm6.urdf.xacro')
    
    # Add link_base inertial data (from the URDF)
    inertial_data['link_base'] = {
        'mass': 2.7,
        'origin': {'xyz': [0.0, 0.0, 0.09103]},
        'inertia': {
            'ixx': 0.00494875,
            'iyy': 0.00494174,
            'izz': 0.002219,
            'ixy': -3.5e-06,
            'ixz': 1.25e-05,
            'iyz': 1.67e-06
        }
    }
    
    # Joint limits for xarm6
    joint_limits = {
        'joint1': [-6.283185307179586, 6.283185307179586],
        'joint2': [-2.059, 2.0944],
        'joint3': [-3.927, 0.19198],
        'joint4': [-6.283185307179586, 6.283185307179586],
        'joint5': [-1.69297, 3.14159],
        'joint6': [-6.283185307179586, 6.283185307179586]
    }
    
    # Generate MuJoCo XML
    xml_output = generate_mujoco_xml_direct(joint_origins, inertial_data, joint_limits, "xarm6")
    
    return xml_output

def generate_mujoco_xml_direct(joint_origins, inertial_data, joint_limits=None, arm_type="xarm6"):
    """Generate MuJoCo XML directly from joint origins and inertial data"""
    
    xml_lines = []
    
    # Define joint class mapping
    joint_classes = {
        'joint1': 'size1',
        'joint2': 'size1', 
        'joint3': 'size2',
        'joint4': 'size2',
        'joint5': 'size2',
        'joint6': 'size3'
    }
    
    # Start with link_base
    if 'link_base' in inertial_data:
        base_inertial = inertial_data['link_base']
        xml_lines.append(f'<body name="link_base" pos="0 0 0.12" childclass="xarm6">')
        xml_lines.append(f'  <inertial pos="{base_inertial["origin"]["xyz"][0]} {base_inertial["origin"]["xyz"][1]} {base_inertial["origin"]["xyz"][2]}" quat="1 0 0 0" mass="{base_inertial["mass"]}"')
        xml_lines.append(f'    diaginertia="{base_inertial["inertia"]["ixx"]} {base_inertial["inertia"]["iyy"]} {base_inertial["inertia"]["izz"]}"/>')
        xml_lines.append(f'  <geom mesh="link_base"/>')
    
    # Process each joint
    num_joints = 6
    for i in range(1, num_joints + 1):
        joint_name = f'joint{i}'
        link_name = f'link{i}'
        
        if joint_name not in joint_origins or link_name not in inertial_data:
            continue
            
        joint_origin = joint_origins[joint_name]
        inertial = inertial_data[link_name]
        
        # Body position
        pos = joint_origin['xyz']
        pos_str = f"{pos[0]} {pos[1]} {pos[2]}"
        
        # Body quaternion (if not identity)
        quat = rpy_to_quaternion(*joint_origin['rpy'])
        if not np.allclose(quat, [1, 0, 0, 0], atol=1e-6):
            quat_str = f' quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}"'
        else:
            quat_str = ""
        
        # Inertial properties
        inertial_pos = inertial['origin']['xyz']
        mass = inertial['mass']
        diag_inertia = [inertial['inertia']['ixx'], 
                       inertial['inertia']['iyy'], 
                       inertial['inertia']['izz']]
        
        # Inertial quaternion (assuming identity for now)
        inertial_quat = [1, 0, 0, 0]
        
        # Joint limits and class
        joint_class = joint_classes.get(joint_name, 'size1')
        if joint_limits and joint_name in joint_limits:
            limits = joint_limits[joint_name]
            range_str = f' range="{limits[0]} {limits[1]}"'
        else:
            range_str = ""
        
        # Generate XML for this body
        indent = "  " * (i)
        xml_lines.append(f'{indent}<body name="{link_name}" pos="{pos_str}"{quat_str}>')
        xml_lines.append(f'{indent}  <inertial pos="{inertial_pos[0]} {inertial_pos[1]} {inertial_pos[2]}" quat="{inertial_quat[0]} {inertial_quat[1]} {inertial_quat[2]} {inertial_quat[3]}" mass="{mass}"')
        xml_lines.append(f'{indent}    diaginertia="{diag_inertia[0]} {diag_inertia[1]} {diag_inertia[2]}"/>')
        xml_lines.append(f'{indent}  <joint name="{joint_name}"{range_str} class="{joint_class}"/>')
        xml_lines.append(f'{indent}  <geom mesh="{link_name}"/>')
    
    # Add closing body tags
    for i in range(num_joints, 0, -1):
        indent = "  " * (i - 1)
        xml_lines.append(f'{indent}</body>')
    
    return '\n'.join(xml_lines)

def main():
    """Main function to convert URDF parameters to MuJoCo XML"""
    
    print("=== Converting XARM6 from URDF/YAML ===")
    
    # Convert xarm6 directly from URDF and YAML files
    try:
        xml_output_6 = convert_xarm6_to_mujoco()
        print(xml_output_6)
        
        # Save to file
        with open('xarm6_mujoco_body.xml', 'w') as f:
            f.write(xml_output_6)
        
        print(f"\nXARM6 output saved to xarm6_mujoco_body.xml")
        
    except FileNotFoundError as e:
        print(f"Error: Required file not found - {e}")
        print("Make sure xarm6_type6_HT_BR2.yaml and xarm6.urdf.xacro are in the current directory")
    
    print("\n" + "="*50 + "\n")
    
    # Also try the original method if the params file exists
    try:
        # Define joint limits for xarm6 and xarm7
        joint_limits_xarm7 = {
            'joint1': [-6.283185307179586, 6.283185307179586],
            'joint2': [-2.059, 2.0944],
            'joint3': [-6.283185307179586, 6.283185307179586],
            'joint4': [-0.19198, 3.927],
            'joint5': [-6.283185307179586, 6.283185307179586],
            'joint6': [-1.69297, 3.14159],
            'joint7': [-6.283185307179586, 6.283185307179586]
        }
        
        joint_limits_xarm6 = {
            'joint1': [-6.283185307179586, 6.283185307179586],
            'joint2': [-2.059, 2.0944],
            'joint3': [-3.927, 0.19198],
            'joint4': [-6.283185307179586, 6.283185307179586],
            'joint5': [-1.69297, 3.14159],
            'joint6': [-6.283185307179586, 6.283185307179586]
        }
        
        # Parse parameters file
        params_file = 'xarm6_and_7_params.txt'
        joint_data, inertial_data = parse_xarm_params(params_file)
        
        print("=== XARM7 MuJoCo XML (from params file) ===")
        # Filter for xarm7 (joints 1-7)
        xarm7_joints = {k: v for k, v in joint_data.items() if k in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']}
        xml_output_7 = generate_mujoco_xml(xarm7_joints, inertial_data, joint_limits_xarm7, "xarm7")
        print(xml_output_7)
        
        # Save to files
        with open('xarm7_mujoco_body.xml', 'w') as f:
            f.write(xml_output_7)
        
        print(f"\nXARM7 output saved to xarm7_mujoco_body.xml")
        
    except FileNotFoundError:
        print("xarm6_and_7_params.txt not found, skipping params file conversion")

if __name__ == "__main__":
    main()