import json

def generate_urdf(config_file, output_file="robot.urdf"):
    # Load configuration
    with open(config_file, "r") as f:
        config = json.load(f)

    platform = config["platform"]
    dimensions = platform["dimensions"]
    mass = platform["mass"]
    attachment_points = platform["attachment_points"]

    # URDF template
    urdf_template = f"""<?xml version="1.0"?>
<robot name="cable_robot_platform">
  <!-- Base Link: World -->
  <link name="world"/>

  <!-- Platform Link -->
  <link name="platform">
    <!-- Visual Properties -->
    <visual>
      <geometry>
        <box size="{dimensions[0]} {dimensions[1]} {dimensions[2]}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <!-- Collision Properties -->
    <collision>
      <geometry>
        <box size="{dimensions[0]} {dimensions[1]} {dimensions[2]}"/>
      </geometry>
    </collision>
    <!-- Inertial Properties -->
    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Floating Joint to Spawn in Gazebo -->
  <joint name="floating_joint" type="floating">
    <parent link="world"/>
    <child link="platform"/>
  </joint>

  <!-- Attachment Points -->
"""

    # Add attachment points
    for i, point in enumerate(attachment_points, start=1):
        urdf_template += f"""
  <!-- Attachment Point {i} -->
  <link name="attachment_point_{i}">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="attachment_joint_{i}" type="fixed">
    <parent link="platform"/>
    <child link="attachment_point_{i}"/>
    <origin xyz="{point[0]} {point[1]} {point[2]}" rpy="0 0 0"/>
  </joint>
"""

    urdf_template += "</robot>"

    # Save URDF to file
    with open(output_file, "w") as f:
        f.write(urdf_template)

    print(f"URDF file generated: {output_file}")

# Run the generator
generate_urdf("robot_config.json")