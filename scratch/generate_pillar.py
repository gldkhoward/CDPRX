def generate_pillar_sdf_string(x, y):
    sdf_template = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="pillar">
    <static>true</static>  <!-- Make the pillar static -->
    <link name="pillar_link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 2.0</size>  <!-- Dimensions of the pillar -->
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>  <!-- Gray color -->
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 2.0</size>  <!-- Collision geometry -->
          </box>
        </geometry>
      </collision>
    </link>
    <pose>{x} {y} 1.0 0 0 0</pose>  <!-- Position of the pillar -->
  </model>
</sdf>'''
    return sdf_template.format(x=x, y=y)

def generate_pillar_sdf(x, y, output_file):
    sdf_content = generate_pillar_sdf_string(x, y)
    with open(output_file, 'w') as f:
        f.write(sdf_content)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Generate static pillar SDF with specified base center coordinates')
    parser.add_argument('--x', type=float, required=True, help='X coordinate of the base center')
    parser.add_argument('--y', type=float, required=True, help='Y coordinate of the base center')
    parser.add_argument('--output', type=str, default='pillar.sdf', help='Output SDF file name')
    
    args = parser.parse_args()
    generate_pillar_sdf(args.x, args.y, args.output)
    print(f"SDF generated successfully at {args.output}")