import os
from generate_pillar import generate_pillar_sdf

def generate_square_pillars(output_folder="sdf_pillars"):
    # Define the coordinates for the 4 pillars in a square
    pillar_coordinates = [
        (1.0, 1.0),   # Top-right
        (-1.0, 1.0),  # Top-left
        (-1.0, -1.0), # Bottom-left
        (1.0, -1.0)   # Bottom-right
    ]

    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Generate SDF files for each pillar
    for i, (x, y) in enumerate(pillar_coordinates, start=1):
        output_file = os.path.join(output_folder, f"pillar_{i}.sdf")
        generate_pillar_sdf(x, y, output_file)
        print(f"Generated {output_file} at ({x}, {y})")

if __name__ == "__main__":
    generate_square_pillars()