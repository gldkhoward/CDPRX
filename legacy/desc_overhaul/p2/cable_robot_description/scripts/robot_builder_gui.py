import tkinter as tk
from tkinter import messagebox
import json

def generate_urdf():
    # Get user inputs
    platform_dimensions = [float(entry_length.get()), float(entry_width.get()), float(entry_height.get())]
    platform_mass = float(entry_mass.get())
    attachment_points = []

    # Parse attachment points
    for point in entry_attachment_points.get().split(";"):
        x, y, z = map(float, point.strip().split(","))
        attachment_points.append([x, y, z])

    # Save configuration to a JSON file
    config = {
        "platform": {
            "dimensions": platform_dimensions,
            "mass": platform_mass,
            "attachment_points": attachment_points
        }
    }

    with open("robot_config.json", "w") as f:
        json.dump(config, f, indent=4)

    messagebox.showinfo("Success", "Configuration saved to robot_config.json. Run the URDF generator script.")

# Create the GUI
root = tk.Tk()
root.title("Robot Builder")

# Platform dimensions
tk.Label(root, text="Platform Dimensions (x, y, z):").grid(row=0, column=0)
entry_length = tk.Entry(root)
entry_length.grid(row=0, column=1)
entry_width = tk.Entry(root)
entry_width.grid(row=0, column=2)
entry_height = tk.Entry(root)
entry_height.grid(row=0, column=3)

# Platform mass
tk.Label(root, text="Platform Mass (kg):").grid(row=1, column=0)
entry_mass = tk.Entry(root)
entry_mass.grid(row=1, column=1)

# Attachment points
tk.Label(root, text="Attachment Points (x,y,z; x,y,z; ...):").grid(row=2, column=0)
entry_attachment_points = tk.Entry(root, width=50)
entry_attachment_points.grid(row=2, column=1, columnspan=3)

# Generate URDF button
btn_generate = tk.Button(root, text="Generate URDF Configuration", command=generate_urdf)
btn_generate.grid(row=3, column=0, columnspan=4)

root.mainloop()