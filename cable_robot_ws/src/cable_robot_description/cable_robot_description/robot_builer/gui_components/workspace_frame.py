import tkinter as tk
from tkinter import ttk, messagebox

class WorkspaceFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.create_widgets()

    def create_widgets(self):
        # Position Limits
        pos_frame = ttk.LabelFrame(self, text="Position Limits")
        pos_frame.pack(fill='x', padx=5, pady=5)

        self.pos_entries = {}
        positions = [
            ("X", "min_x", "max_x", -1.0, 1.0),
            ("Y", "min_y", "max_y", -1.0, 1.0),
            ("Z", "min_z", "max_z", 0.0, 2.0)
        ]

        for i, (axis, min_key, max_key, min_default, max_default) in enumerate(positions):
            # Create frame for each axis
            axis_frame = ttk.Frame(pos_frame)
            axis_frame.pack(fill='x', padx=5, pady=2)

            # Axis label
            ttk.Label(axis_frame, text=f"{axis} (meters):").pack(side='left', padx=5)
            
            # Min entry
            ttk.Label(axis_frame, text="Min:").pack(side='left', padx=(20,5))
            min_entry = ttk.Entry(axis_frame, width=10)
            min_entry.pack(side='left', padx=5)
            min_entry.insert(0, str(min_default))
            self.pos_entries[min_key] = min_entry

            # Max entry
            ttk.Label(axis_frame, text="Max:").pack(side='left', padx=(20,5))
            max_entry = ttk.Entry(axis_frame, width=10)
            max_entry.pack(side='left', padx=5)
            max_entry.insert(0, str(max_default))
            self.pos_entries[max_key] = max_entry

        # Orientation Limits
        orient_frame = ttk.LabelFrame(self, text="Orientation Limits (degrees)")
        orient_frame.pack(fill='x', padx=5, pady=5)

        self.orient_entries = {}
        orientations = [
            ("Roll", "min_roll", "max_roll", -45.0, 45.0),
            ("Pitch", "min_pitch", "max_pitch", -45.0, 45.0),
            ("Yaw", "min_yaw", "max_yaw", -90.0, 90.0)
        ]

        for i, (rot, min_key, max_key, min_default, max_default) in enumerate(orientations):
            # Create frame for each rotation
            rot_frame = ttk.Frame(orient_frame)
            rot_frame.pack(fill='x', padx=5, pady=2)

            # Rotation label
            ttk.Label(rot_frame, text=f"{rot}:").pack(side='left', padx=5)
            
            # Min entry
            ttk.Label(rot_frame, text="Min:").pack(side='left', padx=(20,5))
            min_entry = ttk.Entry(rot_frame, width=10)
            min_entry.pack(side='left', padx=5)
            min_entry.insert(0, str(min_default))
            self.orient_entries[min_key] = min_entry

            # Max entry
            ttk.Label(rot_frame, text="Max:").pack(side='left', padx=(20,5))
            max_entry = ttk.Entry(rot_frame, width=10)
            max_entry.pack(side='left', padx=5)
            max_entry.insert(0, str(max_default))
            self.orient_entries[max_key] = max_entry

        # Add validation button
        ttk.Button(self, text="Validate Workspace", 
                  command=self.validate_workspace).pack(pady=10)

        # Status label
        self.status_label = ttk.Label(self, text="")
        self.status_label.pack(pady=5)

    def validate_workspace(self):
        """Validate that min values are less than max values"""
        try:
            # Check position limits
            for axis in ['x', 'y', 'z']:
                min_val = float(self.pos_entries[f'min_{axis}'].get())
                max_val = float(self.pos_entries[f'max_{axis}'].get())
                if min_val >= max_val:
                    raise ValueError(f"{axis.upper()} min value must be less than max value")

            # Check orientation limits
            for rot in ['roll', 'pitch', 'yaw']:
                min_val = float(self.orient_entries[f'min_{rot}'].get())
                max_val = float(self.orient_entries[f'max_{rot}'].get())
                if min_val >= max_val:
                    raise ValueError(f"{rot.capitalize()} min value must be less than max value")

            self.status_label.config(text="✓ Workspace configuration is valid", foreground="green")
        except ValueError as e:
            self.status_label.config(text=f"✗ {str(e)}", foreground="red")
            messagebox.showerror("Validation Error", str(e))

    def get_config(self):
        try:
            config = {}
            
            # Get position limits
            for key in self.pos_entries:
                config[key] = float(self.pos_entries[key].get())
                
            # Get orientation limits
            for key in self.orient_entries:
                config[key] = float(self.orient_entries[key].get())
                
            return config
            
        except ValueError as e:
            messagebox.showerror("Error", "Please ensure all values are valid numbers")
            raise

    def set_config(self, config):
        if not config:
            return
            
        # Set position limits
        for key in self.pos_entries:
            if key in config:
                self.pos_entries[key].delete(0, tk.END)
                self.pos_entries[key].insert(0, str(config[key]))
                
        # Set orientation limits
        for key in self.orient_entries:
            if key in config:
                self.orient_entries[key].delete(0, tk.END)
                self.orient_entries[key].insert(0, str(config[key]))
        
        # Validate the workspace after loading
        self.validate_workspace()