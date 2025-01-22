import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np

class PlatformFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.create_widgets()

    def create_widgets(self):
        # Create main scroll canvas
        canvas = tk.Canvas(self)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Basic Properties
        basic_frame = ttk.LabelFrame(self.scrollable_frame, text="Basic Properties")
        basic_frame.pack(fill='x', padx=5, pady=5)

        # Mass
        mass_frame = ttk.Frame(basic_frame)
        mass_frame.pack(fill='x', padx=5, pady=2)
        ttk.Label(mass_frame, text="Mass (kg):").pack(side='left', padx=5)
        self.mass_entry = ttk.Entry(mass_frame, width=15)
        self.mass_entry.pack(side='left', padx=5)
        self.mass_entry.insert(0, "1.0")  # Default value

        # Dimensions
        dim_frame = ttk.LabelFrame(basic_frame, text="Dimensions")
        dim_frame.pack(fill='x', padx=5, pady=2)
        
        self.dim_entries = {}
        dimensions = [
            ("Length (m)", "length"),
            ("Width (m)", "width"),
            ("Height (m)", "height")
        ]
        
        for i, (label, key) in enumerate(dimensions):
            ttk.Label(dim_frame, text=label).grid(row=i, column=0, padx=5, pady=2, sticky='w')
            entry = ttk.Entry(dim_frame, width=15)
            entry.grid(row=i, column=1, padx=5, pady=2)
            entry.insert(0, "0.5" if key != "height" else "0.1")  # Default values
            self.dim_entries[key] = entry

        # Center of Mass Offset
        com_frame = ttk.LabelFrame(self.scrollable_frame, text="Center of Mass Offset")
        com_frame.pack(fill='x', padx=5, pady=5)
        
        self.com_entries = {}
        for i, axis in enumerate(['x', 'y', 'z']):
            ttk.Label(com_frame, text=f"{axis} (m):").grid(row=i, column=0, padx=5, pady=2, sticky='w')
            entry = ttk.Entry(com_frame, width=15)
            entry.grid(row=i, column=1, padx=5, pady=2)
            entry.insert(0, "0.0")  # Default value
            self.com_entries[axis] = entry

        # Inertia Matrix
        inertia_frame = ttk.LabelFrame(self.scrollable_frame, text="Inertia Matrix")
        inertia_frame.pack(fill='x', padx=5, pady=5)
        
        self.inertia_entries = []
        for i in range(3):
            row_entries = []
            for j in range(3):
                entry = ttk.Entry(inertia_frame, width=15)
                entry.grid(row=i, column=j, padx=2, pady=2)
                # Default values: moment of inertia on diagonal
                default_value = "0.1" if i == j else "0.0"
                entry.insert(0, default_value)
                row_entries.append(entry)
            self.inertia_entries.append(row_entries)

        # Help text for inertia matrix
        ttk.Label(inertia_frame, text="Values in kg⋅m²", 
                 font=('TkDefaultFont', 8)).grid(row=3, column=0, columnspan=3, pady=(0,5))

        # Attachment Points
        att_frame = ttk.LabelFrame(self.scrollable_frame, text="Attachment Points")
        att_frame.pack(fill='x', padx=5, pady=5)
        
        # Add help text
        help_text = "Enter points as: x1,y1,z1; x2,y2,z2; ...\nExample: 0.2,0.2,0.05; -0.2,0.2,0.05; 0.2,-0.2,0.05"
        ttk.Label(att_frame, text=help_text, 
                 font=('TkDefaultFont', 8)).pack(fill='x', padx=5, pady=(2,0))
        
        self.attachment_entry = ttk.Entry(att_frame)
        self.attachment_entry.pack(fill='x', padx=5, pady=5)
        self.attachment_entry.insert(0, "0.25,0.25,0.05; -0.25,0.25,0.05; 0.25,-0.25,0.05; -0.25,-0.25,0.05")

        # Button to calculate inertia matrix from dimensions and mass
        ttk.Button(self.scrollable_frame, 
                  text="Calculate Inertia from Dimensions", 
                  command=self.calculate_inertia).pack(pady=5)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def calculate_inertia(self):
        try:
            mass = float(self.mass_entry.get())
            length = float(self.dim_entries['length'].get())
            width = float(self.dim_entries['width'].get())
            height = float(self.dim_entries['height'].get())

            # Calculate inertia for a rectangular prism
            Ixx = (mass/12) * (width**2 + height**2)
            Iyy = (mass/12) * (length**2 + height**2)
            Izz = (mass/12) * (length**2 + width**2)

            # Update inertia matrix entries
            inertia_values = [
                [Ixx, 0, 0],
                [0, Iyy, 0],
                [0, 0, Izz]
            ]

            for i in range(3):
                for j in range(3):
                    self.inertia_entries[i][j].delete(0, tk.END)
                    self.inertia_entries[i][j].insert(0, f"{inertia_values[i][j]:.6f}")

        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values for mass and dimensions")

    def get_config(self):
        try:
            # Parse attachment points
            attachment_points = []
            points_str = self.attachment_entry.get().strip()
            if points_str:
                for point in points_str.split(";"):
                    x, y, z = map(float, point.strip().split(","))
                    attachment_points.append([x, y, z])

            # Get inertia matrix
            inertia_matrix = []
            for i in range(3):
                row = []
                for j in range(3):
                    row.append(float(self.inertia_entries[i][j].get()))
                inertia_matrix.append(row)

            return {
                'mass': float(self.mass_entry.get()),
                'dimensions': [
                    float(self.dim_entries[dim].get())
                    for dim in ['length', 'width', 'height']
                ],
                'com_offset': [
                    float(self.com_entries[axis].get())
                    for axis in ['x', 'y', 'z']
                ],
                'inertia': inertia_matrix,
                'attachment_points': attachment_points
            }
            
        except ValueError as e:
            messagebox.showerror("Error", "Please ensure all values are valid numbers")
            raise

    def set_config(self, config):
        if not config:
            return
            
        # Set mass
        self.mass_entry.delete(0, tk.END)
        self.mass_entry.insert(0, str(config['mass']))

        # Set dimensions
        for i, dim in enumerate(['length', 'width', 'height']):
            self.dim_entries[dim].delete(0, tk.END)
            self.dim_entries[dim].insert(0, str(config['dimensions'][i]))

        # Set COM offset
        for i, axis in enumerate(['x', 'y', 'z']):
            self.com_entries[axis].delete(0, tk.END)
            self.com_entries[axis].insert(0, str(config['com_offset'][i]))

        # Set inertia matrix
        for i in range(3):
            for j in range(3):
                self.inertia_entries[i][j].delete(0, tk.END)
                self.inertia_entries[i][j].insert(0, str(config['inertia'][i][j]))

        # Set attachment points
        points_str = "; ".join([",".join(map(str, point)) 
                              for point in config['attachment_points']])
        self.attachment_entry.delete(0, tk.END)
        self.attachment_entry.insert(0, points_str)