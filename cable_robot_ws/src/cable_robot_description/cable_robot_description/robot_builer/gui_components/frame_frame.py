import tkinter as tk
from tkinter import ttk, messagebox

class FrameFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.pillar_entries = []
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

        # Number of pillars control
        pillar_control = ttk.Frame(self.scrollable_frame)
        pillar_control.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(pillar_control, text="Number of Pillars:").pack(side='left', padx=5)
        self.num_pillars_var = tk.StringVar(value="4")
        self.num_pillars_entry = ttk.Entry(pillar_control, 
                                         textvariable=self.num_pillars_var,
                                         width=10)
        self.num_pillars_entry.pack(side='left', padx=5)
        
        ttk.Button(pillar_control, text="Update Pillars", 
                  command=self.update_pillar_entries).pack(side='left', padx=5)

        # Frame dimensions
        frame_dim = ttk.LabelFrame(self.scrollable_frame, text="Frame Dimensions")
        frame_dim.pack(fill='x', padx=5, pady=5)

        self.frame_dim_entries = {}
        dimensions = [
            ("Width (m)", "width"),
            ("Length (m)", "length"),
            ("Height (m)", "height")
        ]

        for i, (label, key) in enumerate(dimensions):
            ttk.Label(frame_dim, text=label).grid(row=i, column=0, padx=5, pady=2, sticky='w')
            entry = ttk.Entry(frame_dim, width=15)
            entry.grid(row=i, column=1, padx=5, pady=2)
            entry.insert(0, "2.0")  # Default value
            self.frame_dim_entries[key] = entry

        # Pillars container
        self.pillars_frame = ttk.LabelFrame(self.scrollable_frame, text="Pillars")
        self.pillars_frame.pack(fill='x', padx=5, pady=5)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Initialize default pillars
        self.update_pillar_entries()

    def create_pillar_entry(self, index):
        frame = ttk.LabelFrame(self.pillars_frame, text=f"Pillar {index + 1}")
        frame.pack(fill='x', padx=5, pady=5)

        entries = {}
        
        # Mass entry
        mass_frame = ttk.Frame(frame)
        mass_frame.pack(fill='x', padx=5, pady=2)
        ttk.Label(mass_frame, text="Mass (kg):").pack(side='left', padx=5)
        mass_entry = ttk.Entry(mass_frame, width=15)
        mass_entry.pack(side='left', padx=5)
        mass_entry.insert(0, "50.0")  # Default value
        entries['mass'] = mass_entry

        # Dimensions
        dim_frame = ttk.LabelFrame(frame, text="Dimensions")
        dim_frame.pack(fill='x', padx=5, pady=2)
        
        dim_entries = {}
        dimensions = [
            ("Width (m)", "width"),
            ("Length (m)", "length"),
            ("Height (m)", "height")
        ]
        
        for i, (label, key) in enumerate(dimensions):
            ttk.Label(dim_frame, text=label).grid(row=i, column=0, padx=5, pady=2, sticky='w')
            entry = ttk.Entry(dim_frame, width=15)
            entry.grid(row=i, column=1, padx=5, pady=2)
            entry.insert(0, "0.2" if key != "height" else "2.0")  # Default values
            dim_entries[key] = entry
        entries['dimensions'] = dim_entries

        # Attachment points
        att_frame = ttk.LabelFrame(frame, text="Attachment Points")
        att_frame.pack(fill='x', padx=5, pady=2)
        
        ttk.Label(att_frame, text="Format: x,y,z; x,y,z; ...").pack(fill='x', padx=5)
        att_entry = ttk.Entry(att_frame)
        att_entry.pack(fill='x', padx=5, pady=2)
        att_entry.insert(0, "0.1,0.1,1.8; -0.1,0.1,1.8")  # Default value
        entries['attachment_points'] = att_entry

        return entries

    def update_pillar_entries(self):
        # Clear existing entries
        for widget in self.pillars_frame.winfo_children():
            widget.destroy()
        self.pillar_entries.clear()

        try:
            num_pillars = int(self.num_pillars_var.get())
            for i in range(num_pillars):
                entries = self.create_pillar_entry(i)
                self.pillar_entries.append(entries)
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number of pillars")

    def get_config(self):
        pillars_config = []
        
        for entries in self.pillar_entries:
            # Parse attachment points
            attachment_points = []
            points_str = entries['attachment_points'].get().strip()
            if points_str:
                for point in points_str.split(";"):
                    x, y, z = map(float, point.strip().split(","))
                    attachment_points.append([x, y, z])

            pillar_config = {
                'mass': float(entries['mass'].get()),
                'dimensions': [
                    float(entries['dimensions'][dim].get())
                    for dim in ['width', 'length', 'height']
                ],
                'attachment_points': attachment_points
            }
            pillars_config.append(pillar_config)

        return {
            'dimensions': [
                float(self.frame_dim_entries[dim].get())
                for dim in ['width', 'length', 'height']
            ],
            'pillars': pillars_config
        }

    def set_config(self, config):
        if not config:
            return
            
        # Set frame dimensions
        for key, value in zip(['width', 'length', 'height'], config['dimensions']):
            self.frame_dim_entries[key].delete(0, tk.END)
            self.frame_dim_entries[key].insert(0, str(value))

        # Update number of pillars
        self.num_pillars_var.set(str(len(config['pillars'])))
        self.update_pillar_entries()

        # Set pillar values
        for pillar_config, entries in zip(config['pillars'], self.pillar_entries):
            entries['mass'].delete(0, tk.END)
            entries['mass'].insert(0, str(pillar_config['mass']))

            # Set dimensions
            for i, dim in enumerate(['width', 'length', 'height']):
                entries['dimensions'][dim].delete(0, tk.END)
                entries['dimensions'][dim].insert(0, str(pillar_config['dimensions'][i]))

            # Set attachment points
            points_str = "; ".join([",".join(map(str, point)) 
                                  for point in pillar_config['attachment_points']])
            entries['attachment_points'].delete(0, tk.END)
            entries['attachment_points'].insert(0, points_str)