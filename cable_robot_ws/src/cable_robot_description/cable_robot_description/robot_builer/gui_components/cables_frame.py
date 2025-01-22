import tkinter as tk
from tkinter import ttk

class CablesFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.cable_entries = []
        self.create_widgets()

    def create_widgets(self):
        # Number of cables input
        num_cables_frame = ttk.Frame(self)
        num_cables_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(num_cables_frame, text="Number of Cables:").pack(side='left', padx=5)
        self.num_cables_entry = ttk.Entry(num_cables_frame, width=10)
        self.num_cables_entry.pack(side='left', padx=5)
        ttk.Button(num_cables_frame, text="Update", 
                  command=self.update_cable_entries).pack(side='left', padx=5)

        # Scrollable frame for cable parameters
        self.canvas = tk.Canvas(self)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        scrollbar.pack(side="right", fill="y")

        # Set default number of cables
        self.num_cables_entry.insert(0, "8")
        self.update_cable_entries()

    def create_cable_entry(self, index):
        frame = ttk.LabelFrame(self.scrollable_frame, text=f"Cable {index + 1}")
        frame.pack(fill='x', padx=5, pady=5)

        entries = {}
        
        # Cable parameters
        params = [
            ("Stiffness (N/m)", "stiffness"),
            ("Damping (Ns/m)", "damping"),
            ("Min Length (m)", "min_length"),
            ("Max Length (m)", "max_length"),
            ("Min Tension (N)", "min_tension"),
            ("Max Tension (N)", "max_tension"),
            ("Diameter (m)", "diameter")
        ]

        for i, (label, key) in enumerate(params):
            ttk.Label(frame, text=label).grid(row=i, column=0, padx=5, pady=2)
            entry = ttk.Entry(frame, width=15)
            entry.grid(row=i, column=1, padx=5, pady=2)
            entries[key] = entry

        return entries

    def update_cable_entries(self):
        # Clear existing entries
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()
        self.cable_entries.clear()

        try:
            num_cables = int(self.num_cables_entry.get())
            for i in range(num_cables):
                entries = self.create_cable_entry(i)
                self.cable_entries.append(entries)
                
                # Set default values
                entries["stiffness"].insert(0, "1000.0")
                entries["damping"].insert(0, "10.0")
                entries["min_length"].insert(0, "0.1")
                entries["max_length"].insert(0, "2.0")
                entries["min_tension"].insert(0, "10.0")
                entries["max_tension"].insert(0, "1000.0")
                entries["diameter"].insert(0, "0.002")
        except ValueError:
            print("Please enter a valid number of cables")

    def get_config(self):
        cables_config = []
        for entries in self.cable_entries:
            cable_config = {
                'stiffness': float(entries['stiffness'].get()),
                'damping': float(entries['damping'].get()),
                'min_length': float(entries['min_length'].get()),
                'max_length': float(entries['max_length'].get()),
                'min_tension': float(entries['min_tension'].get()),
                'max_tension': float(entries['max_tension'].get()),
                'diameter': float(entries['diameter'].get())
            }
            cables_config.append(cable_config)
        return cables_config

    def set_config(self, config):
        if not config:
            return
            
        # Update number of cables
        self.num_cables_entry.delete(0, tk.END)
        self.num_cables_entry.insert(0, str(len(config)))
        self.update_cable_entries()

        # Set values for each cable
        for i, (cable_config, entries) in enumerate(zip(config, self.cable_entries)):
            for key, value in cable_config.items():
                entries[key].delete(0, tk.END)
                entries[key].insert(0, str(value))