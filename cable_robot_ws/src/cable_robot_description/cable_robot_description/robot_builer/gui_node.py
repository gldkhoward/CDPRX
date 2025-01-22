
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from .config_handler import ConfigHandler
from .gui_components import (platform_frame, cables_frame, frame_frame, workspace_frame)

def get_workspace_root():
    pkg_dir = get_package_share_directory('cable_robot_description')
    workspace_root = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..'))
    if os.path.basename(workspace_root) == 'install':
        workspace_root = os.path.dirname(workspace_root)
    return workspace_root

class CableRobotGUI(Node):
    def __init__(self):
        super().__init__('cable_robot_description')
        
        # Set up workspace paths
        ws_root = get_workspace_root()
        self.base_output_dir = os.path.join(ws_root, 'config')
        os.makedirs(self.base_output_dir, exist_ok=True)
        
        self.config_path = os.path.join(self.base_output_dir, 'robot_config.yaml')
        self.config_handler = ConfigHandler(self.config_path)
        
        # Initialize the GUI
        self.create_gui()
        
        self.get_logger().info('Cable Robot GUI initialized')
        self.get_logger().info(f'Config will be saved to: {self.config_path}')

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Cable Robot Configuration")

        # Create notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(expand=True, fill='both', padx=5, pady=5)

        # Create frames for each tab
        self.platform_frame = platform_frame.PlatformFrame(notebook)
        self.cables_frame =cables_frame.CablesFrame(notebook)
        self.frame_frame = frame_frame.FrameFrame(notebook)
        self.workspace_frame = workspace_frame.WorkspaceFrame(notebook)

        # Add frames to notebook
        notebook.add(self.platform_frame, text="Platform")
        notebook.add(self.cables_frame, text="Cables")
        notebook.add(self.frame_frame, text="Frame")
        notebook.add(self.workspace_frame, text="Workspace")

        # Create bottom control panel
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill='x', padx=5, pady=5)

        # Add control buttons
        ttk.Button(control_frame, text="Save Configuration", 
                  command=self.save_configuration).pack(side='left', padx=5)
        ttk.Button(control_frame, text="Load Configuration", 
                  command=self.load_configuration).pack(side='left', padx=5)

        # Status label
        self.status_label = ttk.Label(control_frame, text="")
        self.status_label.pack(side='right', padx=5)

    def save_configuration(self):
        try:
            config = {
                'platform': self.platform_frame.get_config(),
                'cables': self.cables_frame.get_config(),
                'frame': self.frame_frame.get_config(),
                'workspace': self.workspace_frame.get_config()
            }
            
            self.config_handler.save_config(config)
            
            self.status_label.config(text="Configuration saved successfully!")
            self.get_logger().info(f'Configuration saved to: {self.config_path}')
            
            messagebox.showinfo("Success", 
                              f"Configuration saved successfully!\nFile: {self.config_path}")
            
        except Exception as e:
            self.status_label.config(text=f"Error: {str(e)}")
            self.get_logger().error(f'Error saving configuration: {str(e)}')
            messagebox.showerror("Error", f"An error occurred: {str(e)}")

    def load_configuration(self):
        try:
            config = self.config_handler.load_config()
            
            self.platform_frame.set_config(config.get('platform', {}))
            self.cables_frame.set_config(config.get('cables', {}))
            self.frame_frame.set_config(config.get('frame', {}))
            self.workspace_frame.set_config(config.get('workspace', {}))
            
            self.status_label.config(text="Configuration loaded successfully!")
            self.get_logger().info('Configuration loaded successfully')
            
        except Exception as e:
            self.status_label.config(text=f"Error: {str(e)}")
            self.get_logger().error(f'Error loading configuration: {str(e)}')
            messagebox.showerror("Error", f"An error occurred: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gui_node = CableRobotGUI()
        gui_node.root.mainloop()
    except Exception as e:
        print(f"Error running GUI: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()