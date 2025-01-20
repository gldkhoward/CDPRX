import os
import pathlib

def create_structure():
    # Define base directory structure
    structure = {
        'include/cable_robot_core': {
            'core': ['types.hpp', 'constants.hpp'],
            'kinematics': ['forward_kinematics.hpp', 'inverse_kinematics.hpp', 'jacobian.hpp'],
            'workspace': ['wrench_closure.hpp', 'wrench_feasible.hpp', 'workspace_analyzer.hpp'],
            'controllers': ['tension_controller.hpp', 'trajectory_controller.hpp'],
            'utils': ['math_utils.hpp', 'configuration.hpp', 'visualization_utils.hpp']
        },
        'src': {
            'core': ['types.cpp'],
            'kinematics': ['forward_kinematics.cpp', 'inverse_kinematics.cpp', 'jacobian.cpp'],
            'workspace': ['wrench_closure.cpp', 'wrench_feasible.cpp', 'workspace_analyzer.cpp'],
            'controllers': ['tension_controller.cpp', 'trajectory_controller.cpp'],
            'utils': ['math_utils.cpp', 'configuration.cpp']
        },
        'test': {
            'kinematics': ['test_forward_kinematics.cpp', 'test_inverse_kinematics.cpp'],
            'workspace': ['test_workspace.cpp'],
            'controllers': ['test_controllers.cpp']
        },
        'config': ['robot_params.yaml', 'controller_config.yaml'],
        'launch': ['cable_robot_core.launch.py'],
        'scripts/analysis': ['workspace_visualization.py', 'performance_analysis.py'],
        'docs': ['kinematics.md', 'workspace.md', 'controllers.md']
    }

    # Get current directory
    base_path = pathlib.Path.cwd()

    # Create directories and files
    for dir_path, contents in structure.items():
        # Create full path
        full_path = base_path / dir_path

        if isinstance(contents, list):
            # Create directory if it doesn't exist
            full_path.mkdir(parents=True, exist_ok=True)
            
            # Create files
            for file_name in contents:
                file_path = full_path / file_name
                if not file_path.exists():
                    file_path.touch()
                    print(f"Created file: {file_path}")
        else:
            # Handle nested directories
            for subdir, files in contents.items():
                subdir_path = full_path / subdir
                subdir_path.mkdir(parents=True, exist_ok=True)
                
                for file_name in files:
                    file_path = subdir_path / file_name
                    if not file_path.exists():
                        file_path.touch()
                        print(f"Created file: {file_path}")

if __name__ == "__main__":
    create_structure()