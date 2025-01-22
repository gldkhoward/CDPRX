import yaml
from typing import Dict, Any
from .robot_config import RobotConfiguration

class YAMLHandler:
    @staticmethod
    def save_config(config: RobotConfiguration, filepath: str) -> None:
        with open(filepath, 'w') as f:
            yaml.dump(config, f)
    
    @staticmethod
    def load_config(filepath: str) -> RobotConfiguration:
        with open(filepath, 'r') as f:
            return yaml.load(f, Loader=yaml.SafeLoader)