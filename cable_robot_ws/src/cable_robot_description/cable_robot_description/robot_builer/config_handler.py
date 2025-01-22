import yaml
from typing import Dict, Any

class ConfigHandler:
    def __init__(self, config_path: str):
        self.config_path = config_path

    def save_config(self, config: Dict[str, Any]) -> None:
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

    def load_config(self) -> Dict[str, Any]:
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)