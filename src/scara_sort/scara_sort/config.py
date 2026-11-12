"""Find and read the YAML files in this package's config folder."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def find_config(path):
    """Use the path as given, or else look in this package's installed share folder.

    So `config/items.yaml` finds the installed copy, and any other path works too.
    """
    if os.path.exists(path):
        return path
    return os.path.join(get_package_share_directory('scara_sort'), path)


def load_yaml(path):
    with open(find_config(path)) as f:
        return yaml.safe_load(f)
