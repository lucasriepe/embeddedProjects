#!/usr/bin/env python3
"""
Script to generate configuration from .env file (WiFi config removed)
"""

import os
from pathlib import Path

def load_env_file(env_path):
    """Loads the .env file and returns a dictionary"""
    env_vars = {}
    if os.path.exists(env_path):
        with open(env_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    env_vars[key.strip()] = value.strip()
    return env_vars

# WiFi config generation removed

def main():
    print("WiFi config generation has been removed from this script.")

if __name__ == "__main__":
    main()