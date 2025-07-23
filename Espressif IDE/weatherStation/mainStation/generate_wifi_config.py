#!/usr/bin/env python3
"""
Script to generate WiFi configuration from .env file
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

def generate_wifi_config_header(env_vars, output_path):
    """Generates the wifi_config.h file"""
    wifi_ssid = env_vars.get('WIFI_SSID', 'YourWiFiName')
    wifi_password = env_vars.get('WIFI_PASSWORD', 'YourWiFiPassword')
    timezone = env_vars.get('TIMEZONE', 'CET-1CEST,M3.5.0,M10.5.0/3')
    
    header_content = f'''#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// Automatically generated from .env file
// DO NOT EDIT MANUALLY!

#define WIFI_SSID "{wifi_ssid}"
#define WIFI_PASS "{wifi_password}"
#define TIMEZONE_STRING "{timezone}"

#endif // WIFI_CONFIG_H
'''
    
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(header_content)
    
    print(f"WiFi configuration generated: {output_path}")
    print(f"SSID: {wifi_ssid}")
    print(f"Password: {'*' * len(wifi_password)}")
    print(f"Timezone: {timezone}")

def main():
    # Define paths
    script_dir = Path(__file__).parent
    env_file = script_dir / '.env'
    output_file = script_dir / 'main' / 'wifi_config.h'
    
    # Load .env file
    env_vars = load_env_file(env_file)
    
    if not env_vars:
        print("Warning: .env file not found or empty!")
        print("Create .env file with WIFI_SSID and WIFI_PASSWORD")
        return
    
    # Generate header file
    generate_wifi_config_header(env_vars, output_file)

if __name__ == '__main__':
    main()