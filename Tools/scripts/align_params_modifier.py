#!/usr/bin/env python3
"""
ArduPilot Parameter File Modifier

This script modifies ArduPilot parameter files by removing specified parameters
and appending new ones at the end. Supports multiple base files with different operations.
"""
import os

# =============================================================================
# CONFIGURATION SECTION - MODIFY THIS PART
# =============================================================================

# Base files with their respective operations
BASE_FILES_CONFIG = {
    # M3
    "libraries/AP_HAL_ChibiOS/hwdef/AP6m-M3/defaults.parm": [
        (["A10","A10-M3"], "libraries/AP_HAL_ChibiOS/hwdef/AP6m-M3-A10/defaults.parm"),
        (["OF","OF-M3"], "libraries/AP_HAL_ChibiOS/hwdef/AP6m-M3-OF/defaults.parm"),
    ],

    # M450/M460/M490 4in1
    "libraries/AP_HAL_ChibiOS/hwdef/AP6-M460-ds/defaults.parm": [
        ("A10", "libraries/AP_HAL_ChibiOS/hwdef/AP6-M460-A10-ds/defaults.parm"),
        ("M490", "libraries/AP_HAL_ChibiOS/hwdef/AP6-M490-ds/defaults.parm"),
        (["M490", "A10"], "libraries/AP_HAL_ChibiOS/hwdef/AP6-M490-A10-ds/defaults.parm"),
        ("M450", "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450-ds/defaults.parm"),
        (["M450","A10"], "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450-A10-ds/defaults.parm"),
        (["M450", "A10","NO-GPS"], "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450-nogps-ds/defaults.parm"),
    ],

    # MR25 4in1
    "libraries/AP_HAL_ChibiOS/hwdef/AP6m-MR25/defaults.parm": [
        ("A10", "libraries/AP_HAL_ChibiOS/hwdef/AP6m-MR25-A10/defaults.parm"),
    (["A10","NO-GPS"], "libraries/AP_HAL_ChibiOS/hwdef/AP6m-MR25-no-gps/defaults.parm"),
    ],

    # M450 old ESC
    "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450/defaults.parm": [
        (["A10","OF","OF-M450"], "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450-A10-OF/defaults.parm"),
        ("A10", "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450-A10/defaults.parm"),
        (["OF","OF-M450"], "libraries/AP_HAL_ChibiOS/hwdef/AP6-M450-OF/defaults.parm"),
    ],

    # M460 old ESC
    "libraries/AP_HAL_ChibiOS/hwdef/AP6-M460/defaults.parm": [
        ("A10", "libraries/AP_HAL_ChibiOS/hwdef/AP6-M460-A10/defaults.parm"),
        ("AP5", "libraries/AP_HAL_ChibiOS/hwdef/AP5-M460/defaults.parm"),
        ("AP3", "libraries/AP_HAL_ChibiOS/hwdef/AP3-M460/defaults.parm"),
    ],

    # M490 old ESC
    "libraries/AP_HAL_ChibiOS/hwdef/AP6-M490/defaults.parm": [
        ("A10", "libraries/AP_HAL_ChibiOS/hwdef/AP6-M490-A10/defaults.parm"),
        ("AP5", "libraries/AP_HAL_ChibiOS/hwdef/AP5-M490/defaults.parm"),
        ("AP3", "libraries/AP_HAL_ChibiOS/hwdef/AP3-M490/defaults.parm"),
    ],

}

# Configuration rules
CONFIGURATIONS = {
    "A10": {
        "remove": ["PRX1_TYPE", "RNGFND1_TYPE", "RNGFND2_TYPE",
                   "MNT1_TYPE", "MNT1_RC_RATE", "CAM1_TYPE",
                   "RC7_OPTION", "RC9_OPTION", "RC11_OPTION",
                   "RC12_OPTION", "RC13_OPTION"],
        "add": """
# A10
FLTMODE6 6
LOIT_ACC_MAX 300
LOIT_SPEED 700
PILOT_SPEED_DN 200
PILOT_SPEED_UP 250
PILOT_Y_RATE 60
RC9_OPTION 19
SCR_USER6 1 # OF
"""
    },

    "A10-M3": {
        "remove": ["ATC_RAT_YAW_I", "ATC_RAT_YAW_P"],
        "add": """
# A10-M3
ATC_RAT_YAW_I 0.03
ATC_RAT_YAW_P 0.3
"""
    },

    "OF": {
        "remove": [],
        "add": """
# OF
ARMING_CHECK 982518 # no GPS and no camera
EK3_FLOW_DELAY 34
EK3_SRC2_POSXY 0
EK3_SRC2_POSZ 1
EK3_SRC2_VELXY 5
EK3_SRC2_VELZ 0
EK3_SRC2_YAW 1
EK3_SRC_OPTIONS 0
FLOW_TYPE 5
RNGFND2_MAX_CM 430
RNGFND2_MIN_CM 1
RNGFND2_ORIENT 25
RNGFND2_TYPE 10
"""
    },

    "OF-M3": {
        "remove": [],
        "add": """
# OF-M3
FLOW_ORIENT_YAW 9000
FLOW_POS_X -0.02
FLOW_POS_Y 0.041
FLOW_POS_Z 0.02
RELAY6_FUNCTION 1
RELAY6_PIN 93
RNGFND2_POS_X -0.02
RNGFND2_POS_Y 0.038
"""
    },

    "OF-M450": {
        "remove": [],
        "add": """
# OF-M450
FLOW_ORIENT_YAW 9000
FLOW_POS_X 0.045
FLOW_POS_Y 0.045
FLOW_POS_Z 0.02
RNGFND2_POS_X 0.045
RNGFND2_POS_Y 0.04
"""
    },

    "SERVO-MNT": {
        "remove": [],
        "add": """
# MOUNT SERVO
MNT1_RC_RATE 0
MNT1_TYPE 1
RC12_OPTION 213
SERVO6_FUNCTION 7
"""
    },

    "NO-GPS": {
        "remove": [],
        "add": """
# NO GPS
AHRS_GPS_USE 0
BATT_FS_CRT_ACT 0
BATT_FS_LOW_ACT 0
COMPASS_ENABLE 0
EK3_SRC1_POSXY 0
EK3_SRC1_VELXY 0
EK3_SRC1_VELZ 0
EK3_SRC1_YAW 0
FLTMODE1 2
FLTMODE4 2
FLTMODE6 2
GPS_TYPE 0
"""
    },

    "DSHOT": {
        "remove": ["MOT_SPIN_MAX"],
        "add": """
# DSHOT
MOT_PWM_TYPE 5
SERVO_DSHOT_ESC 1
SERVO_DSHOT_RATE 2
"""
    },

    "AP5": {
        "remove": ["AHRS_ORIENTATION"],
        "add": """
# AP5
"""
    },

    "AP3": {
        "remove": ["AHRS_ORIENTATION", "MNT1_TYPE", "MNT1_RC_RATE", "RC11_OPTION",
                   "RC13_OPTION", "RC9_OPTION", "CAM1_TYPE"],
        "add": """
# AP3
RC9_OPTION 19
SR4_EXTRA1 50
SR4_EXTRA2 10
SR4_EXTRA3 3
SR4_EXT_STAT 2
SR4_POSITION 3
SR4_RAW_SENS 2
SR4_RC_CHAN 50
"""
    },

    "M490": {
        "remove": [],
        "add": """
# M490 PID
ATC_ANG_PIT_P 14
ATC_ANG_RLL_P 10
ATC_ANG_YAW_P 4.8
ATC_RAT_PIT_D 0.013
ATC_RAT_PIT_I 0.17
ATC_RAT_PIT_P 0.17
ATC_RAT_RLL_D 0.014
ATC_RAT_RLL_I 0.16
ATC_RAT_RLL_P 0.16
ATC_RAT_YAW_I 0.08
ATC_RAT_YAW_P 0.8
"""
    },

    "M450": {
        "remove": ["BATT_CAPACITY"],
        "add": """
# M450 PID
ATC_ACCEL_P_MAX 60000
ATC_ACCEL_R_MAX 60000
ATC_ACCEL_Y_MAX 30000
ATC_ANG_PIT_P 10
ATC_ANG_RLL_P 10
ATC_ANG_YAW_P 7
ATC_RAT_PIT_D 0.008
ATC_RAT_PIT_I 0.11
ATC_RAT_PIT_P 0.11
ATC_RAT_RLL_D 0.006
ATC_RAT_RLL_I 0.11
ATC_RAT_RLL_P 0.11
ATC_RAT_YAW_D 0.01
ATC_RAT_YAW_FLTE 1.36
ATC_RAT_YAW_I 0.06
ATC_RAT_YAW_P 0.6
ATC_SLEW_YAW 8000
BATT_CAPACITY 4200
PSC_ACCZ_I 0.6
PSC_ACCZ_P 0.3
"""
    },

}
# =============================================================================
# SCRIPT CODE - NO NEED TO MODIFY BELOW THIS LINE
# =============================================================================

import os
from typing import List, Set, Dict, Tuple, Union


def apply_single_config(lines: List[str], config_name: str) -> List[str]:
    """
    Apply a single configuration to the parameter lines.
    
    Args:
        lines: List of parameter file lines
        config_name: Configuration name to apply
        
    Returns:
        Modified list of lines
    """
    if config_name not in CONFIGURATIONS:
        print(f"✗ Configuration '{config_name}' not found")
        return lines
    
    config = CONFIGURATIONS[config_name]
    remove_params = set(config.get("remove", []))
    add_params = config.get("add", "")
    
    # Process lines - keep only those not in remove list
    filtered_lines = []
    for line in lines:
        line_stripped = line.strip()
        if line_stripped and not line_stripped.startswith('#'):
            param_name = line_stripped.split()[0] if line_stripped.split() else ""
            if param_name not in remove_params:
                filtered_lines.append(line)
        else:
            # Keep comments and empty lines
            filtered_lines.append(line)
    
    # Add new parameters at the end
    if add_params:
        # Ensure there's a newline before adding new params
        if filtered_lines and not filtered_lines[-1].endswith('\n'):
            filtered_lines.append('\n')
        
        # Add each line from add_params
        for add_line in add_params.split('\n'):
            if add_line.strip():  # Skip empty lines
                filtered_lines.append(add_line + '\n')
    
    return filtered_lines


def process_parameter_file(input_path: str, output_path: str, configs: Union[str, List[str]]) -> bool:
    """
    Process a parameter file by applying one or more configurations in sequence.
    
    Args:
        input_path: Path to input parameter file
        output_path: Path to output parameter file
        configs: Single config name or list of config names to apply in order
        
    Returns:
        True if successful, False otherwise
    """
    # Normalize configs to list
    if isinstance(configs, str):
        config_list = [configs]
        config_display = configs
    else:
        config_list = configs
        config_display = " -> ".join(configs)
    
    # Validate all configs exist
    for config_name in config_list:
        if config_name not in CONFIGURATIONS:
            print(f"✗ Configuration '{config_name}' not found")
            return False
    
    try:
        # Read input file
        with open(input_path, 'r') as f:
            lines = f.readlines()
        
        # Apply each configuration in sequence
        current_lines = lines
        for config_name in config_list:
            current_lines = apply_single_config(current_lines, config_name)
        
        # Create output directory if it doesn't exist
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # Write output file
        with open(output_path, 'w') as f:
            f.writelines(current_lines)
        
        print(f"✓ Processed [{config_display}]: {os.path.basename(input_path)} -> {output_path}")
        return True
        
    except Exception as e:
        print(f"✗ Error processing [{config_display}]: {e}")
        return False


def main():
    """Main function to process all configured file operations."""
    print("ArduPilot Parameter File Modifier")
    print("=" * 60)
    
    total_success = 0
    total_operations = 0
    
    # Process each base file and its operations
    for base_file, operations in BASE_FILES_CONFIG.items():
        print(f"\nProcessing base file: {base_file}")
        print("-" * 60)
        
        # Check if base file exists
        if not os.path.exists(base_file):
            print(f"✗ Base file not found: {base_file}")
            continue
        
        success_count = 0
        operation_count = len(operations)
        total_operations += operation_count
        
        # Process each operation for this base file
        for configs, output_path in operations:
            if process_parameter_file(base_file, output_path, configs):
                success_count += 1
                total_success += 1
        
        print(f"Base file summary: {success_count}/{operation_count} operations successful")
    
    print("=" * 60)
    print(f"Overall summary: {total_success}/{total_operations} operations successful")


if __name__ == "__main__":
    main()
