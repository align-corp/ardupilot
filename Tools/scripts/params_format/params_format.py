'''
tool to manipulate ArduPilot parameters file to a default format,
so they could be easy compared.
'''

import re
import sys
import os

line_to_delete = ["AHRS_TRIM", "BARO1", "BARO2", "BARO3", "COMPASS_DEV", "COMPASS_DIA", "COMPASS_MOT", "COMPASS_ODI",
                  "COMPASS_OFS", "COMPASS_PRIO", "COMPASS_SCALE", "INS_ACC1", "INS_ACC2","INS_ACC3",
                  "INS_ACCOFF", "INS_ACCSCAL", "INS_ACC_ID", "INS_GYR1", "INS_GYR2", "INS_GYR3", "INS_GYROFF"]

def transform_line(line):
    # Use regular expressions to match the desired pattern
    match = re.match(r'^\s*([^,\s]+)\s*([-+]?\d*\.?\d+)\s*$', line)
    
    if match:
        non_numeric_part = match.group(1).strip()
        numeric_part = match.group(2).strip()

        # Check if the numeric part is an integer
        if float(numeric_part) == int(float(numeric_part)):
            numeric_part = int(float(numeric_part))
        else:
            numeric_part = float(numeric_part)

        # Combine the matched parts into the desired format
        transformed_line = f"{non_numeric_part} {numeric_part}"

        return transformed_line
    else:
        # If the line doesn't match the expected pattern, return it as is
        return line.strip()

def transform_file(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            # Iterate over each substring and check if it appears in the line
            skip = False
            for substring in line_to_delete:
                if substring in line:
                    skip = True
                    break
            if not skip:
                transformed_line = transform_line(line)
                outfile.write(transformed_line + '\n')

def generate_output_filename(input_filename):
    base_name, extension = os.path.splitext(input_filename)
    return f"{base_name}_format.parm"

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python params_format.py input_file")
        sys.exit(1)

    input_file_path = sys.argv[1]
    output_file_path = generate_output_filename(input_file_path)

    transform_file(input_file_path, output_file_path)
    print(f"Transformation complete. Check the output file: {output_file_path}")
