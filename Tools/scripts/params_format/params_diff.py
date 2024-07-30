'''
tool to compare ArduPilot parameters. An output file is generated with the different parameters.
'''

import difflib
import sys

def compare_files(file1, file2, output_file):
    with open(file1, 'r') as f1, open(file2, 'r') as f2, open(output_file, 'w') as out:
        differ = difflib.Differ()
        diff = list(differ.compare(f1.readlines(), f2.readlines()))

        for line in diff:
            if line.startswith('- '):
                out.write(line[2:])

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python params_diff.py file1.parm file2.parm\nWaring: order matters! The first input file values will be written in the output file")
        sys.exit(1)

    input_file1 = sys.argv[1]
    input_file2 = sys.argv[2]
    output_file = "defaults.parm"

    compare_files(input_file1, input_file2, output_file)
    print(f"Differences written to: {output_file}")
