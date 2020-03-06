#!/usr/bin/env python3

import os
import yaml
import argparse
from mako.template import Template
import contextlib

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.realpath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename",
        default='occ_sensor.yaml',
        help="Input File Name")
    parser.add_argument(
        "-i", "--input-dir",
        dest='inputdir',
        default=script_dir,
        help="Input directory")

    args = parser.parse_args()

    # Default to the one that is in the current.
    yaml_dir = script_dir
    yaml_file = os.path.join(yaml_dir, 'occ_sensor.yaml')

    if args.inputdir:
        yaml_dir = args.inputdir

    if args.filename:
        yaml_file = os.path.join(yaml_dir, args.filename)

    with open(yaml_file, 'r') as fd:
        ifile = yaml.safe_load(fd)

        # Render the mako template
        template = os.path.join(script_dir, 'occ_sensor.mako.hpp')
        t = Template(filename=template)
        with open('occ_sensor.hpp', 'w') as fd:
            fd.write(
                t.render(
                    occDict=ifile))
