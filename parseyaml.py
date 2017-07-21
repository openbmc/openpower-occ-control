#!/usr/bin/env python

import os
import yaml
from mako.template import Template
import contextlib

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(script_dir, 'occ_sensor.yaml'), 'r') as fd:
        yamlDict = yaml.safe_load(fd)

        # Render the mako template
        template = os.path.join(script_dir, 'occ_sensor.mako.hpp')
        t = Template(filename=template)
        with open('occ_sensor.hpp', 'w') as fd:
            fd.write(
                t.render(
                    occDict=yamlDict))
