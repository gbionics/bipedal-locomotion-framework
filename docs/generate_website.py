#!/bin/python3

# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import subprocess
import os
import argparse

from generate_documentation_files import generate_documentation_files


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='generate_website.py is a python script useful to generate the '
                                                 'documentation of the bipedal-locomotion-framework repo.')
    parser.add_argument('--doxygen_awesome_path',
                        type=str,
                        required=True,
                        help='Path to the doxygen-awesome-css directory')
    args = parser.parse_args()

    generate_documentation_files(src_folder="../src",
                                 main_page="pages/main-page.dox",
                                 additional_pages=["pages/python-additional-info.md",
                                                   "pages/so3-minjerk.md"],
                                 input_files_path="./",
                                 doxygen_awesome_path=args.doxygen_awesome_path)

    try:
        subprocess.check_call(["doxygen", "Doxyfile"])
    except subprocess.CalledProcessError as error:
        raise ValueError(error)

