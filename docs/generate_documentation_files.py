#!/usr/bin/env python3

# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import argparse
import os


def generate_documentation_files(input_files_path: str, src_folder: str,
                                 main_page: str, doxygen_awesome_path: str,
                                 additional_pages=[]):
    src_path = os.path.normpath(src_folder)
    input_files_path = os.path.normpath(input_files_path)
    main_page = os.path.normpath(main_page)
    doxygen_awesome_path = os.path.normpath(doxygen_awesome_path)

    folders = [os.path.normpath(folder[0]) for folder in os.walk(src_path)
               if os.path.basename(os.path.normpath(folder[0])) == 'include']

    try:
        input_file = open(os.path.join(input_files_path, 'Doxyfile.in'), 'rt')
        output_file = open(os.path.join(input_files_path, 'Doxyfile'), 'wt')

        for line in input_file:
            output_file.write(line.replace('@BLF_INCLUDE_FOLDERS@',
                                           "\"" + "\" \"".join(folders) + "\"")
                              .replace('@BLF_MAIN_PAGE@', main_page)
                              .replace('@BLF_ADDITIONAL_FILES@', "\"" + "\" \"".join(additional_pages) + "\"")
                              .replace('@BLF_DOXYGEN_AWESOME_PATH@', doxygen_awesome_path))

    except FileNotFoundError:
        print("Invalid doxyfile input file.")
    finally:
        input_file.close()
        output_file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='generate_doxyfile.py is a python script useful to generate the Doxyfile from Doxyfile.in.')
    parser.add_argument('--input_files_path', type=str, default='./', required=False, help='Path of the input files.')
    parser.add_argument('--src_folder', type=str, required=True, help='Path of the src folder.')
    parser.add_argument('--main_page', type=str, required=True, help='Path to the the main page.')
    parser.add_argument('--doxygen_awesome_path', type=str, required=True, help='Path to doxygen-awesome-css directory.')
    args = parser.parse_args()

    generate_documentation_files(input_files_path=args.input_files_path,
                                 src_folder=args.src_folder,
                                 main_page=args.main_page,
                                 doxygen_awesome_path=args.doxygen_awesome_path)
