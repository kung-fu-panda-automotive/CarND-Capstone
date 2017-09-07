#!/bin/bash
#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   06.09.2017
#-------------------------------------------------------------------------------

echo "====================================================="
echo "Deleting previous build files"

rm -rf ./sphinx/build
mkdir ./sphinx/build

echo "Building Sphinx Documentation"
cd sphinx
make html
cd ..