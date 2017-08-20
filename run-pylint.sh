#!/bin/bash
#-------------------------------------------------------------------------------
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   19.08.2017
#-------------------------------------------------------------------------------

PACKAGES="tl_detector twist_controller waypoint_updater"
SRCDIR="ros/src"
PYLINT_FLAGS=" \
--disable=RP0001 \
--disable=RP0002 \
--disable=RP0003 \
--disable=RP0004 \
--disable=RP0101 \
--disable=RP0401 \
--disable=RP0701 \
--disable=RP0801 \
--output-format=colorized"

if [ ! -x "`which pylint 2>/dev/null`" ]; then
    echo "[!] Unable to find pylint, please install it"
    exit 1
fi

if [ ! -r .pylint -o ! -d ros ]; then
    echo "[!] This program needs to be run from the root of the source tree"
    exit 1
fi

RET=0
TOPDIR=`pwd`
for P in ${PACKAGES}; do
    echo "====================================================="
    echo "[i] Checking package ${P}"
    DIR=${SRCDIR}/${P}
    if [ ! -d ${DIR} ]; then
        echo "[!] Directory ${DIR} does not exist; skipping."
        continue
    fi

    cd ${DIR}
    for FILE in `find -type f | grep py$`; do
        echo "[i] Analyzing ${FILE}..."
        pylint --rcfile ${TOPDIR}/.pylint ${PYLINT_FLAGS} ${FILE}
        TMPRET=$?
        if [ ${TMPRET} -ne 0 ]; then
            RET=${TMPRET}
        fi
    done
    cd ${TOPDIR}
done

exit ${RET}
