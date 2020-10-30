#!/bin/bash
# Test script for travis-ci.org continuous integration.

# Stop script early on any error; check variables
set -eu

# Paths to tools installed by travis-install.sh
MAIN_DIR=${PWD}
BUILD_DIR=${PWD}/travis_build
export PATH=${BUILD_DIR}/pru-gcc/bin:${PATH}
PYTHON=${BUILD_DIR}/python-env/bin/python


######################################################################
# Travis CI helpers
######################################################################

start_test()
{
    echo "::group::=============== $1 $2"
    set -x
}

finish_test()
{
    set +x
    echo "=============== Finished $2"
    echo "::endgroup::"
}


######################################################################
# Check for whitespace errors
######################################################################

start_test check_whitespace "Check whitespace"
./scripts/check_whitespace.sh
finish_test check_whitespace "Check whitespace"


######################################################################
# Run compile tests for several different MCU types
######################################################################

DICTDIR=${BUILD_DIR}/dict
mkdir -p ${DICTDIR}

for TARGET in test/configs/*.config ; do
    start_test mcu_compile "$TARGET"
    make clean
    make distclean
    unset CC
    cp ${TARGET} .config
    make olddefconfig
    make V=1
    finish_test mcu_compile "$TARGET"
    cp out/klipper.dict ${DICTDIR}/$(basename ${TARGET} .config).dict
done


######################################################################
# Verify klippy host software
######################################################################

start_test klippy "Test invoke klippy"
$PYTHON scripts/test_klippy.py -d ${DICTDIR} test/klippy/*.test
finish_test klippy "Test invoke klippy"
