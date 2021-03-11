#!/bin/bash
#
#  additional steps for the installation procedure
#
#  Copyright (C)
#  Honda Research Institute Europe GmbH
#  Carl-Legien-Str. 30
#  63073 Offenbach/Main
#  Germany
#
#  UNPUBLISHED PROPRIETARY MATERIAL.
#  ALL RIGHTS RESERVED.
#
#


cd build/${MAKEFILE_PLATFORM}
make doc
make install


# EOF
