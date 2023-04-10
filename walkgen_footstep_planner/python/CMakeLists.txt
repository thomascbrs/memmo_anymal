#!/usr/bin/env python3
#
# Copyright 2022 University of Edinburgh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of  nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

SET(${FOOTSTEP_PLANNER_NAME}_SOURCES
    __init__.py
    FootStepPlanner.py
    FootStepTrajectory.py
    FootStepTrajectoryBezier.py
    GaitManager.py
    FootStepManager.py
    params.py
    )

SET(${FOOTSTEP_PLANNER_NAME}_TOOLS_SOURCES
    __init__.py
    Surface.py
    optimisation.py
    Filter.py
    Logger.py
    )

FOREACH(src ${${FOOTSTEP_PLANNER_NAME}_SOURCES})
PYTHON_INSTALL_ON_SITE(${FOOTSTEP_PLANNER_NAME} ${src})
ENDFOREACH(src ${FOOTSTEP_PLANNER_NAME}_SOURCES)

FOREACH(src ${${FOOTSTEP_PLANNER_NAME}_TOOLS_SOURCES})
PYTHON_INSTALL_ON_SITE(${FOOTSTEP_PLANNER_NAME}/tools ${src})
ENDFOREACH(src ${FOOTSTEP_PLANNER_NAME}_TOOLS_SOURCES)

ADD_LIBRARY(${FOOTSTEP_PLANNER_NAME} INTERFACE)
INSTALL(TARGETS ${FOOTSTEP_PLANNER_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
