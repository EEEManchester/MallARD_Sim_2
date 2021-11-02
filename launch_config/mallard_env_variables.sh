#!/bin/bash

# Environment variables to determine launch setups for different circumstances
# Include this file in .bashrc to source it
# source ~/mallard_ws/src/MallARD/launch_config/mallard_env_variables.sh

export MALLARD_REAL=1            # 0: Simulation Mode. 1: Real Robot Mode. 
export MALLARD_COMP=1            # 0: Robot computer. 1: Base Staion comptuer. 2: another comptuer. Only relavant for Real Robot Mode.
export MALLARD_AUTONOMOUS=1      # 0: Manual Mode. 1: Autonomous Mode. 

export MALLARD_VERSION='001_EL' # The version of MallARD. E.g. 001_SIM, 002_SIM, 003_SIM, 001_REAL, 002_REAL, 003_REAL, 001_EL.
