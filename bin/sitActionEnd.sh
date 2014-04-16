#!/bin/bash

################################################
#  THIS SCRIPT WILL START THE SITTING AACTION  #
################################################


rosservice call /walking_controller/sit "down: false
weight_threshold: 600.0"


