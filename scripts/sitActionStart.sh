#!/bin/bash

################################################
#  THIS SCRIPT WILL START THE SITTING AACTION  #
################################################


rosservice call /walking_controller/sit "down: true
weight_threshold: 250.0"


