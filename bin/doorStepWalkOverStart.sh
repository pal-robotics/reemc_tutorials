#!/bin/bash

################################################
#  THIS SCRIPT WILL START THE DOOR STEP ACTION #
################################################

rosservice call /walking_controller/over_obstacle "{}"

