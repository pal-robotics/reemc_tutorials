#!/bin/bash

# text formatting variables
TXT_BF='\033[1m'
TXT_RED='\e[31m'
TXT_GREEN='\e[32m'
TXT_CYAN='\e[96m'
TXT_RESET='\e[0m'

# Usage: DUR_STR=$(_duration_str START_TIME END_TIME)
function _duration_str
{
  local DIFF=$(echo "$2 - $1" | bc)
  local DIFF_M=$(echo "($DIFF / 60) % 60" | bc)
  local DIFF_S=$(echo "$DIFF - $DIFF_M * 60" | bc)

  echo `printf "%dm %.2fs\n" $DIFF_M $DIFF_S`
}

function _assert_success
{
  if [ $1 -ne 0 ]; then
    echo -e "${TXT_RED}Upper body joints test failed.${TXT_RESET}"
    exit $1
  fi
}

function _move_joint
{
  rosrun play_motion move_joint $@
  _assert_success $?
}

function _play_motion
{
  axcli /play_motion "$@"
  _assert_success $?
}

function _go_home
{
  axcli --wait /play_motion "{motion_name: 'home'}" 2>/dev/null # Try to reach the pose directly...
  if [ $? -ne 0 ]; then
    # ...if not possible, follow a two-step approach
    _play_motion "{motion_name: 'interact'}"
    _play_motion "{motion_name: 'home', skip_planning: true}"
  fi
}

# Usage:_yes_no_question "Are you hungry?"
# Function prints:
# Are you hungry? [yes/no]
# and waits for user input. If 'yes', returns; if 'no', exits with code 1, else keeps asking for yes/no input
function _yes_no_question
{
  local ready=''
  echo -e "$@ [yes/no]"
  while [[ $ready != 'yes' ]]; do
    read ready

    if [[ $ready == 'no' ]]; then
      echo 'Aborting...'
      exit 1
    fi

    if [[ $ready != 'yes' ]]; then
      echo -e "Please answer ${TXT_BF}yes${TXT_RESET} or ${TXT_BF}no${TXT_RESET} to the above question, "\
  "or ctrl+c to abort."
    fi
  done
}
