#!/bin/bash

function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

function print_result() {
  if [ $1 -eq 0 ]; then
    echo_green "[Passed]"
  else
    # last command had a non-zero exit, we're done
    echo_red "[Failed]"
    exit $1
  fi
  echo ""
}

# run the dao unit tests first
echo_blue "Test 1: Server DAO Unit Tests"
cd server/src
python test.py
print_result $? # check return value of last command


echo "Setup db for client_rest tests"
python setup_db_for_client_tests.py
python server.py -g &
serverPID=$! # get PID for last executed command
sleep 5

# move to the client_rest test directory
echo_blue "Test 2: Client Rest Integration Tests"
cd ../../client/lib
python test.py
retVal=$? # return value of last command

# shutdown the server process
kill -9 $serverPID
# at least on the mac, i have to kill both of these in order
# for the process to actually die (something with python3.7?)
let otherServerPID=$serverPID+2
# the || makes it fail quietly if there's no second PID
kill -9 $otherServerPID || echo "No second PID to kill"

print_result $retVal