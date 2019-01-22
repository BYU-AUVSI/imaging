#!/bin/bash

function verifyLastExit() {
    retVal=$? # return value of last command
    if [ $retVal -ne 0 ]; then
        # last command had a non-zero exit, we're done
        exit $retVal
    fi
}

# run the dao unit tests first
cd server/src
python test.py
verifyLastExit

echo "=============================="
echo "Setup db for client_rest tests"
python setup_db_for_client_tests.py
verifyLastExit

python server.py &
serverPID=$! # get PID for last executed command
sleep 5

# move to the client_rest test directory
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

if [ $retVal -ne 0 ]; then
    # last command had a non-zero exit, we're done
    exit $retVal
fi
