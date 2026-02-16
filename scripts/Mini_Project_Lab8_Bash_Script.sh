#!/bin/bash

LOGFILE=$HOME/Downloads/putty.log
OUT=final_output.txt

echo "Processing putty"

if [ ! -f "$LOGFILE" ]; then
	echo "Log file not found: $LOGFILE"
	exit 1
fi

grep "^v[0-9]" "$LOGFILE" > final_output.txt

echo "process complete"

