#!/bin/bash
# Usage: ./kill_by_pattern.sh <pattern>

if [ $# -ne 1 ]; then
  echo "Usage: $0 <pattern>"
  exit 1
fi

PATTERN="$1"

ps aux | grep -F "$PATTERN" | grep -v grep | grep -v "$0" | awk '{ print $2 }' | while read PID; do
    if [ -n "$PID" ]; then
        echo "Killing PID: $PID (matching \"$PATTERN\")"
        kill -9 "$PID"
    fi
done
