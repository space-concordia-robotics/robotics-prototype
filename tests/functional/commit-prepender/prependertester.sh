#!/usr/bin/env bash
BRANCH_NAME=$1

# Trims down the branch name to only include the issue number.
TRIMMED=$(echo $BRANCH_NAME | sed -e 's/[^0-9][^0-9]*\([0-9][0-9]*\).*/\1/g' )

echo "TRIMMED: $TRIMMED"
