#!/bin/bash

# Include any branches for which you wish to disable this script
  BRANCHES_TO_SKIP=(master develop staging test)

# Get the current branch name
BRANCH_NAME=$(git symbolic-ref --short HEAD)

# Trims down the branch name to only include the issue number.
TRIMMED=$(echo $BRANCH_NAME | sed -e 's/[^0-9][^0-9]*\([0-9][0-9]*\).*/\1/g' )

#Checks to see if branch is excluded, if so exit script and allow regular commit
#If branch is not excluded, then proceed with script
if [[ $BRANCH_NAME == $BRANCHES_TO_SKIP  ]];then
	exit 0
fi
#checks to make sure trimmed is a number, if not it exits and tells user to check wiki
if [[ $TRIMMED =~ [\$0-9] ]];then
# If it isn't excluded, preprend the trimmed branch identifier to the given message
  if [ -n "$BRANCH_NAME" ]  && [[ ! $BRANCH_EXCLUDED -eq 1 ]]; then
    sed -i.bak -e "1s/^/[#$TRIMMED] /" $1
fi
 else
    echo "Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Work-Flow"
    exit 1
fi
