#!/bin/bash
#If there is already a commit message this hook is disabled
if [[ "$2" = "commit" ]]; then
    #statements
    exit 0
fi
# Include any branches for which you wish to disable this script
BRANCHES_TO_SKIP=(master develop staging test)

#Error message to be returned if branch name is not named properly
BRANCH_NAME_ERROR="Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Work-Flow"

# Get the current branch name
BRANCH_NAME=$(git symbolic-ref --short HEAD)

# Trims down the branch name to only include the issue number.
TRIMMED=$(echo $BRANCH_NAME | sed -e 's/[^0-9][^0-9]*\([0-9][0-9]*\).*/\1/g' )

# If branch is not excluded, then proceed with script
if [[ " ${BRANCHES_TO_SKIP[*]} " == *" $BRANCH_NAME "* ]]; then
    exit 0
fi

# Checks to make sure trimmed is a number, if not it exits and tells user to check wiki
if [[ $TRIMMED =~ [\$0-9] ]];then
# If it isn't excluded, preprend the trimmed branch identifier to the given message
  if [ -n "$BRANCH_NAME" ]  && [[ ! $BRANCH_EXCLUDED -eq 1 ]]; then
    sed -i.bak -e "1s/^/[#$TRIMMED] /" $1
fi
 else
    echo $BRANCH_NAME_ERROR
    exit 1
fi
