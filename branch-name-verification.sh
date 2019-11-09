#!/bin/bash
# Include any branches for which you wish to disable this script
BRANCHES_TO_SKIP=(master develop staging test)

# Error message to be returned if branch name is not named properly
BRANCH_NAME_ERROR="Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions"

# Get the current branch name
BRANCH_NAME=$(git symbolic-ref --short HEAD)

# Trims down the branch name to not include the issue number.
TRIM_NUM=$(echo $BRANCH_NAME | sed -e 's/[0-9]//g' )

# Checks to see if branch is excluded from rule
# If branch is not excluded, then proceed with script
if [[ " ${BRANCHES_TO_SKIP[*]} " == *" $BRANCH_NAME "* ]]; then
    exit 0
fi

# Checks to see if branch name includes any upper case letters. If it does, it exits and tells user to check wiki
if [[ $TRIM_NUM =~ [\$A-Z] ]];then
# If any upper case letters are in the branch name,
  echo $BRANCH_NAME_ERROR
  exit 1
fi
# Checks to see if branch name includes a hypen seperator before the issue number. If not, it exits and tells user to check wiki
if [[ "${TRIM_NUM: -1}" != *-* ]];then
# If a hyphen is not found,
  echo $BRANCH_NAME_ERROR
  exit 1
fi
# Checks to see if branch name includes various other symbols. If so, it exits and tells user to check wiki
if [[ $BRANCH_NAME == *[\/'!'\@\$\%\,\.\_\+\£\=\¬\<\>\]]* ]];then
# If any of these symbols are found,
  echo $BRANCH_NAME_ERROR
  exit 1
fi
