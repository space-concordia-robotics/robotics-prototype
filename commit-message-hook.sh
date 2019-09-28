#!/bin/bash
#If there is already a commit message this hook is disabled
if [[ "$2" = "commit" ]]; then
    #statements
    exit 0
fi
# Include any branches for which you wish to disable this script
  BRANCHES_TO_SKIP=(master develop staging test)

#Error message to be returned if branch name is not named properly
ERROR="Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Work-Flow"

# Get the current branch name
BRANCH_NAME=$(git symbolic-ref --short HEAD)

# Trims down the branch name to not include the issue number.
TRIM_NUM=$(echo $BRANCH_NAME | sed -e 's/[0-9]//g' )

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
    echo $ERROR
    exit 1
fi
#Checks to see if branch name includes any upper case letters. If it does, it exits and tells user to check wiki
if [[ $TRIM_NUM =~ [\$A-Z] ]];then
# If any upper case letters are in the branch name,
  echo $ERROR
  exit 1
fi
#Checks to see if branch name includes a hypen seperator before the issue number. If not, it exits and tells user to check wiki
if [[ "${TRIM_NUM: -1}" != *-* ]];then
# If a hyphen is not found,
  echo $ERROR
  exit 1
fi
#Checks to see if branch name includes various other symbols. If so, it exits and tells user to check wiki
if [[ $BRANCH_NAME == *[\/'!'\@\$\%\&\(\)\:\,\.\_\+\|\~\;\£\=\¬\<\>\]]* ]];then
# If any of these symbols are found,
  echo $ERROR
  exit 1
fi
