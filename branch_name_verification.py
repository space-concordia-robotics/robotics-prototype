#!/usr/bin/env python
import robot.basestation.app as app
from robot.basestation.app import run_shell
import re

excluded_branches = {'master', 'develop', 'staging', 'test'}

output, error = run_shell("git symbolic-ref --short HEAD")
branch_name = output.decode("utf8")[0:-1]

def is_excluded_branch(branch_name, excluded_branches):
    """
    check if branch name is in list of excluded branches
    """
    return branch_name in excluded_branches

def has_upper_case_letters(trim_issue_num):
    """
    check if trimed branch name only contains lower case characters
    """
    return not (trim_issue_num.islower())

def has_hyphen_seperator(trim_issue_num):
    """
    check for hyphen seperator between branch name and issue number
    """
    return trim_issue_num.endswith('-')

def has_issue_num(branch_name):
    """
    check if there is an issue number
    """
    return not re.search(r'\d+$', branch_name) is None

def has_invalid_symbols(branch_name):
    """
    check for invalid invalid_symbols
    """
    invalid_symbols = {"/", "'", '!', '?', '@', '$', '%', ',', '.', '_', '+', '£', '=', '¬', '<', '>'}
    return (bool(set(branch_name) & invalid_symbols)) or ('\\' in branch_name)

def branch_name_error(branch_name, excluded_branches):
    """
    run all name verification functions and print error message if any fails
    """
    trim_issue_num = re.sub(r'\d+$', '', branch_name)
    error_msg = 'Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions'
    errors = {}
    if not is_excluded_branch(branch_name, excluded_branches):
        errors[0] = has_upper_case_letters(trim_issue_num)
        errors[1] = not has_hyphen_seperator(trim_issue_num)
        errors[2] = not has_issue_num(branch_name)
        errors[3] = has_invalid_symbols(branch_name)

        if not all(errors[x] == False for x in errors):
            print(error_msg)

    return not all(errors[x] == False for x in errors)

if __name__ == "__main__":
    branch_name_error(branch_name, excluded_branches)
