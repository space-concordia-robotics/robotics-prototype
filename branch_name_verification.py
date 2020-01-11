#!/usr/bin/env python
import robot.basestation.app as app
from robot.basestation.app import run_shell
import re

excluded_branches = {'master', 'develop', 'staging', 'test'}

output, error = run_shell("git", "symbolic-ref --short HEAD", False)
branch_name = output.decode("utf8")[:-1]

def is_excluded_branch(branch_name, excluded_branches):
    """
    check if branch name is in list of excluded branches
    """
    return branch_name in excluded_branches

def get_issue_num(branch_name):
    """
    Gets issue number from branch named
    """
    issue_num = re.search(r'\d+$', branch_name)
    if issue_num is not None:
        issue_num = issue_num.group(0)
    return issue_num

def has_issue_num(branch_name):
    """
    check if there is an issue number
    """
    return not get_issue_num(branch_name) is None

def has_invalid_symbols(branch_name):
    """
    check for invalid invalid_symbols
    """
    invalid_symbols = {"/", "'", '!', '?', '@', '$', '%', ',', '.', '_', '+', '£', '=', '¬', '<', '>'}
    return (bool(set(branch_name) & invalid_symbols)) or ('\\' in branch_name)

def has_double_hyphen(branch_name):
    """
    Checkout for double hyphens
    """
    return branch_name.find('--') != -1

def has_branch_name_error(branch_name):
    """
    run all name verification functions and print error message if any fails
    """
    trim_issue_num = re.sub(r'\d+$', '', branch_name)
    error_msg = 'Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions'
    is_name_valid = True
    is_name_valid = trim_issue_num.islower()\
    and trim_issue_num.endswith('-')\
    and has_issue_num(branch_name)\
    and not has_invalid_symbols(branch_name)\
    and not has_double_hyphen(branch_name)

    if not is_name_valid:
        print(error_msg)

    return not is_name_valid

if __name__ == "__main__" and not is_excluded_branch(branch_name, excluded_branches):
    has_branch_name_error(branch_name)
