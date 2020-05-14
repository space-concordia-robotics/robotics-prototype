#!/usr/bin/env python3
import robot.basestation.app as app
from robot.util.utils import run_shell
import re

excluded_branches = {'master', 'develop', 'staging', 'test'}

def get_branch_name():
    output, error = run_shell('git', 'symbolic-ref --short HEAD', False)
    return output.decode('utf8')[:-1]

def is_excluded_branch(branch_name, excluded_branches):
    """
    returns true if branch name is in list of excluded branches
    """
    return branch_name in excluded_branches

def get_issue_num(branch_name):
    """
    Gets issue number from branch name
    returns None if there is no issue number
    """
    issue_num = re.search(r'\d+$', branch_name)
    if issue_num is not None:
        issue_num = issue_num.group(0)
    return issue_num

def has_issue_num(branch_name):
    """
    check if there is an issue number
    returns false if the the output of get_issue_num is None, otherwise
    returns true
    """
    return not get_issue_num(branch_name) is None

def has_invalid_symbols(branch_name):
    """
    returns true if branch name contains any invalid symbols
    """
    invalid_symbols = {'/', "'", '!', '?', '@', '$', '%', ',', '.', '_', '+', '£', '=', '¬', '<', '>'}
    # Breaks down branch_name into a set of its characters, and checks for any matching elements with
    # the list of invalid symbols. Returns true if there are any matching elements found.
    # There had to be a seperate check for '\', because it wouldn't work when it was included
    # in the list of invalid symbols.
    return (bool(set(branch_name) & invalid_symbols)) or ('\\' in branch_name)

def has_double_hyphen(branch_name):
    """
    returns true if branch name has any double hyphens
    """
    return branch_name.find('--') != -1

def has_branch_name_error(branch_name):
    """
    run all name verification functions and print error message if any fails
    return true if branch name has error
    """
    wiki_url = 'https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions'
    trim_issue_num = re.sub(r'\d+$', '', branch_name)
    error_msg = 'Branch name is not named properly, please see wiki for formating: ' + wiki_url
    is_name_valid = True
    is_name_valid = trim_issue_num.islower()\
    and trim_issue_num.endswith('-')\
    and has_issue_num(branch_name)\
    and not has_invalid_symbols(branch_name)\
    and not branch_name.startswith('-')\
    and not branch_name.endswith('-')\
    and not has_double_hyphen(branch_name)

    if not is_name_valid:
        print(error_msg)

    return not is_name_valid

if __name__ == '__main__':
    branch_name = get_branch_name()
    if not is_excluded_branch(branch_name, excluded_branches):
        has_branch_name_error(branch_name)
