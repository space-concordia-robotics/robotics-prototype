#!/usr/bin/env python
from subprocess import check_output
import re

def main():
    """
    declare variables and run error_checking function
    """
    excluded_branches = {'master', 'develop', 'staging', 'test'}
    invalid_symbols = {"/", "'", '!', '@', '$', '%', ',', '.', '_', '+', '£', '=', '¬', '<', '>'}
    branch_name = check_output(["git","symbolic-ref", "--short", "HEAD"]).decode("utf8")[0:-1]
    trim_issue_num = re.sub(r'\d+$', '', branch_name)
    branch_name_error = False
    branch_error_msg = 'Branch name is not named properly, please see wiki for formating: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions'
    error_checking(branch_name, trim_issue_num, invalid_symbols, excluded_branches, branch_name_error, branch_error_msg)

def branch_excluded_check(branch_name, excluded_branches):
    """
    check if branch name is in list of excluded branches
    """
    if branch_name in excluded_branches:
        return True

    else:
        return False

def upper_case_letters_check(trim_issue_num, branch_name_error):
    """
    check if trimed branch name only contains lower case characters
    """
    if not (trim_issue_num.islower()):
        branch_name_error = True

    return branch_name_error

def hyphen_seperator_check(trim_issue_num, branch_name_error):
    """
    check for hyphen seperator between branch name and issue number
    """
    if not trim_issue_num.endswith('-'):
        branch_name_error = True

    return branch_name_error

def issue_num_check(branch_name, branch_name_error):
    """
    check if there is an issue number
    """
    if re.search(r'\d+$', branch_name) is None:
        branch_name_error = True

    return branch_name_error

def invalid_symbols_check(branch_name, invalid_symbols, branch_name_error):
    """
    check for invalid invalid_symbols
    """
    if (bool(set(branch_name) & invalid_symbols)) or ('\\' in branch_name):
        branch_name_error = True

    return branch_name_error

def error_checking(branch_name, trim_issue_num, invalid_symbols, excluded_branches, branch_name_error, branch_error_msg):
    """
    run all name verification functions and print error message if any fails
    """
    if not branch_excluded_check(branch_name, excluded_branches):
        branch_name_error = upper_case_letters_check(trim_issue_num, branch_name_error)
        branch_name_error = hyphen_seperator_check(trim_issue_num, branch_name_error)
        branch_name_error = issue_num_check(branch_name, branch_name_error)
        branch_name_error = invalid_symbols_check(branch_name, invalid_symbols, branch_name_error)

        if branch_name_error:
            print(branch_error_msg)

if __name__ == "__main__":
    main()
