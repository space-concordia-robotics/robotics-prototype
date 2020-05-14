#!/usr/bin/env python3
import robot.basestation.app as app
from robot.util.utils import run_shell, write_to_file
from branch_name_verification_hook import is_excluded_branch, get_issue_num, get_branch_name, excluded_branches, wiki_url, error_msg
import re
import sys

def add_commit_issue_num(branch_name, error_msg):
    """
    Add commit issue number to commit message, return error if there is no issue number found in branch name
    """
    issue_num = get_issue_num(branch_name)

    if not get_issue_num(branch_name) is None:
        write_to_file(sys.argv[1], '[#' + issue_num + '] ', 0, 0)
    else:
        print(error_msg)

if __name__ == '__main__':
    branch_name = get_branch_name()
    #If there is already a commit message this hook is disabled
    if not is_excluded_branch(branch_name, excluded_branches) and sys.argv[2] != "commit":
        add_commit_issue_num(branch_name, error_msg)
