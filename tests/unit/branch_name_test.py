from subprocess import check_output
import re

def test_start_branch():
    excluded_branches = {'master', 'develop', 'staging', 'test'}
    invalid_symbols = {"/", "'", '!', '@', '$', '%', ',', '.', '_', '+', '£', '=', '¬', '<', '>'}
    branch_name = check_output(["git","symbolic-ref", "--short", "HEAD"]).decode("utf8")[0:-1]
    trim_issue_num = re.sub(r'\d+$', '', branch_name)
    branch_name_error = False
    error_checking(branch_name, trim_issue_num, invalid_symbols, excluded_branches, branch_name_error)

def branch_excluded_check(branch_name, excluded_branches):
    if branch_name in excluded_branches:
        return True

    else:
        return False

def upper_case_letters_check(trim_issue_num, branch_name_error):
    if not (trim_issue_num.islower()):
        branch_name_error = True

    return branch_name_error

def hyphen_seperator_check(trim_issue_num, branch_name_error):
    if not trim_issue_num.endswith('-'):
        branch_name_error = True

    return branch_name_error

def issue_num_check(branch_name, branch_name_error):
    if re.search(r'\d+$', branch_name) is None:
        branch_name_error = True

    return branch_name_error

def invalid_symbols_check(branch_name, invalid_symbols, branch_name_error):
    if (bool(set(branch_name) & invalid_symbols)) or ('\\' in branch_name):
        branch_name_error = True

    return branch_name_error

def error_checking(branch_name, trim_issue_num, invalid_symbols, excluded_branches, branch_name_error):
    if not branch_excluded_check(branch_name, excluded_branches):
        assert upper_case_letters_check(trim_issue_num, branch_name_error) == False
        assert hyphen_seperator_check(trim_issue_num, branch_name_error) == False
        assert issue_num_check(branch_name, branch_name_error) == False
        assert invalid_symbols_check(branch_name, invalid_symbols, branch_name_error) == False
