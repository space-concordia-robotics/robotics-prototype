import branch_name_verification
from branch_name_verification import is_excluded_branch, has_upper_case_letters, has_hyphen_seperator, has_issue_num, has_invalid_symbols, branch_name_error

excluded_branches = {'test-1', 'sdfsdfs', '!@dvsfosk*&*&^fs4943'}

def test_excluded_branches():
    assert is_excluded_branch('test', excluded_branches) == False
    assert is_excluded_branch('test-1', excluded_branches) == True
    assert is_excluded_branch('!@dvsfosk*&*&^fs4943', excluded_branches) == True

def test_upper_case_letters():
    assert has_upper_case_letters('feat-test-branch-') == False
    assert has_upper_case_letters('Feat-test-branch-') == True
    assert has_upper_case_letters('feat-test-Branch-') == True
    assert has_upper_case_letters('FEAT-TEST-BRANCH-') == True

def test_hyphen_seperator():
    assert has_hyphen_seperator('feat-test-branch') == False
    assert has_hyphen_seperator('feat-test-branch-') == True

def test_issue_num():
    assert has_issue_num('feat-test-branch-123') == True
    assert has_issue_num('feat-test-branch') == False

def test_invalid_symbols():
    assert has_invalid_symbols('feat-test-branch-123') == False
    assert has_invalid_symbols('feat-test-br/anch-123') == True
    assert has_invalid_symbols('feat-test-b\\ranch-123') == True
    assert has_invalid_symbols('f@eat-tes?t-b\\ranch-12?3') == True
    assert has_invalid_symbols('feat-test-branch-12?3') == True
    assert has_invalid_symbols('!@#$-!@$%-(*&)-)()-123') == True

def test_branch_name():
    assert branch_name_error('test-1', excluded_branches) == False
    assert branch_name_error('Feat-test-branch-123', excluded_branches) == True
    assert branch_name_error('feat-test-branch123', excluded_branches) == True
    assert branch_name_error('feat-test-branch-', excluded_branches) == True
    assert branch_name_error('fe@t-test-branch-123', excluded_branches) == True
    assert branch_name_error('feat-test-branch-123', excluded_branches) == False
    assert branch_name_error('upgrade-ubuntu-18-304', excluded_branches) == False
