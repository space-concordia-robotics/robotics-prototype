import branch_name_verification as branch_name_verification
from branch_name_verification import is_excluded_branch, get_issue_num, has_issue_num, has_invalid_symbols, has_branch_name_error

excluded_branches = {'test-1', 'sdfsdfs', '!@dvsfosk*&*&^fs4943'}

def test_excluded_branches():
    """
    test excluded branches checking
    """
    assert is_excluded_branch('test', excluded_branches) == False
    assert is_excluded_branch('test-1', excluded_branches) == True
    assert is_excluded_branch('!@dvsfosk*&*&^fs4943', excluded_branches) == True

def test_get_issue_num():
    assert get_issue_num('upgrade-ubuntu-18-304') == '304'
    assert get_issue_num('upgrade-ubuntu-18') == '18'
    assert get_issue_num('304-upgrade-ubuntu-18') == '18'

def test_has_issue_num():
    """
    test issue number checking
    """
    assert has_issue_num('feat-test-branch-123') == True
    assert has_issue_num('feat-test-branch') == False

def test_invalid_symbols():
    """
    test invalid symbols checking
    """
    assert has_invalid_symbols('feat-test-branch-123') == False
    assert has_invalid_symbols('feat-test-br/anch-123') == True
    assert has_invalid_symbols('feat-test-b\\ranch-123') == True
    assert has_invalid_symbols('f@eat-tes?t-b\\ranch-12?3') == True
    assert has_invalid_symbols('feat-test-branch-12?3') == True
    assert has_invalid_symbols('!@#$-!@$%-(*&)-)()-123') == True

def test_branch_name():
    """
    test branch name checking function
    """
    assert has_branch_name_error('Feat-test-branch-123') == True
    assert has_branch_name_error('feat-test-branch123') == True
    assert has_branch_name_error('feat-test-branch-') == True
    assert has_branch_name_error('fe@t-test-branch-123') == True
    assert has_branch_name_error('feat-test-branch-123') == False
    assert has_branch_name_error('upgrade-ubuntu-18-304') == False
