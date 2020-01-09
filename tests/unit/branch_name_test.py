import branch_name_verification as branch_name_verification
from branch_name_verification import is_excluded_branch, has_upper_case_letters, has_hyphen_seperator, get_issue_num, has_issue_num, has_invalid_symbols, branch_name_error

excluded_branches = {'test-1', 'sdfsdfs', '!@dvsfosk*&*&^fs4943'}

def test_excluded_branches():
    """
    test excluded branches checking
    """
    assert is_excluded_branch('test', excluded_branches) == False
    assert is_excluded_branch('test-1', excluded_branches) == True
    assert is_excluded_branch('!@dvsfosk*&*&^fs4943', excluded_branches) == True

def test_upper_case_letters():
    """
    test upper case letters checking
    """
    assert has_upper_case_letters('feat-test-branch-') == False
    assert has_upper_case_letters('Feat-test-branch-') == True
    assert has_upper_case_letters('feat-test-Branch-') == True
    assert has_upper_case_letters('FEAT-TEST-BRANCH-') == True

def test_hyphen_seperator():
    """
    test hyphen checking
    """
    assert has_hyphen_seperator('feat-test-branch') == False
    assert has_hyphen_seperator('feat-test-branch-') == True

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
    assert branch_name_error('test-1', excluded_branches) == False
    assert branch_name_error('Feat-test-branch-123', excluded_branches) == True
    assert branch_name_error('feat-test-branch123', excluded_branches) == True
    assert branch_name_error('feat-test-branch-', excluded_branches) == True
    assert branch_name_error('fe@t-test-branch-123', excluded_branches) == True
    assert branch_name_error('feat-test-branch-123', excluded_branches) == False
    assert branch_name_error('upgrade-ubuntu-18-304', excluded_branches) == False
