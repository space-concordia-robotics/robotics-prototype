import branch_name_verification_hook as branch_name_verification_hook
from branch_name_verification_hook import *

excluded_branches = {'test-1', 'sdfsdfs', '!@dvsfosk*&*&^fs4943'}

def test_is_excluded_branch():
    """
    test excluded branches checking
    """
    assert not is_excluded_branch('test', excluded_branches)
    assert is_excluded_branch('test-1', excluded_branches)
    assert is_excluded_branch('!@dvsfosk*&*&^fs4943', excluded_branches)

def test_get_issue_num():
    """
    test get issue number function
    """
    assert get_issue_num('upgrade-ubuntu-18-304') == '304'
    assert get_issue_num('upgrade-ubuntu-18') == '18'
    assert get_issue_num('304-upgrade-ubuntu-18') == '18'
    assert get_issue_num('----------------------18') == '18'

def test_has_issue_num():
    """
    test issue number checking
    """
    assert has_issue_num('feat-test-branch-123')
    assert not has_issue_num('feat-test-branch')
    assert has_issue_num('----------------------18')

def test_has_invalid_symbols():
    """
    test invalid symbols checking
    """
    assert not has_invalid_symbols('feat-test-branch-123')
    assert has_invalid_symbols('feat-test-br/anch-123')
    assert has_invalid_symbols('feat-test-b\\ranch-123')
    assert has_invalid_symbols('f@eat-tes?t-b\\ranch-12?3')
    assert has_invalid_symbols('feat-test-branch-12?3')
    assert has_invalid_symbols('!@#$-!@$%-(*&)-)()-123')
    assert not has_invalid_symbols('----------------------18')

def test_has_double_hyphen():
    """
    test double hyphen checking
    """
    assert not has_double_hyphen('feat-test-branch-123')
    assert has_double_hyphen('----------------------18')
    assert has_double_hyphen('feat---------18')
    assert has_double_hyphen('feat--test-branch-123')

def test_has_branch_name_error():
    """
    test branch name checking function
    """
    assert has_branch_name_error('Feat-test-branch-123')
    assert has_branch_name_error('feat-test-branch123')
    assert has_branch_name_error('feat-test-branch-')
    assert has_branch_name_error('fe@t-test-branch-123')
    assert not has_branch_name_error('feat-test-branch-123')
    assert not has_branch_name_error('upgrade-ubuntu-18-304')
    assert has_branch_name_error('----------------------18')
    assert has_branch_name_error('feat--test-branch-123')
