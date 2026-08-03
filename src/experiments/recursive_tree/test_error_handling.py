"""
Error Handling Test Suite for Recursive Tree Configuration Validation

This test suite verifies that validate_branches() correctly rejects invalid
configurations and produces clear, actionable error messages.

Usage:
    uv run src/experiments/recursive_tree/test_error_handling.py

What it tests:
- Duplicate branch IDs
- Missing root branch
- Multiple root branches
- References to unknown parent IDs
- Missing attach_link for non-root branches
- Non-integer attach_link values
- attach_link out of valid range
- Total link count exceeding PhysX limit (64)

All tests verify both that ValueError is raised AND that the error message
contains the expected information.
"""

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from tree_config import validate_branches


# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================

def assert_validation_error(branches: list, expected_msg_fragment: str, test_name: str) -> None:
    """
    Verify that validate_branches() raises ValueError with expected message.
    
    Args:
        branches: Configuration to test
        expected_msg_fragment: String that should appear in error message
        test_name: Name of test (for error reporting)
    
    Raises:
        AssertionError: If validation passes or error message is wrong
    """
    try:
        validate_branches(branches)
        raise AssertionError(
            f"[{test_name}] Expected ValueError but validation passed"
        )
    except ValueError as e:
        error_msg = str(e)
        if expected_msg_fragment not in error_msg:
            raise AssertionError(
                f"[{test_name}] Error message doesn't contain expected fragment.\n"
                f"  Expected fragment: '{expected_msg_fragment}'\n"
                f"  Actual message: '{error_msg}'"
            )
        # Success - error was raised with correct message
        return error_msg


def print_test_result(test_num: int, test_name: str, description: str, error_msg: str) -> None:
    """Print formatted test result."""
    print(f"\nTest {test_num}: {test_name}")
    print(f"  Description: {description}")
    print(f"  ✓ Correctly rejected with error:")
    # Truncate long messages
    msg_lines = error_msg.split('\n')
    for line in msg_lines[:3]:  # Show first 3 lines max
        print(f"    {line}")
    if len(msg_lines) > 3:
        print(f"    ... ({len(msg_lines) - 3} more lines)")


# ==============================================================================
# TEST CASES
# ==============================================================================

def test_duplicate_ids() -> str:
    """Test 1: Duplicate branch IDs should be rejected."""
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "branch",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.005,
            "height": 0.08,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "branch",  # DUPLICATE!
            "parent": "trunk",
            "attach_link": 4,
            "n_links": 2,
            "radius": 0.005,
            "height": 0.08,
            "tilt": 30.0,
            "rot": 90.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "Duplicate branch id: 'branch'",
        "test_duplicate_ids"
    )


def test_no_root() -> str:
    """Test 2: Configuration must have exactly one root branch."""
    branches = [
        {
            "id": "branchA",
            "parent": "trunk",  # References non-existent trunk
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 45.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "No root branch found",
        "test_no_root"
    )


def test_multiple_roots() -> str:
    """Test 3: Only one root branch is allowed."""
    branches = [
        {
            "id": "trunk1",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "trunk2",
            "parent": None,  # Second root!
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "Multiple root branches",
        "test_multiple_roots"
    )


def test_unknown_parent() -> str:
    """Test 4: Parent ID must exist in the configuration."""
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "branch",
            "parent": "nonexistent",  # Unknown parent!
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.005,
            "height": 0.08,
            "tilt": 45.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "references unknown parent 'nonexistent'",
        "test_unknown_parent"
    )


def test_missing_attach_link() -> str:
    """Test 5: Non-root branches must specify attach_link."""
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "branch",
            "parent": "trunk",
            # Missing attach_link!
            "n_links": 3,
            "radius": 0.005,
            "height": 0.08,
            "tilt": 45.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "has a parent but no 'attach_link'",
        "test_missing_attach_link"
    )


def test_attach_link_not_integer() -> str:
    """Test 6: attach_link must be an integer."""
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "branch",
            "parent": "trunk",
            "attach_link": "3",  # String instead of int!
            "n_links": 3,
            "radius": 0.005,
            "height": 0.08,
            "tilt": 45.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "attach_link must be an int",
        "test_attach_link_not_integer"
    )


def test_attach_link_out_of_range() -> str:
    """Test 7: attach_link must be within [1, parent.n_links]."""
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "branch",
            "parent": "trunk",
            "attach_link": 10,  # Out of range! trunk has only 5 links
            "n_links": 3,
            "radius": 0.005,
            "height": 0.08,
            "tilt": 45.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "attach_link=10 is out of range [1, 5]",
        "test_attach_link_out_of_range"
    )


def test_too_many_links() -> str:
    """Test 8: Total link count must not exceed PhysX limit (64)."""
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 65,  # Exceeds limit!
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
        }
    ]
    
    return assert_validation_error(
        branches,
        "Total link count 65 exceeds PhysX articulation limit of 64",
        "test_too_many_links"
    )


# ==============================================================================
# MAIN RUNNER
# ==============================================================================

def main():
    """Run all error handling tests and generate report."""
    print("=" * 80)
    print(" " * 25 + "ERROR HANDLING TEST SUITE")
    print(" " * 20 + "Recursive Tree Configuration Validation")
    print("=" * 80)
    print()
    print("This suite verifies that invalid configurations are rejected with clear")
    print("error messages.")
    print()
    
    test_suite = [
        (1, "duplicate_ids", test_duplicate_ids, 
         "Duplicate branch IDs"),
        (2, "no_root", test_no_root,
         "No root branch defined"),
        (3, "multiple_roots", test_multiple_roots,
         "Multiple root branches"),
        (4, "unknown_parent", test_unknown_parent,
         "Reference to non-existent parent"),
        (5, "missing_attach_link", test_missing_attach_link,
         "Branch without attach_link"),
        (6, "attach_link_not_integer", test_attach_link_not_integer,
         "Non-integer attach_link"),
        (7, "attach_link_out_of_range", test_attach_link_out_of_range,
         "attach_link outside valid range"),
        (8, "too_many_links", test_too_many_links,
         "Total links exceed PhysX limit"),
    ]
    
    results = []
    
    for test_num, test_name, test_func, description in test_suite:
        try:
            error_msg = test_func()
            print_test_result(test_num, test_name, description, error_msg)
            results.append((test_name, True, description))
        except AssertionError as e:
            print(f"\nTest {test_num}: {test_name}")
            print(f"  ❌ FAILED: {e}")
            results.append((test_name, False, description))
        except Exception as e:
            print(f"\nTest {test_num}: {test_name}")
            print(f"  ❌ ERROR: {e}")
            import traceback
            traceback.print_exc()
            results.append((test_name, False, description))
    
    # Final report
    print()
    print("=" * 80)
    print(" " * 30 + "FINAL REPORT")
    print("=" * 80)
    print()
    
    passed = sum(1 for _, p, _ in results if p)
    total = len(results)
    
    print(f"{'Test':<30} {'Status':<10} {'Description':<40}")
    print("-" * 80)
    for name, passed_flag, desc in results:
        status = "✅ PASS" if passed_flag else "❌ FAIL"
        print(f"{name:<30} {status:<10} {desc:<40}")
    
    print("-" * 80)
    print(f"Tests passed: {passed}/{total}")
    print()
    
    if passed == total:
        print("=" * 80)
        print(" " * 25 + "✅ ALL TESTS PASSED ✅")
        print("=" * 80)
        print()
        print("VERDICT: Configuration validation correctly rejects all invalid")
        print("         configurations with clear, actionable error messages.")
        print()
    else:
        print("=" * 80)
        print(" " * 25 + "❌ SOME TESTS FAILED ❌")
        print("=" * 80)
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
