import subprocess
import re
from pathlib import Path

# Import the parser from the other file
from test_result_parser import parse_gtest_xmls

def get_workspace_root():
    # Start from where the script is run
    current = Path.cwd()

    for parent in [current] + list(current.parents):
        if (parent / "src").exists():
            print(parent)
            return parent

    raise RuntimeError("Could not locate workspace root (no 'src/' directory found)")


def get_gtest_xml_paths():
    workspace_root = get_workspace_root()

    try:
        result = subprocess.run(
            ["colcon", "test-result", "--all"],
            cwd=workspace_root,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False
        )

        # Match lines that end in .gtest.xml
        gtest_files = re.findall(r'^(.*?\.gtest\.xml):', result.stdout, re.MULTILINE)

        # Convert to absolute paths
        gtest_files_abs = [workspace_root / Path(path) for path in gtest_files]

        print("Found .gtest.xml files:")
        for path in gtest_files_abs:
            print(path)

        return gtest_files_abs

    except subprocess.CalledProcessError as e:
        print("Error running colcon command:", e)
        print("stderr:", e.stderr)
        return []

if __name__ == "__main__":
    files = get_gtest_xml_paths()

    if files:
        test_result = parse_gtest_xmls(files)

        print(f"\n--- Combined Test Report ---")
        print(f"Total Tests: {test_result.total_tests}")
        print(f"Failures: {test_result.total_failures}")
        print(f"Errors: {test_result.total_errors}")
        print(f"Disabled: {test_result.total_disabled}")
        print(f"Total Time: {test_result.total_time:.2f}s")
        print(f"Suites: {len(test_result.suites)}")

        for suite in test_result.suites:
            print(f"\nSuite: {suite.name} ({suite.tests} tests, {suite.failures} failures, {suite.errors} errors)")

            for case in suite.testcases:
                print(f" - Test: {case.name} ({case.result}) [{case.classname}]")
                print(f"   Time: {case.time:.3f}s  Status: {case.status}")

                # Print properties if any
                if case.properties:
                    print(f"   Properties:")
                    for key, val in case.properties.items():
                        print(f"     • {key}: {val}")

                # Show failure details if present
                if case.failure_message:
                    print(f"   ❌ FAILURE:")
                    if case.failure_type:
                        print(f"     Type: {case.failure_type}")
                    print(f"     Message: {case.failure_message.strip()}")
