import xml.etree.ElementTree as ET
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict

@dataclass
class TestCase:
    name: str
    classname: str
    time: float
    timestamp: str
    status: str = ""
    result: str = ""
    failure_message: str = ""
    failure_type: str = ""
    properties: Dict[str, str] = field(default_factory=dict)

@dataclass
class TestSuite:
    name: str
    tests: int
    failures: int
    errors: int
    disabled: int
    time: float
    timestamp: str
    testcases: List[TestCase] = field(default_factory=list)

@dataclass
class TestResult:
    total_tests: int
    total_failures: int
    total_errors: int
    total_disabled: int
    total_time: float
    timestamp: str
    name: str
    suites: List[TestSuite] = field(default_factory=list)

def parse_gtest_xmls(file_paths: List[Path]) -> TestResult:
    all_suites = []
    total_tests = total_failures = total_errors = total_disabled = 0
    total_time = 0.0
    global_timestamp = ""
    global_name = "CombinedTestResult"

    for path in file_paths:
        tree = ET.parse(path)
        root = tree.getroot()

        if not global_timestamp:
            global_timestamp = root.attrib.get("timestamp", "")

        for suite_elem in root.findall("testsuite"):
            suite = TestSuite(
                name=suite_elem.attrib.get("name", "UnnamedSuite"),
                tests=int(suite_elem.attrib.get("tests", 0)),
                failures=int(suite_elem.attrib.get("failures", 0)),
                errors=int(suite_elem.attrib.get("errors", 0)),
                disabled=int(suite_elem.attrib.get("disabled", 0)),
                time=float(suite_elem.attrib.get("time", 0.0)),
                timestamp=suite_elem.attrib.get("timestamp", ""),
            )

            for case_elem in suite_elem.findall("testcase"):
              case = TestCase(
                  name=case_elem.attrib.get("name", "UnnamedTest"),
                  classname=case_elem.attrib.get("classname", ""),
                  time=float(case_elem.attrib.get("time", 0.0)),
                  timestamp=case_elem.attrib.get("timestamp", ""),
                  status=case_elem.attrib.get("status", ""),
                  result=case_elem.attrib.get("result", "")
              )

              # Check for failure
              failure_elem = case_elem.find("failure")
              if failure_elem is not None:
                  case.failure_type = failure_elem.attrib.get("type", "")
                  case.failure_message = failure_elem.text or ""

              # Parse extra properties if available
              props_elem = case_elem.find("properties")
              if props_elem is not None:
                  for prop in props_elem.findall("property"):
                      key = prop.attrib.get("name")
                      val = prop.attrib.get("value")
                      if key and val:
                          case.properties[key] = val

              suite.testcases.append(case)


            all_suites.append(suite)

            # Aggregate totals
            total_tests += suite.tests
            total_failures += suite.failures
            total_errors += suite.errors
            total_disabled += suite.disabled
            total_time += suite.time

    return TestResult(
        total_tests=total_tests,
        total_failures=total_failures,
        total_errors=total_errors,
        total_disabled=total_disabled,
        total_time=total_time,
        timestamp=global_timestamp,
        name=global_name,
        suites=all_suites
    )
