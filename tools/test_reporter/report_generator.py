import sys
import os
from pathlib import Path
import matplotlib.pyplot as plt

def generate_test_report(test_result, output_dir="test_report"):
    output_path = Path(output_dir)
    charts_path = output_path / "charts"
    report_md_path = output_path / "report.md"

    output_path.mkdir(parents=True, exist_ok=True)
    charts_path.mkdir(parents=True, exist_ok=True)

    # === 1. Create a chart (TestCases per Suite) ===
    suite_names = [suite.name for suite in test_result.suites]
    test_counts = [len(suite.testcases) for suite in test_result.suites]

    plt.figure(figsize=(10, 6))
    bars = plt.bar(suite_names, test_counts, color='skyblue')
    plt.title("Test Cases per Suite")
    plt.xlabel("Test Suite")
    plt.ylabel("Number of Test Cases")
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()

    chart_file = charts_path / "test_summary.png"
    plt.savefig(chart_file)
    plt.close()

    # === 2. Build Markdown content ===
    md_lines = []

    # Overall status
    overall_pass = test_result.total_failures == 0 and test_result.total_errors == 0
    overall_icon = "✅" if overall_pass else "❌"

    # Summary
    md_lines.append(f"# {overall_icon} Test Report\n")
    md_lines.append(f"**Total Tests**: {test_result.total_tests}  \n")
    md_lines.append(f"**Failures**: {test_result.total_failures}  \n")
    md_lines.append(f"**Errors**: {test_result.total_errors}  \n")
    md_lines.append(f"**Disabled**: {test_result.total_disabled}  \n")
    md_lines.append(f"**Time**: {test_result.total_time:.2f}s  \n")
    md_lines.append(f"**Timestamp**: {test_result.timestamp}\n")

    # Chart
    md_lines.append("\n---\n")
    md_lines.append("## 📈 Test Distribution\n")
    md_lines.append(f"![Test Case Summary](charts/test_summary.png)\n")

    # Detailed results
    md_lines.append("\n---\n")
    md_lines.append("## 🧪 Detailed Results\n")

    for suite in test_result.suites:
        suite_pass = suite.failures == 0 and suite.errors == 0
        suite_icon = "✅" if suite_pass else "❌"
        md_lines.append(f"\n### {suite_icon} Suite: **{suite.name}**")
        md_lines.append(f"- Tests: {suite.tests}, Failures: {suite.failures}, Errors: {suite.errors}, Time: {suite.time:.2f}s")

        for case in suite.testcases:
            case_pass = case.result == "completed" and not case.failure_message
            result_icon = "✅" if case_pass else "❌"
            md_lines.append(f"  - {result_icon} **{case.name}** ({case.classname}) - {case.time:.2f}s")

            # Check if it's the PerceptionAccuracyTest/0 format
            if case.name.startswith("PerceptionAccuracyTest/"):
                # Extract numeric part of the test case name, e.g., "0"
                test_number = case.name.split("/")[-1]
                md_lines.append(f"    - **Test Number**: {test_number}")
                # Generate Pie Chart for the properties (assuming these are percentages)
                generate_accuracy_pie_charts(case.properties, test_number, charts_path)

                # Add pie chart images to the markdown
                accuracy_properties = {k: v for k, v in case.properties.items() if 'Accuracy' in k}
                for key, val in accuracy_properties.items():
                    md_lines.append(f"    - **{key}**: {val}")
                    md_lines.append(f"      ![Pie Chart for {key}](charts/{key}_{test_number}.png)")

            # Failure message
            if case.failure_message:
                md_lines.append(f"    - **Failure**: **{case.failure_type}**")
                md_lines.append(case.failure_message.strip())

            # Properties as table
            if case.properties:
                md_lines.append("    - **Properties:**")
                md_lines.append("")
                md_lines.append("      | Key | Value |")
                md_lines.append("      |-----|-------|")
                for key, val in case.properties.items():
                    md_lines.append(f"      | {key} | {val} |")
                md_lines.append("")

    # Write to file
    with open(report_md_path, "w") as f:
        f.write("\n".join(md_lines))

    print(f"✅ Markdown report generated: {report_md_path.resolve()}")

def generate_accuracy_pie_charts(properties, test_number, charts_path):
    # If there are any relevant properties (e.g., accuracy metrics), plot them
    accuracy_properties = {k: v for k, v in properties.items() if 'Accuracy' in k}
    for key, value in accuracy_properties.items():
        value = float(value)  # Convert to float for accuracy
        remaining = 1 - value  # Calculate the remainder to complete the pie chart to 1

        # Prepare data for pie chart
        labels = [f"{key} ({value*100:.2f}%)", f"Other ({remaining*100:.2f}%)"]
        sizes = [value, remaining]
        colors = ['#5bf092', '#ff9999']

        plt.figure(figsize=(3, 3))
        plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=140, colors=colors)
        plt.title(f"{key} Breakdown for Test {test_number}")

        chart_file = charts_path / f"{key}_{test_number}.png"
        plt.savefig(chart_file)
        plt.close()

        print(f"  ✅ Pie chart generated: {chart_file.resolve()}")

# Add sibling directory to sys.path for imports
sys.path.append(str(Path(__file__).resolve().parents[1] / "result_procesor"))
from result_reader import get_gtest_xml_paths
from test_result_parser import parse_gtest_xmls

def main():
    print("🚀 Generating test report...")

    xml_files = get_gtest_xml_paths()
    if not xml_files:
        print("❌ No .gtest.xml files found.")
        return

    test_result = parse_gtest_xmls(xml_files)
    generate_test_report(test_result)

if __name__ == "__main__":
    main()
