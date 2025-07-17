#!/usr/bin/env python3
"""
Basicmicro Driver Test Runner

Custom test runner with enhanced reporting, performance tracking, and
support for different test categories. Provides comprehensive test
execution with detailed reporting for development and CI/CD integration.

Usage:
    python test_runner.py [options]
    
Options:
    --unit                Run unit tests only
    --integration         Run integration tests only  
    --performance         Run performance tests only
    --regression          Run regression tests only
    --hardware            Run hardware tests (requires hardware)
    --coverage            Generate coverage report
    --benchmark           Generate performance benchmarks
    --output-dir DIR      Output directory for reports (default: test_results)
    --timeout SECONDS     Test timeout in seconds (default: 300)
    --verbose             Verbose output
    --parallel            Run tests in parallel
    --performance-limits FILE  Custom performance limits file

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import argparse
import sys
import os
import json
import time
import subprocess
from pathlib import Path
from datetime import datetime
import xml.etree.ElementTree as ET


class TestRunner:
    """Enhanced test runner with reporting and performance tracking"""
    
    def __init__(self, output_dir="test_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.test_results = {}
        self.performance_results = {}
        self.start_time = None
        self.end_time = None
        
    def run_test_category(self, category, additional_args=None):
        """Run a specific test category"""
        additional_args = additional_args or []
        
        print(f"\n{'='*60}")
        print(f"Running {category.upper()} tests")
        print(f"{'='*60}")
        
        # Build pytest command
        cmd = ["python3", "-m", "pytest"]
        
        # Add test path based on category
        if category in ['unit', 'integration', 'performance', 'regression', 'hardware']:
            test_path = f"test/{category}/"
            if not Path(test_path).exists():
                print(f"Warning: Test directory {test_path} does not exist")
                return False
            cmd.append(test_path)
        else:
            # Run all tests for 'all' category
            cmd.append("test/")
        
        # Add markers
        if category != 'all':
            cmd.extend(["-m", category])
        
        # Add output formats
        junit_file = self.output_dir / f"{category}_results.xml"
        cmd.extend(["--junitxml", str(junit_file)])
        
        # Add additional arguments
        cmd.extend(additional_args)
        
        # Run tests
        start_time = time.time()
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            end_time = time.time()
            
            # Store results
            self.test_results[category] = {
                'returncode': result.returncode,
                'stdout': result.stdout,
                'stderr': result.stderr,
                'duration': end_time - start_time,
                'junit_file': str(junit_file)
            }
            
            # Print output
            print(result.stdout)
            if result.stderr:
                print("STDERR:", result.stderr)
                
            # Parse JUnit results
            if junit_file.exists():
                self._parse_junit_results(category, junit_file)
            
            return result.returncode == 0
            
        except subprocess.TimeoutExpired:
            print(f"Tests timed out after 300 seconds")
            self.test_results[category] = {
                'returncode': -1,
                'stdout': '',
                'stderr': 'Test execution timed out',
                'duration': 300,
                'junit_file': str(junit_file)
            }
            return False
        except Exception as e:
            print(f"Error running tests: {e}")
            return False
    
    def _parse_junit_results(self, category, junit_file):
        """Parse JUnit XML results"""
        try:
            tree = ET.parse(junit_file)
            root = tree.getroot()
            
            # Extract test statistics
            testsuites = root.findall('.//testsuite')
            total_tests = 0
            total_failures = 0
            total_errors = 0
            total_skipped = 0
            total_time = 0.0
            
            for testsuite in testsuites:
                total_tests += int(testsuite.get('tests', 0))
                total_failures += int(testsuite.get('failures', 0))
                total_errors += int(testsuite.get('errors', 0))
                total_skipped += int(testsuite.get('skipped', 0))
                total_time += float(testsuite.get('time', 0))
            
            self.test_results[category]['statistics'] = {
                'total_tests': total_tests,
                'passed': total_tests - total_failures - total_errors - total_skipped,
                'failed': total_failures,
                'errors': total_errors,
                'skipped': total_skipped,
                'execution_time': total_time
            }
            
        except Exception as e:
            print(f"Warning: Could not parse JUnit results: {e}")
    
    def run_coverage_analysis(self):
        """Run coverage analysis"""
        print(f"\n{'='*60}")
        print("Running Coverage Analysis")
        print(f"{'='*60}")
        
        cmd = [
            "python3", "-m", "pytest",
            "test/unit/", "test/integration/",
            "--cov=basicmicro_driver",
            "--cov-report=html:" + str(self.output_dir / "coverage_html"),
            "--cov-report=xml:" + str(self.output_dir / "coverage.xml"),
            "--cov-report=term"
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            print(result.stdout)
            if result.stderr:
                print("STDERR:", result.stderr)
            return result.returncode == 0
        except Exception as e:
            print(f"Error running coverage analysis: {e}")
            return False
    
    def run_performance_benchmarks(self):
        """Run performance benchmarks"""
        print(f"\n{'='*60}")
        print("Running Performance Benchmarks")
        print(f"{'='*60}")
        
        # Run performance tests with benchmark output
        cmd = [
            "python3", "-m", "pytest",
            "test/performance/",
            "-m", "performance",
            "--benchmark-json=" + str(self.output_dir / "benchmark_results.json"),
            "-v"
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            print(result.stdout)
            if result.stderr:
                print("STDERR:", result.stderr)
            return result.returncode == 0
        except Exception as e:
            print(f"Error running performance benchmarks: {e}")
            return False
    
    def generate_summary_report(self):
        """Generate comprehensive summary report"""
        report_file = self.output_dir / "test_summary.json"
        html_report_file = self.output_dir / "test_summary.html"
        
        # Create JSON summary
        summary = {
            'timestamp': datetime.now().isoformat(),
            'total_duration': self.end_time - self.start_time if self.end_time and self.start_time else 0,
            'test_results': self.test_results,
            'environment': {
                'python_version': sys.version,
                'platform': sys.platform,
                'working_directory': str(Path.cwd())
            }
        }
        
        with open(report_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        # Create HTML summary
        self._generate_html_report(summary, html_report_file)
        
        # Print summary to console
        self._print_summary(summary)
        
        return report_file, html_report_file
    
    def _generate_html_report(self, summary, html_file):
        """Generate HTML test report"""
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Basicmicro Driver Test Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; }}
        .header {{ background-color: #f0f0f0; padding: 20px; border-radius: 5px; }}
        .category {{ margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }}
        .passed {{ background-color: #d4edda; }}
        .failed {{ background-color: #f8d7da; }}
        .stats {{ display: flex; gap: 20px; }}
        .stat {{ text-align: center; padding: 10px; background-color: #e9ecef; border-radius: 3px; }}
        .timestamp {{ color: #666; font-size: 0.9em; }}
        table {{ width: 100%; border-collapse: collapse; margin: 10px 0; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>Basicmicro Driver Test Report</h1>
        <p class="timestamp">Generated: {summary['timestamp']}</p>
        <p>Total Duration: {summary.get('total_duration', 0):.2f} seconds</p>
    </div>
"""
        
        # Add test category results
        for category, results in summary['test_results'].items():
            stats = results.get('statistics', {})
            status_class = 'passed' if results['returncode'] == 0 else 'failed'
            
            html_content += f"""
    <div class="category {status_class}">
        <h2>{category.upper()} Tests</h2>
        <div class="stats">
            <div class="stat">
                <strong>{stats.get('total_tests', 0)}</strong><br>Total
            </div>
            <div class="stat">
                <strong>{stats.get('passed', 0)}</strong><br>Passed
            </div>
            <div class="stat">
                <strong>{stats.get('failed', 0)}</strong><br>Failed
            </div>
            <div class="stat">
                <strong>{stats.get('skipped', 0)}</strong><br>Skipped
            </div>
            <div class="stat">
                <strong>{stats.get('execution_time', 0):.2f}s</strong><br>Duration
            </div>
        </div>
    </div>
"""
        
        html_content += """
</body>
</html>
"""
        
        with open(html_file, 'w') as f:
            f.write(html_content)
    
    def _print_summary(self, summary):
        """Print test summary to console"""
        print(f"\n{'='*60}")
        print("TEST SUMMARY")
        print(f"{'='*60}")
        
        total_tests = 0
        total_passed = 0
        total_failed = 0
        total_skipped = 0
        
        for category, results in summary['test_results'].items():
            stats = results.get('statistics', {})
            print(f"\n{category.upper()}:")
            print(f"  Status: {'PASSED' if results['returncode'] == 0 else 'FAILED'}")
            print(f"  Tests: {stats.get('total_tests', 0)}")
            print(f"  Passed: {stats.get('passed', 0)}")
            print(f"  Failed: {stats.get('failed', 0)}")
            print(f"  Skipped: {stats.get('skipped', 0)}")
            print(f"  Duration: {stats.get('execution_time', 0):.2f}s")
            
            total_tests += stats.get('total_tests', 0)
            total_passed += stats.get('passed', 0)
            total_failed += stats.get('failed', 0)
            total_skipped += stats.get('skipped', 0)
        
        print(f"\nOVERALL:")
        print(f"  Total Tests: {total_tests}")
        print(f"  Passed: {total_passed}")
        print(f"  Failed: {total_failed}")
        print(f"  Skipped: {total_skipped}")
        print(f"  Success Rate: {(total_passed/total_tests*100) if total_tests > 0 else 0:.1f}%")
        print(f"  Total Duration: {summary.get('total_duration', 0):.2f}s")


def main():
    """Main test runner function"""
    parser = argparse.ArgumentParser(description="Basicmicro Driver Test Runner")
    parser.add_argument("--unit", action="store_true", help="Run unit tests only")
    parser.add_argument("--integration", action="store_true", help="Run integration tests only")
    parser.add_argument("--performance", action="store_true", help="Run performance tests only")
    parser.add_argument("--regression", action="store_true", help="Run regression tests only")
    parser.add_argument("--hardware", action="store_true", help="Run hardware tests")
    parser.add_argument("--coverage", action="store_true", help="Generate coverage report")
    parser.add_argument("--benchmark", action="store_true", help="Generate performance benchmarks")
    parser.add_argument("--output-dir", default="test_results", help="Output directory for reports")
    parser.add_argument("--timeout", type=int, default=300, help="Test timeout in seconds")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--parallel", action="store_true", help="Run tests in parallel")
    
    args = parser.parse_args()
    
    # Create test runner
    runner = TestRunner(args.output_dir)
    runner.start_time = time.time()
    
    # Determine which tests to run
    test_categories = []
    if args.unit:
        test_categories.append('unit')
    if args.integration:
        test_categories.append('integration')
    if args.performance:
        test_categories.append('performance')
    if args.regression:
        test_categories.append('regression')
    if args.hardware:
        test_categories.append('hardware')
    
    # If no specific categories, run all
    if not test_categories:
        test_categories = ['unit', 'integration', 'performance', 'regression']
    
    # Build additional pytest arguments
    additional_args = []
    if args.verbose:
        additional_args.append("-v")
    if args.parallel:
        additional_args.extend(["-n", "auto"])
    
    # Run tests
    all_passed = True
    for category in test_categories:
        passed = runner.run_test_category(category, additional_args)
        if not passed:
            all_passed = False
    
    # Run coverage analysis if requested
    if args.coverage:
        runner.run_coverage_analysis()
    
    # Run performance benchmarks if requested
    if args.benchmark:
        runner.run_performance_benchmarks()
    
    # Generate reports
    runner.end_time = time.time()
    report_files = runner.generate_summary_report()
    
    print(f"\nReports generated:")
    for report_file in report_files:
        print(f"  {report_file}")
    
    # Exit with appropriate code
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()