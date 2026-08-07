#!/usr/bin/env python3
"""Catch the mistakes that break a build on the car but pass a quick eyeball.

    python3 tools/preflight.py

Everything here is a static check with no ROS and no build. It exists because
each of these has cost a round trip to the Jetson and back:

  * a package.xml that is well-formed XML and still illegal (catkin_pkg
    rejects a <depend> repeated as <exec_depend>, and kills the whole
    workspace's rosdep along with it)
  * a launch file using $(var x) that no <arg> or <let> declares
  * a yaml or xacro that stopped parsing

Not a substitute for building. It just makes the trip worth taking.
"""
import glob
import re
import sys
import xml.etree.ElementTree as ET

import yaml

FAILURES = []


def fail(path, msg):
    FAILURES.append(f"{path}: {msg}")


def check_package_xml():
    # <depend> expands to all three; naming it again in any of them is fatal.
    implied = ("build_depend", "build_export_depend", "exec_depend")
    for f in sorted(glob.glob("src/**/package.xml", recursive=True)):
        root = ET.parse(f).getroot()
        generic = {e.text.strip() for e in root.findall("depend")}
        for tag in implied:
            for e in root.findall(tag):
                if e.text.strip() in generic:
                    fail(f, f"<{tag}>{e.text.strip()}</{tag}> is already covered by <depend>")
        for tag in implied + ("depend", "buildtool_depend", "test_depend"):
            names = [e.text.strip() for e in root.findall(tag)]
            for dup in sorted({n for n in names if names.count(n) > 1}):
                fail(f, f"<{tag}>{dup}</{tag}> listed {names.count(dup)} times")


def check_launch_vars():
    for f in sorted(glob.glob("src/**/launch/*.xml", recursive=True)):
        body = re.sub(r"<!--.*?-->", "", open(f).read(), flags=re.S)
        declared = set(re.findall(r'<(?:arg|let)\s+name="([^"]+)"', body))
        for used in sorted(set(re.findall(r"\$\(var ([^)]+)\)", body)) - declared):
            fail(f, f"$(var {used}) has no <arg> or <let>")


def check_parses():
    for f in glob.glob("src/**/*.xml", recursive=True) + glob.glob("src/**/*.xacro", recursive=True):
        try:
            ET.parse(f)
        except Exception as e:
            fail(f, f"XML: {e}")
    for f in glob.glob("src/**/*.yaml", recursive=True):
        try:
            yaml.safe_load(open(f))
        except Exception as e:
            fail(f, f"YAML: {e}")


def main():
    check_package_xml()
    check_launch_vars()
    check_parses()
    if FAILURES:
        print("\n".join(FAILURES))
        print(f"\n{len(FAILURES)} problem(s)")
        return 1
    print("preflight clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
