"""Kiem tra chuoi URDF doc duoc bang YAML.

ur_sim_control.launch.py truyen chuoi URDF vao tham so robot_description ma khong boc
ParameterValue(..., value_type=str) nen launch doc chuoi do nhu YAML. Chi can mot dau hai
cham o cuoi dong trong comment la buoc khoi dong chet.
"""
import os
import subprocess

import pytest
import yaml

URDF = os.path.join(os.path.dirname(__file__), "..", "urdf", "ur3e_pen.urdf.xacro")


def test_urdf_doc_duoc_bang_yaml():
    urdf = subprocess.run(
        ["xacro", os.path.abspath(URDF), "ur_type:=ur3e", "name:=ur", "sim_ignition:=true"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout

    try:
        parsed = yaml.safe_load(urdf)
    except yaml.YAMLError as error:
        mark = getattr(error, "problem_mark", None)
        dong = urdf.splitlines()[mark.line] if mark else ""
        pytest.fail(
            "URDF khong doc duoc bang YAML nen bringup se hong. "
            f"Dong {mark.line + 1 if mark else '?'}: {dong.strip()[:120]}"
        )

    assert isinstance(parsed, str), (
        "URDF phai duoc YAML hieu la mot chuoi don; kieu doc duoc la "
        f"{type(parsed).__name__}"
    )
