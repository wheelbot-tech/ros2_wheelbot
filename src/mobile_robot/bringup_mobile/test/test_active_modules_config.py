import importlib.util
from pathlib import Path

import pytest


LAUNCH_FILE = (
    Path(__file__).parents[1] / "launch" / "control_Layer_serial_launch.py"
)
SPEC = importlib.util.spec_from_file_location(
    "control_layer_serial_launch", LAUNCH_FILE
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_derives_active_wheels_from_modules():
    assert MODULE._active_wheels_from_modules("fr_, RL") == [
        "front_right",
        "rear_left",
    ]


@pytest.mark.parametrize(
    "active_modules",
    ["", "FR,unknown", "FR,fr_"],
)
def test_rejects_invalid_active_modules(active_modules):
    with pytest.raises(RuntimeError):
        MODULE._active_wheels_from_modules(active_modules)


@pytest.mark.parametrize(
    'node_name',
    [
        'swerve_controller',
        '/robot_1/swerve_controller',
        '/**/swerve_controller',
    ],
)
def test_finds_swerve_controller_parameters(node_name):
    parameters = {'active_wheels': ['front_right', 'rear_left']}
    controllers = {
        '/robot_1/controller_manager': {'ros__parameters': {}},
        node_name: {'ros__parameters': parameters},
    }

    assert MODULE._find_swerve_controller_parameters(controllers) is parameters


def test_rejects_ambiguous_swerve_controller_parameters():
    controllers = {
        'swerve_controller': {'ros__parameters': {}},
        '/robot_1/swerve_controller': {'ros__parameters': {}},
    }

    with pytest.raises(RuntimeError, match='multiple'):
        MODULE._find_swerve_controller_parameters(controllers)
