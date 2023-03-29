import os
import pytest
import unittest
import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
def generate_test_description():
    TEST_PROC_PATH = os.path.join(
        get_package_share_directory("navigation_controller"),
        "navigation_launch.py",
    )

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    dut_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', TEST_PROC_PATH, 'use_rviz:=False', 'x:=1.16',
             'y:=0.52', 'theta:=1.0'],
    )

    return launch.LaunchDescription(
        [
            dut_process,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"dut_process": dut_process}


# These tests will run concurrently with the dut process.  After all these
# tests are done, the launch system will shut down the processes that
# it started up
class TestNavigation(unittest.TestCase):
    def test_navigator_goal_succeded(self, proc_output):
        # This will match stdout from any process.
        proc_output.assertWaitFor("Navigation succeeded!", timeout=120,
                                  stream="stdout")
