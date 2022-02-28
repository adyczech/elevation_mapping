# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the GTest Action."""

import os

from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
import launch_testing
from launch.events import Shutdown

from launch_testing.actions import GTest

def generate_test_description():
    test_path = os.path.join(
        get_package_prefix("elevation_mapping"),
        'test',
        'test_postprocessor_pool'
    )
    param_file = os.path.join(
        get_package_prefix("elevation_mapping"),
        'test',
        'postprocessor_pipeline.yaml'
    )
    print(test_path)
    # cmd = test_path + ' --ros-args --params-file postprocessor_pipeline.yaml'
    # cmd = [test_path, '--ros-args', '--params-file',  param_file]
    ld = LaunchDescription([
        GTest(path=test_path, timeout=5.0, on_exit=[EmitEvent(event=Shutdown())]),
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ])
    # ls = LaunchService()
    # ls.include_launch_description(ld)

    
    # assert 0 == ls.run()

    return ld


# def launch_gtest(test_path):
#     """Launch a gtest."""
#     cmd = test_path + ['--ros-args', '--params-file', 'postprocessor_pipeline.yaml']

#     ld = LaunchDescription([
#         GTest(path=str(cmd), timeout=5.0, on_exit=[EmitEvent(event=Shutdown())])
#     ])
#     ls = LaunchService()
#     ls.include_launch_description(ld)
#     assert 0 == ls.run()


# def test_gtest_postprocessor():
#     """Test running a non-locking gtest with timeout."""
#     launch_gtest('.')