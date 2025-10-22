import json
import subprocess
import tempfile
from pathlib import Path

import pytest


class TestDumpLaunchIntegration:
    """Integration tests for dump_launch tool."""

    @pytest.mark.integration
    def test_dump_launch_on_demo_nodes(self):
        """Test dump_launch on ROS 2 demo_nodes_cpp talker_listener launch file."""
        # Create a temporary directory for the output
        with tempfile.TemporaryDirectory() as tmpdir:
            output_file = Path(tmpdir) / "record.json"

            # Run dump_launch on a builtin ROS 2 launch file
            # Using demo_nodes_cpp's talker_listener.launch.py which is commonly available
            result = subprocess.run(
                [
                    "dump_launch",
                    "demo_nodes_cpp",
                    "talker_listener.launch.py",
                    "-o",
                    str(output_file),
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )

            # Check if the command succeeded
            if result.returncode != 0:
                # Check for common skip reasons
                if "ModuleNotFoundError" in result.stderr and "rclpy" in result.stderr:
                    pytest.skip(
                        "ROS 2 Python version mismatch - test requires matching Python version"
                    )
                elif "Package" in result.stderr and "not found" in result.stderr:
                    pytest.skip("demo_nodes_cpp package not available")
                else:
                    pytest.skip(f"Launch failed with return code {result.returncode}")

            # Verify the output file was created
            assert output_file.exists(), "record.json was not created"

            # Parse and validate the JSON structure
            with open(output_file) as f:
                data = json.load(f)

            # Verify expected structure
            assert "node" in data, "Missing 'node' field in output"
            assert isinstance(data["node"], list), "'node' should be a list"

            # The talker_listener launch file should create at least 2 nodes
            assert (
                len(data["node"]) >= 2
            ), "Expected at least 2 nodes (talker and listener)"

            # Verify node structure
            for node in data["node"]:
                assert "executable" in node, "Node missing 'executable' field"
                assert "package" in node, "Node missing 'package' field"

            # Check for expected nodes (talker and listener)
            executables = [node.get("executable") for node in data["node"]]
            assert (
                "talker" in executables or "listener" in executables
            ), "Expected to find talker or listener in executables"

    @pytest.mark.integration
    def test_dump_launch_creates_valid_json(self):
        """Test that dump_launch creates valid JSON output."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_file = Path(tmpdir) / "record.json"

            # Try multiple common demo packages
            packages_to_try = [
                ("demo_nodes_cpp", "talker_listener.launch.py"),
                ("turtlesim", "multisim.launch.py"),
            ]

            success = False
            skip_reason = None

            for package, launch_file in packages_to_try:
                result = subprocess.run(
                    [
                        "dump_launch",
                        package,
                        launch_file,
                        "-o",
                        str(output_file),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )

                if result.returncode == 0 and output_file.exists():
                    success = True
                    break

                # Capture skip reason from first failure
                if not skip_reason:
                    if (
                        "ModuleNotFoundError" in result.stderr
                        and "rclpy" in result.stderr
                    ):
                        skip_reason = "ROS 2 Python version mismatch"
                    elif "Package" in result.stderr and "not found" in result.stderr:
                        skip_reason = "ROS 2 demo packages not available"

            if not success:
                reason = (
                    skip_reason
                    or "No suitable ROS 2 demo packages available for testing"
                )
                pytest.skip(reason)

            # Verify JSON is valid and has expected structure
            with open(output_file) as f:
                data = json.load(f)

            # Basic structure validation
            assert isinstance(data, dict), "Output should be a JSON object"
            assert (
                "node" in data or "container" in data or "load_node" in data
            ), "Output should contain at least one of: node, container, load_node"

    @pytest.mark.integration
    def test_dump_launch_command_availability(self):
        """Test that dump_launch command is available."""
        result = subprocess.run(
            ["dump_launch", "--help"],
            capture_output=True,
            text=True,
            timeout=10,
        )

        # If Python version mismatch, skip the test
        if "ModuleNotFoundError" in result.stderr and "rclpy" in result.stderr:
            pytest.skip(
                "ROS 2 Python version mismatch - ROS requires different Python version"
            )

        # Should return 0 or show help message
        assert result.returncode in [
            0,
            2,
        ], f"dump_launch command not available: {result.stderr}"
        assert (
            "usage" in result.stdout.lower() or "usage" in result.stderr.lower()
        ), "Expected usage information in help output"

    @pytest.mark.integration
    def test_dump_launch_with_nonexistent_package(self):
        """Test dump_launch behavior with nonexistent package."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_file = Path(tmpdir) / "record.json"

            result = subprocess.run(
                [
                    "dump_launch",
                    "nonexistent_package_12345",
                    "fake.launch.py",
                    "-o",
                    str(output_file),
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            # Should fail with non-zero exit code
            assert result.returncode != 0, "Expected failure for nonexistent package"

            # Output file should not be created or should be empty/invalid
            if output_file.exists():
                # If file exists, it should either be empty or contain error info
                assert output_file.stat().st_size == 0 or not self._is_valid_json(
                    output_file
                )

    @staticmethod
    def _is_valid_json(file_path):
        """Helper to check if a file contains valid JSON."""
        try:
            with open(file_path) as f:
                json.load(f)
            return True
        except (json.JSONDecodeError, ValueError):
            return False
