import pytest
from ros_cmdline import parse_ros_cmdline


class TestParseRosCmdline:
    """Test ROS command line parsing functionality."""

    @pytest.mark.unit
    def test_basic_command_no_ros_args(self):
        """Test parsing a basic command with no ROS args."""
        cmdline = ["ros2", "run", "pkg", "node", "arg1", "arg2"]
        result = parse_ros_cmdline(cmdline)

        assert result.command == "ros2"
        assert result.user_args == ["run", "pkg", "node", "arg1", "arg2"]
        assert result.remaps == {}
        assert result.params == {}

    @pytest.mark.unit
    def test_command_with_remaps(self):
        """Test parsing command with remaps."""
        cmdline = ["cmd", "arg", "--ros-args", "-r", "src:=tgt", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.remaps == {"src": "tgt"}
        assert result.user_args == ["arg"]

    @pytest.mark.unit
    def test_command_with_params(self):
        """Test parsing command with parameters."""
        cmdline = ["cmd", "--ros-args", "-p", "param:=value", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.params == {"param": "value"}

    @pytest.mark.unit
    def test_command_with_params_file(self):
        """Test parsing command with parameter file."""
        cmdline = ["cmd", "--ros-args", "--params-file", "/path/to/params.yaml", "--"]
        result = parse_ros_cmdline(cmdline)

        assert "/path/to/params.yaml" in result.params_files

    @pytest.mark.unit
    def test_command_with_log_level(self):
        """Test parsing command with log level."""
        cmdline = ["cmd", "--ros-args", "--log-level", "debug", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.log_level == "debug"

    @pytest.mark.unit
    def test_multiple_ros_args_sections(self):
        """Test parsing command with multiple --ros-args sections."""
        cmdline = [
            "cmd",
            "user1",
            "--ros-args",
            "-p",
            "p1:=v1",
            "--",
            "user2",
            "--ros-args",
            "-r",
            "a:=b",
            "--",
        ]
        result = parse_ros_cmdline(cmdline)

        assert result.user_args == ["user1", "user2"]
        assert result.params == {"p1": "v1"}
        assert result.remaps == {"a": "b"}

    @pytest.mark.unit
    def test_mixed_user_and_ros_args(self):
        """Test parsing mixed user and ROS arguments."""
        cmdline = ["cmd", "before", "--ros-args", "-p", "x:=1", "--", "after"]
        result = parse_ros_cmdline(cmdline)

        assert "before" in result.user_args
        assert "after" in result.user_args
        assert result.params == {"x": "1"}

    @pytest.mark.unit
    def test_enable_rosout_logs(self):
        """Test parsing enable rosout logs flag."""
        cmdline = ["cmd", "--ros-args", "--enable-rosout-logs", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.enable_rosout_logs is True

    @pytest.mark.unit
    def test_disable_rosout_logs(self):
        """Test parsing disable rosout logs flag."""
        cmdline = ["cmd", "--ros-args", "--disable-rosout-logs", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.enable_rosout_logs is False

    @pytest.mark.unit
    def test_enable_stdout_logs(self):
        """Test parsing enable stdout logs flag."""
        cmdline = ["cmd", "--ros-args", "--enable-stdout-logs", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.enable_stdout_logs is True

    @pytest.mark.unit
    def test_disable_stdout_logs(self):
        """Test parsing disable stdout logs flag."""
        cmdline = ["cmd", "--ros-args", "--disable-stdout-logs", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.enable_stdout_logs is False

    @pytest.mark.unit
    def test_enclave_option(self):
        """Test parsing enclave option."""
        cmdline = ["cmd", "--ros-args", "-e", "/my/enclave", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.enclave == "/my/enclave"

    @pytest.mark.unit
    def test_long_form_remap(self):
        """Test parsing long-form remap option."""
        cmdline = ["cmd", "--ros-args", "--remap", "src:=tgt", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.remaps == {"src": "tgt"}

    @pytest.mark.unit
    def test_long_form_param(self):
        """Test parsing long-form param option."""
        cmdline = ["cmd", "--ros-args", "--param", "name:=val", "--"]
        result = parse_ros_cmdline(cmdline)

        assert result.params == {"name": "val"}

    @pytest.mark.unit
    def test_multiple_params_and_remaps(self):
        """Test parsing multiple parameters and remaps."""
        cmdline = [
            "cmd",
            "--ros-args",
            "-p",
            "p1:=v1",
            "-p",
            "p2:=v2",
            "-r",
            "r1:=t1",
            "-r",
            "r2:=t2",
            "--",
        ]
        result = parse_ros_cmdline(cmdline)

        assert result.params == {"p1": "v1", "p2": "v2"}
        assert result.remaps == {"r1": "t1", "r2": "t2"}
