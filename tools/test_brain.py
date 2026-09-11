#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "tools" / "brain.py"
LAUNCHER = ROOT / "tools" / "brain"
ROBOT_CONFIG = ROOT / "src" / "robot-config.cpp"

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location("brain", CLI)
brain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(brain)


class FakeTransport:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.writes = []
        self.closed = False

    def write(self, text):
        self.writes.append(text)

    def readline(self, _deadline):
        response = next(self.responses, None)
        if isinstance(response, BaseException):
            raise response
        return response

    def close(self):
        self.closed = True


class BrainCliTests(unittest.TestCase):
    def run_cli(self, *args, input_text=None, env=None):
        child_env = os.environ.copy()
        if env:
            child_env.update(env)
        return subprocess.run(
            [sys.executable, str(CLI), *args],
            cwd=ROOT,
            input=input_text,
            text=True,
            capture_output=True,
            env=child_env,
            timeout=5,
        )

    def test_mock_pose_query_prints_reply_and_terminator(self):
        result = self.run_cli("--mock", "pose")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout,
            "[ok] pose x=12.40 y=31.08 th=-89.70\n[end] pose\n",
        )

    def test_mock_gains_prints_both_axes(self):
        result = self.run_cli("--mock", "gains")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[ok] gains lat kp=16.8000 ki=0.1250 kd=118.0000\n", result.stdout)
        self.assertIn("[ok] gains ang kp=3.5000 ki=0.2500 kd=12.0000\n", result.stdout)
        self.assertTrue(result.stdout.endswith("[end] gains\n"))

    def test_mock_motion_waits_for_matching_terminal_event(self):
        result = self.run_cli("--mock", "drive", "24")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[end] drive\n", result.stdout)
        self.assertIn("[evt] drive done reason=none x=0.00 y=24.00 th=0.00\n", result.stdout)

    def test_mock_motion_no_wait_stops_at_end(self):
        result = self.run_cli("--mock", "drive", "24", "--no-wait")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(result.stdout.endswith("[end] drive\n"))
        self.assertNotIn("[evt]", result.stdout)

    def test_mock_stream_normalizes_seconds_and_waits_for_expiry(self):
        result = self.run_cli("--mock", "stream", "pid", "20", "--for", "8s")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[ok] stream pid 20 8000\n", result.stdout)
        self.assertIn("[tlm] pid t=20 lat.err=2.31 lat.out=54.02 ang.err=0.40 ang.out=3.10\n", result.stdout)
        self.assertTrue(result.stdout.endswith("[evt] stream expired\n"))

    def test_unknown_robot_command_returns_one(self):
        result = self.run_cli("--mock", "frobnicate")
        self.assertEqual(result.returncode, 1)
        self.assertIn("[err] unknown command", result.stdout)
        self.assertTrue(result.stdout.endswith("[end] frobnicate\n"))

    def test_robot_error_returns_one(self):
        result = self.run_cli("--mock", "pose", env={"BRAIN_MOCK_SCENARIO": "error"})
        self.assertEqual(result.returncode, 1)
        self.assertIn("[err] fixture failure", result.stdout)

    def test_timeout_returns_two(self):
        result = self.run_cli("--mock", "pose", env={"BRAIN_MOCK_SCENARIO": "timeout"})
        self.assertEqual(result.returncode, 2)
        self.assertIn("timed out", result.stderr)

    def test_no_device_returns_two(self):
        with tempfile.TemporaryDirectory() as temp:
            Path(temp, "serial.py").write_text(
                "class SerialException(Exception): pass\n"
                "class Serial:\n"
                "    def __init__(self, *args, **kwargs):\n"
                "        raise SerialException('fixture missing device')\n"
            )
            result = self.run_cli(
                "--port",
                "/dev/fixture-missing",
                "pose",
                env={"PYTHONPATH": temp},
            )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("fixture missing device", result.stderr)

    def test_missing_command_is_usage_error_three(self):
        result = self.run_cli()
        self.assertEqual(result.returncode, 3)
        self.assertIn("usage:", result.stderr.lower())

    def test_malformed_for_is_usage_error_three(self):
        for value in ("8", "0s", "1.5s", "banana", "61s"):
            with self.subTest(value=value):
                result = self.run_cli("--mock", "stream", "pid", "20", "--for", value)
                self.assertEqual(result.returncode, 3)
                self.assertIn("--for", result.stderr)

    def test_unrecognized_host_option_is_usage_error_three(self):
        result = self.run_cli("--mock", "pose", "--wat")
        self.assertEqual(result.returncode, 3)
        self.assertIn("usage:", result.stderr.lower())

    def test_for_is_rejected_outside_bounded_stream(self):
        for args in (
            ("pose", "--for", "8s"),
            ("stream", "off", "--for", "8s"),
        ):
            with self.subTest(args=args):
                result = self.run_cli("--mock", *args)
                self.assertEqual(result.returncode, 3)
                self.assertIn("--for", result.stderr)

    def test_non_ascii_command_is_usage_error_three(self):
        result = self.run_cli("--mock", "run", "qualé")
        self.assertEqual(result.returncode, 3)
        self.assertIn("ASCII", result.stderr)

    def test_command_over_wire_limit_is_usage_error_three(self):
        result = self.run_cli("--mock", "x" * 128)
        self.assertEqual(result.returncode, 3)
        self.assertIn("128 bytes", result.stderr)

    def test_ctrl_c_while_waiting_sends_stop_on_same_transport(self):
        transport = FakeTransport(
            [
                "[ok] drive 24.00 accepted",
                "[end] drive",
                KeyboardInterrupt(),
            ]
        )
        output = io.StringIO()
        errors = io.StringIO()
        with contextlib.redirect_stderr(errors):
            code = brain.exchange(transport, "drive 24", wait=True, output=output)
        self.assertEqual(code, 1)
        self.assertEqual(transport.writes, ["drive 24\n", "stop\n"])
        self.assertIn("sent stop", errors.getvalue())


class ProtocolParsingTests(unittest.TestCase):
    def test_extract_tag_accepts_normal_line(self):
        self.assertEqual(brain.extract_tag("[ok] pose x=1"), "[ok] pose x=1")

    def test_extract_tag_keeps_content_from_first_tag_at_any_offset(self):
        self.assertEqual(
            brain.extract_tag("lateralOut: 5.0[end] moveto"),
            "[end] moveto",
        )
        self.assertEqual(
            brain.extract_tag("lateralOut: 5.0[evt] moveto done reason=none"),
            "[evt] moveto done reason=none",
        )

    def test_extract_tag_ignores_untagged_noise(self):
        self.assertIsNone(brain.extract_tag("moveToPoint exited: settled"))

    def test_matching_end_is_required(self):
        transport = FakeTransport(
            [
                "[end] unrelated",
                "debug text[end] pose",
            ]
        )
        output = io.StringIO()
        code = brain.exchange(transport, "pose", wait=True, output=output)
        self.assertEqual(code, 0)
        self.assertEqual(transport.writes, ["pose\n"])
        self.assertTrue(output.getvalue().endswith("[end] pose\n"))

    def test_stopped_motion_event_returns_one(self):
        transport = FakeTransport(
            [
                "[ok] drive 24.00 accepted",
                "[end] drive",
                "[evt] drive stopped reason=controller x=1.00 y=2.00 th=0.00",
            ]
        )
        code = brain.exchange(transport, "drive 24", wait=True, output=io.StringIO())
        self.assertEqual(code, 1)

    def test_exchange_sends_exactly_one_newline_terminated_command(self):
        transport = FakeTransport(["[ok] pose x=0 y=0 th=0", "[end] pose"])
        code = brain.exchange(transport, "pose", wait=True, output=io.StringIO())
        self.assertEqual(code, 0)
        self.assertEqual(transport.writes, ["pose\n"])


class SerialTransportTests(unittest.TestCase):
    class FakeSerialPort:
        opened = []

        def __init__(self, port, baudrate, timeout, write_timeout):
            self.port = port
            self.responses = []
            self.writes = []
            self.closed = False
            self.__class__.opened.append(
                (port, baudrate, timeout, write_timeout, self)
            )

        def reset_input_buffer(self):
            pass

        def write(self, payload):
            text = payload.decode("ascii")
            self.writes.append(text)
            if text == "ver\n" and self.port.endswith("good"):
                self.responses.extend(
                    [
                        b"boot noise[ok] ver built=fixture\n",
                        b"[end] ver\n",
                    ]
                )
            elif text == "pose\n":
                self.responses.extend(
                    [
                        b"[ok] pose x=0.00 y=0.00 th=0.00\n",
                        b"[end] pose\n",
                    ]
                )
            return len(payload)

        def flush(self):
            pass

        def readline(self):
            return self.responses.pop(0) if self.responses else b""

        def close(self):
            self.closed = True

    def setUp(self):
        self.FakeSerialPort.opened = []
        self.serial_module = types.SimpleNamespace(
            Serial=self.FakeSerialPort,
            SerialException=OSError,
        )

    def test_auto_detect_probes_candidates_with_ver_and_keeps_answering_port(self):
        with mock.patch.dict(sys.modules, {"serial": self.serial_module}):
            with mock.patch.object(
                brain.glob,
                "glob",
                return_value=["/dev/cu.usbmodem-bad", "/dev/cu.usbmodem-good"],
            ):
                transport = brain.open_transport(None)
                output = io.StringIO()
                code = brain.exchange(transport, "pose", output=output)
                transport.close()

        self.assertEqual(code, 0)
        self.assertEqual(
            [(entry[0], entry[1]) for entry in self.FakeSerialPort.opened],
            [
                ("/dev/cu.usbmodem-bad", 115200),
                ("/dev/cu.usbmodem-good", 115200),
            ],
        )
        bad = self.FakeSerialPort.opened[0][4]
        good = self.FakeSerialPort.opened[1][4]
        self.assertEqual(bad.writes, ["ver\n"])
        self.assertTrue(bad.closed)
        self.assertEqual(good.writes, ["ver\n", "pose\n"])
        self.assertTrue(good.closed)
        self.assertIn("[end] pose", output.getvalue())

    def test_v5_port_takes_precedence_over_auto_detection(self):
        with mock.patch.dict(
            sys.modules,
            {"serial": self.serial_module},
        ), mock.patch.dict(os.environ, {"V5_PORT": "/dev/cu.fixture-env"}), mock.patch.object(
            brain.glob,
            "glob",
            side_effect=AssertionError("auto-detect must not run"),
        ):
            transport = brain.open_transport(None)
            transport.close()

        self.assertEqual(
            [entry[0] for entry in self.FakeSerialPort.opened],
            ["/dev/cu.fixture-env"],
        )

    def test_explicit_port_takes_precedence_over_v5_port(self):
        with mock.patch.dict(
            sys.modules,
            {"serial": self.serial_module},
        ), mock.patch.dict(os.environ, {"V5_PORT": "/dev/cu.fixture-env"}):
            transport = brain.open_transport("/dev/cu.fixture-cli")
            transport.close()

        self.assertEqual(
            [entry[0] for entry in self.FakeSerialPort.opened],
            ["/dev/cu.fixture-cli"],
        )


class GainSyncTests(unittest.TestCase):
    def run_cli(self, *args, env=None):
        child_env = os.environ.copy()
        if env:
            child_env.update(env)
        return subprocess.run(
            [sys.executable, str(CLI), *args],
            cwd=ROOT,
            text=True,
            capture_output=True,
            env=child_env,
            timeout=5,
        )

    @staticmethod
    def controller_gain_lines(text):
        values = {}
        lines = text.splitlines()
        for controller in ("linearController", "angularController"):
            start = next(i for i, line in enumerate(lines) if controller + "(" in line)
            gain_lines = lines[start : start + 3]
            numbers = []
            normalized = []
            for line in gain_lines:
                code, comment = line.split("//", 1)
                match = re.search(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", code)
                if match:
                    numbers.append(match.group(0))
                    code = code[: match.start()] + "<GAIN>" + code[match.end() :]
                normalized.append(code + "//" + comment)
            values[controller] = (numbers, normalized)
        return values

    @staticmethod
    def mask_controller_gain_tokens(text):
        lines = text.splitlines(keepends=True)
        for controller in ("linearController", "angularController"):
            start = next(i for i, line in enumerate(lines) if controller + "(" in line)
            for index in range(start, start + 3):
                code, comment = lines[index].split("//", 1)
                code = re.sub(
                    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?",
                    "<GAIN>",
                    code,
                    count=1,
                )
                lines[index] = code + "//" + comment
        return "".join(lines)

    def test_sync_gains_rewrites_only_six_numeric_tokens(self):
        original = ROBOT_CONFIG.read_text()
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp, "robot-config.cpp")
            config.write_text(original)
            result = self.run_cli(
                "--mock",
                "sync-gains",
                "--yes",
                env={"BRAIN_CONFIG": str(config)},
            )
            updated = config.read_text()

        self.assertEqual(result.returncode, 0, result.stderr)
        before = self.controller_gain_lines(original)
        after = self.controller_gain_lines(updated)
        self.assertEqual(
            after["linearController"][0],
            ["16.8", "0.125", "118"],
        )
        self.assertEqual(
            after["angularController"][0],
            ["3.5", "0.25", "12"],
        )
        self.assertEqual(before["linearController"][1], after["linearController"][1])
        self.assertEqual(before["angularController"][1], after["angularController"][1])

        self.assertEqual(
            self.mask_controller_gain_tokens(original),
            self.mask_controller_gain_tokens(updated),
        )
        self.assertIn("--- ", result.stdout)
        self.assertIn("+++ ", result.stdout)
        self.assertIn("2 controllers checked, 6 gains changed. run `make`.", result.stdout)

    def test_sync_gains_requires_yes_when_stdin_is_not_tty(self):
        original = ROBOT_CONFIG.read_text()
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp, "robot-config.cpp")
            config.write_text(original)
            result = self.run_cli(
                "--mock",
                "sync-gains",
                env={"BRAIN_CONFIG": str(config)},
            )
            after = config.read_text()
        self.assertEqual(result.returncode, 3)
        self.assertEqual(after, original)
        self.assertIn("--yes", result.stderr)

    def test_sync_is_transactional_when_second_block_is_missing(self):
        original = ROBOT_CONFIG.read_text().replace(
            "lemlib::ControllerSettings angularController",
            "lemlib::ControllerSettings missingAngularController",
            1,
        )
        with self.assertRaises(brain.SyncError):
            brain.rewrite_gains_text(
                original,
                {
                    "lat": (16.8, 0.125, 118.0),
                    "ang": (3.5, 0.25, 12.0),
                },
            )
        self.assertIn("linearController", original)
        self.assertNotIn("lemlib::ControllerSettings angularController", original)

    def test_dump_formats_controller_settings_locally(self):
        result = self.run_cli("--mock", "dump")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("lemlib::ControllerSettings linearController(", result.stdout)
        self.assertIn("16.8, // proportional gain (kP)", result.stdout)
        self.assertIn("lemlib::ControllerSettings angularController(", result.stdout)
        self.assertIn("12 // derivative gain (kD)", result.stdout)
        self.assertNotIn("[ok]", result.stdout)


class LauncherTests(unittest.TestCase):
    def test_launcher_uses_v5_python_candidate_that_has_pyserial(self):
        with tempfile.TemporaryDirectory() as temp:
            wrapper = Path(temp, "python-with-serial")
            wrapper.write_text(
                "#!/bin/sh\n"
                "if [ \"$1\" = \"-c\" ]; then exit 0; fi\n"
                f"exec {sys.executable} \"$@\"\n"
            )
            wrapper.chmod(wrapper.stat().st_mode | stat.S_IXUSR)
            env = os.environ.copy()
            env["V5_PYTHON"] = str(wrapper)
            result = subprocess.run(
                ["/bin/sh", str(LAUNCHER), "--mock", "pose"],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=env,
                timeout=5,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[end] pose", result.stdout)

    def test_launcher_reports_actionable_error_when_no_candidate_has_pyserial(self):
        with tempfile.TemporaryDirectory() as temp:
            python3 = Path(temp, "python3")
            python3.write_text("#!/bin/sh\nexit 1\n")
            python3.chmod(python3.stat().st_mode | stat.S_IXUSR)
            env = os.environ.copy()
            env.pop("V5_PYTHON", None)
            env["PATH"] = temp
            result = subprocess.run(
                ["/bin/sh", str(LAUNCHER), "--mock", "pose"],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=env,
                timeout=5,
            )
        self.assertEqual(result.returncode, 3)
        self.assertIn("pip install pyserial", result.stderr)

    def test_launcher_falls_back_to_pros_shebang_interpreter(self):
        with tempfile.TemporaryDirectory() as temp:
            python3 = Path(temp, "python3")
            python3.write_text("#!/bin/sh\nexit 1\n")
            python3.chmod(python3.stat().st_mode | stat.S_IXUSR)

            pros_python = Path(temp, "pros-python")
            pros_python.write_text(
                "#!/bin/sh\n"
                "if [ \"$1\" = \"-c\" ]; then exit 0; fi\n"
                f"exec {sys.executable} \"$@\"\n"
            )
            pros_python.chmod(pros_python.stat().st_mode | stat.S_IXUSR)

            pros = Path(temp, "pros")
            pros.write_text(f"#!{pros_python}\n")
            pros.chmod(pros.stat().st_mode | stat.S_IXUSR)

            env = os.environ.copy()
            env.pop("V5_PYTHON", None)
            env["PATH"] = temp
            result = subprocess.run(
                ["/bin/sh", str(LAUNCHER), "--mock", "pose"],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=env,
                timeout=5,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[end] pose", result.stdout)


if __name__ == "__main__":
    unittest.main()
