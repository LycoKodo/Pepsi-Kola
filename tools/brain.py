#!/usr/bin/env python3
"""One-shot host client for the V5 DevBridge wire protocol."""

import difflib
import glob
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time


EXIT_ROBOT_ERROR = 1
EXIT_TRANSPORT_ERROR = 2
EXIT_USAGE_ERROR = 3

TAG_RE = re.compile(r"\[(?:ok|err|end|evt|tlm)\]")
GAIN_RE = re.compile(
    r"^\[ok\] gains (lat|ang) "
    r"kp=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?) "
    r"ki=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?) "
    r"kd=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)$"
)
MOTION_VERBS = {"drive", "turn", "moveto", "run"}


class UsageError(Exception):
    pass


class SyncError(Exception):
    pass


class TransportError(Exception):
    pass


class BrainEnvironmentError(Exception):
    pass


class MockTransport:
    def __init__(self, command):
        scenario = os.environ.get("BRAIN_MOCK_SCENARIO", "")
        if scenario == "timeout":
            responses = []
        elif scenario == "error":
            verb = first_token(command)
            responses = ["[err] fixture failure", "[end] " + verb]
        else:
            responses = mock_responses(command)
        self._responses = iter(responses)

    def write(self, _text):
        pass

    def readline(self, _deadline):
        return next(self._responses, None)

    def close(self):
        pass


def first_token(command):
    tokens = command.split()
    return tokens[0].lower() if tokens else "?"


def extract_tag(line):
    match = TAG_RE.search(line)
    if not match:
        return None
    return line[match.start() :].rstrip("\r\n")


def mock_responses(command):
    tokens = command.split()
    verb = first_token(command)
    fixtures = {
        "pose": [
            "moveToPoint exited: settled",
            "[ok] pose x=12.40 y=31.08 th=-89.70",
            "[end] pose",
        ],
        "gains": [
            "[ok] gains lat kp=16.8000 ki=0.1250 kd=118.0000",
            "[ok] gains ang kp=3.5000 ki=0.2500 kd=12.0000",
            "[end] gains",
        ],
        "drive": [
            "[ok] drive 24.00 accepted",
            "[end] drive",
            "lateralOut: 5.0[evt] drive done reason=none x=0.00 y=24.00 th=0.00",
        ],
        "turn": [
            "[ok] turn 90.00 accepted",
            "[end] turn",
            "[evt] turn done reason=none x=0.00 y=0.00 th=90.00",
        ],
        "moveto": [
            "[ok] moveto accepted",
            "lateralOut: 5.0[end] moveto",
            "angularOut: 1.0[evt] moveto done reason=none x=1.00 y=2.00 th=3.00",
        ],
        "run": [
            "[ok] run qual1 accepted",
            "[end] run",
            "[evt] run qual1 done reason=none x=0.00 y=0.00 th=0.00",
        ],
    }
    if verb == "stream" and len(tokens) >= 2 and tokens[1] != "off":
        return [
            "[ok] " + command,
            "[end] stream",
            "[tlm] pid t=20 lat.err=2.31 lat.out=54.02 ang.err=0.40 ang.out=3.10",
            "[evt] stream expired",
        ]
    if verb == "stream" and tokens[1:] == ["off"]:
        return ["[ok] stream off", "[end] stream"]
    if verb in fixtures:
        return fixtures[verb]
    if verb in {
        "ports",
        "motors",
        "motion",
        "sel",
        "bat",
        "ver",
        "help",
        "set",
        "arm",
        "disarm",
        "stop",
    }:
        return ["[ok] " + command, "[end] " + verb]
    return ["[err] unknown command", "[end] " + verb]


def parse_arguments(argv):
    args = list(argv)
    mock = False
    port = None
    yes = False

    index = 0
    command_args = []
    while index < len(args):
        arg = args[index]
        if arg == "--mock":
            mock = True
        elif arg == "--port":
            index += 1
            if index >= len(args) or args[index].startswith("--"):
                raise UsageError("--port requires a device path")
            port = args[index]
        elif arg == "--yes":
            yes = True
        else:
            command_args.append(arg)
        index += 1

    if not command_args:
        raise UsageError("missing command")

    no_wait = False
    if "--no-wait" in command_args:
        if command_args.count("--no-wait") != 1:
            raise UsageError("--no-wait may be supplied once")
        command_args.remove("--no-wait")
        no_wait = True

    had_for = "--for" in command_args
    if had_for:
        if command_args.count("--for") != 1:
            raise UsageError("--for may be supplied once")
        index = command_args.index("--for")
        if index + 1 >= len(command_args):
            raise UsageError("--for requires a duration such as 8s")
        duration = command_args[index + 1]
        if index + 2 != len(command_args):
            raise UsageError("--for must be the final option")
        match = re.fullmatch(r"([1-9]\d*)s", duration)
        if not match:
            raise UsageError("--for requires whole seconds such as 8s")
        milliseconds = int(match.group(1)) * 1000
        if milliseconds > 60000:
            raise UsageError("--for must be between 1s and 60s")
        command_args[index:] = [str(milliseconds)]

    unknown_options = [arg for arg in command_args if arg.startswith("--")]
    if unknown_options:
        raise UsageError("unknown option: " + unknown_options[0])

    verb = command_args[0].lower()
    if no_wait and verb not in MOTION_VERBS:
        raise UsageError("--no-wait is valid only for motion commands")
    if yes and verb != "sync-gains":
        raise UsageError("--yes is valid only for sync-gains")
    if verb in {"dump", "sync-gains"} and len(command_args) != 1:
        raise UsageError(verb + " takes no command arguments")
    if had_for and (
        verb != "stream"
        or len(command_args) != 4
        or command_args[1] == "off"
    ):
        raise UsageError("--for is valid only for bounded streams")
    if verb == "stream" and command_args[1:2] != ["off"]:
        if len(command_args) != 4:
            raise UsageError("stream requires --for <seconds>")

    command = " ".join(command_args)
    try:
        encoded_command = command.encode("ascii")
    except UnicodeEncodeError as error:
        raise UsageError("commands must contain ASCII only") from error
    if len(encoded_command) + 1 > 128:
        raise UsageError("commands are limited to 128 bytes including newline")

    return {
        "mock": mock,
        "port": port,
        "command": command,
        "wait": not no_wait,
        "local": verb if verb in {"dump", "sync-gains"} else None,
        "yes": yes,
    }


def timeout_for(command):
    verb = first_token(command)
    if verb == "turn":
        return 5.0
    if verb in {"drive", "moveto"}:
        return 7.0
    if verb == "run":
        return 122.0
    tokens = command.split()
    if verb == "stream" and len(tokens) == 4:
        try:
            return int(tokens[3]) / 1000.0 + 2.0
        except ValueError:
            pass
    return 2.0


def exchange(transport, command, wait=True, output=sys.stdout):
    verb = first_token(command)
    motion_wait = wait and verb in MOTION_VERBS
    stream_wait = verb == "stream" and command.split()[1:2] != ["off"]
    deadline = time.monotonic() + timeout_for(command)
    saw_error = False
    saw_end = False
    terminal_code = None

    transport.write(command + "\n")
    try:
        while True:
            line = transport.readline(deadline)
            if line is None:
                print("brain: timed out waiting for " + verb, file=sys.stderr)
                return EXIT_TRANSPORT_ERROR

            record = extract_tag(line)
            if record is None:
                continue
            print(record, file=output)

            if record.startswith("[err]"):
                saw_error = True
            if record == "[end] " + verb:
                saw_end = True
                if saw_error:
                    return EXIT_ROBOT_ERROR
                if not motion_wait and not stream_wait:
                    return 0
                if terminal_code is not None:
                    return terminal_code

            if motion_wait and record.startswith("[evt] " + verb + " "):
                terminal_code = (
                    EXIT_ROBOT_ERROR if " stopped " in record else 0
                )
                if saw_end:
                    return terminal_code
            elif stream_wait and record == "[evt] stream expired":
                terminal_code = 0
                if saw_end:
                    return 0
    except KeyboardInterrupt:
        if motion_wait:
            transport.write("stop\n")
            print("brain: interrupted; sent stop", file=sys.stderr)
        return EXIT_ROBOT_ERROR


def parse_gains(records):
    gains = {}
    for line in records.splitlines():
        match = GAIN_RE.match(line)
        if match:
            gains[match.group(1)] = tuple(float(match.group(i)) for i in range(2, 5))
    if set(gains) != {"lat", "ang"}:
        raise SyncError("gains reply did not contain both controller rows")
    return gains


def format_number(value):
    rendered = f"{value:.4f}".rstrip("0").rstrip(".")
    return "0" if rendered == "-0" else rendered


def format_gains(gains):
    labels = (
        ("proportional gain (kP)", 0),
        ("integral gain (kI)", 1),
        ("derivative gain (kD)", 2),
    )
    blocks = []
    for name, axis in (("linearController", "lat"), ("angularController", "ang")):
        lines = ["lemlib::ControllerSettings " + name + "("]
        for label, index in labels:
            comma = "," if index < 2 else ""
            lines.append(
                "    " + format_number(gains[axis][index]) + comma + " // " + label
            )
        lines.append(");")
        blocks.append("\n".join(lines))
    return "\n\n".join(blocks)


NUMBER_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def find_matching_paren(source, open_index):
    depth = 0
    state = "code"
    index = open_index
    while index < len(source):
        char = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "line-comment":
            if char == "\n":
                state = "code"
        elif state == "block-comment":
            if char == "*" and following == "/":
                state = "code"
                index += 1
        elif state in {"string", "character"}:
            if char == "\\":
                index += 1
            elif (state == "string" and char == '"') or (
                state == "character" and char == "'"
            ):
                state = "code"
        elif char == "/" and following == "/":
            state = "line-comment"
            index += 1
        elif char == "/" and following == "*":
            state = "block-comment"
            index += 1
        elif char == '"':
            state = "string"
        elif char == "'":
            state = "character"
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return index
        index += 1
    raise SyncError("unterminated ControllerSettings constructor")


def mask_comments(text):
    masked = list(text)
    state = "code"
    index = 0
    while index < len(text):
        char = text[index]
        following = text[index + 1] if index + 1 < len(text) else ""
        if state == "line-comment":
            if char == "\n":
                state = "code"
            else:
                masked[index] = " "
        elif state == "block-comment":
            if char == "*" and following == "/":
                masked[index] = " "
                masked[index + 1] = " "
                state = "code"
                index += 1
            elif char != "\n":
                masked[index] = " "
        elif char == "/" and following == "/":
            masked[index] = " "
            masked[index + 1] = " "
            state = "line-comment"
            index += 1
        elif char == "/" and following == "*":
            masked[index] = " "
            masked[index + 1] = " "
            state = "block-comment"
            index += 1
        index += 1
    return "".join(masked)


def controller_block(source, name):
    declaration = re.compile(
        r"lemlib::ControllerSettings\s+" + re.escape(name) + r"\s*\("
    )
    matches = list(declaration.finditer(source))
    if len(matches) != 1:
        raise SyncError(
            "expected exactly one ControllerSettings block for " + name
        )
    open_index = source.find("(", matches[0].start(), matches[0].end())
    close_index = find_matching_paren(source, open_index)
    return matches[0].start(), close_index + 1, open_index, close_index


def controller_gain_tokens(source, name):
    _start, _end, open_index, close_index = controller_block(source, name)
    boundaries = []
    depth = 0
    state = "code"
    argument_start = open_index + 1
    index = argument_start
    while index < close_index and len(boundaries) < 3:
        char = source[index]
        following = source[index + 1] if index + 1 < close_index else ""
        if state == "line-comment":
            if char == "\n":
                state = "code"
        elif state == "block-comment":
            if char == "*" and following == "/":
                state = "code"
                index += 1
        elif state in {"string", "character"}:
            if char == "\\":
                index += 1
            elif (state == "string" and char == '"') or (
                state == "character" and char == "'"
            ):
                state = "code"
        elif char == "/" and following == "/":
            state = "line-comment"
            index += 1
        elif char == "/" and following == "*":
            state = "block-comment"
            index += 1
        elif char == '"':
            state = "string"
        elif char == "'":
            state = "character"
        elif char in "([{":
            depth += 1
        elif char in ")]}":
            depth -= 1
        elif char == "," and depth == 0:
            boundaries.append((argument_start, index))
            argument_start = index + 1
        index += 1

    if len(boundaries) != 3:
        raise SyncError(name + " has fewer than three constructor arguments")

    token_spans = []
    numeric = re.compile(r"(\s*)(" + NUMBER_PATTERN + r")(\s*)\Z", re.DOTALL)
    for start, end in boundaries:
        masked = mask_comments(source[start:end])
        match = numeric.fullmatch(masked)
        if not match:
            raise SyncError(
                name + " first three constructor arguments must be numeric"
            )
        token_spans.append(
            (start + match.start(2), start + match.end(2))
        )
    return token_spans


def rewrite_gains_text(source, gains):
    replacements = []
    for controller, axis in (
        ("linearController", "lat"),
        ("angularController", "ang"),
    ):
        if axis not in gains or len(gains[axis]) != 3:
            raise SyncError("missing gains for " + axis)
        spans = controller_gain_tokens(source, controller)
        for span, value in zip(spans, gains[axis]):
            replacements.append((span[0], span[1], format_number(value)))

    changed = 0
    rewritten = source
    for start, end, replacement in sorted(replacements, reverse=True):
        if source[start:end] != replacement:
            changed += 1
        rewritten = rewritten[:start] + replacement + rewritten[end:]
    return rewritten, changed


def controller_blocks_clean(path, current_source):
    try:
        root_result = subprocess.run(
            ["git", "-C", str(path.parent), "rev-parse", "--show-toplevel"],
            text=True,
            capture_output=True,
            timeout=2,
        )
        if root_result.returncode != 0:
            return
        root = Path(root_result.stdout.strip()).resolve()
        relative = path.resolve().relative_to(root)
        head_result = subprocess.run(
            ["git", "-C", str(root), "show", "HEAD:" + relative.as_posix()],
            text=True,
            capture_output=True,
            timeout=2,
        )
        if head_result.returncode != 0:
            return
    except (OSError, subprocess.SubprocessError, ValueError):
        return

    head_source = head_result.stdout
    for name in ("linearController", "angularController"):
        current = controller_block(current_source, name)
        committed = controller_block(head_source, name)
        if current_source[current[0] : current[1]] != head_source[
            committed[0] : committed[1]
        ]:
            raise SyncError(name + " has uncommitted changes")


def write_transactionally(path, text):
    mode = path.stat().st_mode
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=path.parent,
            prefix="." + path.name + ".",
            delete=False,
        ) as temporary:
            temporary.write(text)
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_name = temporary.name
        os.chmod(temporary_name, mode)
        os.replace(temporary_name, path)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


class SerialTransport:
    def __init__(self, port):
        try:
            import serial
        except ImportError as error:
            raise BrainEnvironmentError(
                "pyserial is required; run: python3 -m pip install pyserial"
            ) from error

        try:
            self._serial = serial.Serial(
                port,
                115200,
                timeout=0.1,
                write_timeout=0.5,
            )
            reset = getattr(self._serial, "reset_input_buffer", None)
            if reset:
                reset()
        except (serial.SerialException, OSError) as error:
            raise TransportError(str(error)) from error
        self._serial_exception = serial.SerialException

    def write(self, text):
        try:
            self._serial.write(text.encode("ascii"))
            self._serial.flush()
        except (self._serial_exception, OSError) as error:
            raise TransportError(str(error)) from error

    def readline(self, deadline):
        while time.monotonic() < deadline:
            try:
                line = self._serial.readline()
            except (self._serial_exception, OSError) as error:
                raise TransportError(str(error)) from error
            if line:
                return line.decode("ascii", errors="replace")
        return None

    def close(self):
        self._serial.close()


def probe_ver(transport):
    deadline = time.monotonic() + 0.75
    transport.write("ver\n")
    found = False
    while True:
        line = transport.readline(deadline)
        if line is None:
            return found
        record = extract_tag(line)
        if record is None:
            continue
        if record.startswith("[ok] ver"):
            found = True
        if record == "[end] ver":
            return found


def open_transport(port):
    selected = port or os.environ.get("V5_PORT")
    if selected:
        return SerialTransport(selected)

    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    for candidate in candidates:
        try:
            transport = SerialTransport(candidate)
        except TransportError:
            continue
        try:
            if probe_ver(transport):
                return transport
        except TransportError:
            pass
        transport.close()
    raise TransportError("no V5 user serial device answered `ver`")


def sync_gains(path, old_source, gains, assume_yes):
    controller_blocks_clean(path, old_source)
    new_source, changed = rewrite_gains_text(old_source, gains)
    diff = "".join(
        difflib.unified_diff(
            old_source.splitlines(keepends=True),
            new_source.splitlines(keepends=True),
            fromfile=str(path),
            tofile=str(path),
        )
    )
    if diff:
        sys.stdout.write(diff)

    if changed:
        if not assume_yes:
            if not sys.stdin.isatty():
                raise SyncError(
                    "confirmation requires a TTY; rerun with --yes"
                )
            answer = input("Apply these gain changes? [y/N] ")
            if answer.strip().lower() not in {"y", "yes"}:
                raise SyncError("gain synchronization cancelled")
        write_transactionally(path, new_source)

    print(
        f"2 controllers checked, {changed} gains changed. run `make`."
    )


def usage():
    return (
        "usage: tools/brain [--mock] [--port DEVICE] <command>\n"
        "       tools/brain stream <topic> <period_ms> --for <seconds>s\n"
        "       tools/brain dump\n"
        "       tools/brain sync-gains [--yes]"
    )


def main(argv=None):
    try:
        options = parse_arguments(sys.argv[1:] if argv is None else argv)
    except UsageError as error:
        print("brain: " + str(error), file=sys.stderr)
        print(usage(), file=sys.stderr)
        return EXIT_USAGE_ERROR

    local = options["local"]
    wire_command = "gains" if local else options["command"]

    try:
        transport = (
            MockTransport(wire_command)
            if options["mock"]
            else open_transport(options["port"])
        )
    except BrainEnvironmentError as error:
        print("brain: " + str(error), file=sys.stderr)
        return EXIT_USAGE_ERROR
    except TransportError as error:
        print("brain: " + str(error), file=sys.stderr)
        return EXIT_TRANSPORT_ERROR

    captured = None
    output = sys.stdout
    if local:
        import io

        captured = io.StringIO()
        output = captured

    try:
        try:
            code = exchange(transport, wire_command, options["wait"], output)
        except TransportError as error:
            print("brain: " + str(error), file=sys.stderr)
            code = EXIT_TRANSPORT_ERROR
    finally:
        transport.close()

    if code != 0 or not local:
        if local and captured is not None:
            sys.stdout.write(captured.getvalue())
        return code

    try:
        gains = parse_gains(captured.getvalue())
    except SyncError as error:
        print("brain: " + str(error), file=sys.stderr)
        return EXIT_USAGE_ERROR

    if local == "dump":
        print(format_gains(gains))
        return 0

    config_path = Path(
        os.environ.get(
            "BRAIN_CONFIG",
            Path(__file__).resolve().parents[1] / "src" / "robot-config.cpp",
        )
    )
    try:
        old_source = config_path.read_text(encoding="utf-8")
        sync_gains(config_path, old_source, gains, options["yes"])
    except (OSError, SyncError) as error:
        print("brain: " + str(error), file=sys.stderr)
        return EXIT_USAGE_ERROR
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
