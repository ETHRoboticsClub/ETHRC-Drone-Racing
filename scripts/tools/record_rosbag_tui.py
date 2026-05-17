#!/usr/bin/env python3
"""TUI helper for recording, zipping, and uploading ROS 2 bags."""

from __future__ import annotations

import curses
import datetime as dt
import getpass
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import zipfile


DEFAULT_S3_PREFIX = "s3://ethrc-aeronautics/droneracing/bags"
PRESETS = {
    "all non-NITROS topics": None,
    "imu-cam calibration": ["/cam0/image_raw", "/cam0/camera_info", "/imu0"],
    "compressed video": ["/cam0/image_h264", "/cam0/camera_info", "/imu0"],
    "imu only": ["/imu0"],
    "control debug": ["/imu0", "/battery", "/motor_rpm", "/drone_attitude_cmd"],
}


def slug(value: str) -> str:
    value = value.strip().lower()
    value = re.sub(r"[^a-z0-9]+", "-", value)
    return value.strip("-") or "untitled"


def default_person() -> str:
    try:
        name = subprocess.check_output(
            ["git", "config", "user.name"], text=True, stderr=subprocess.DEVNULL
        ).strip()
        if name:
            return name
    except (OSError, subprocess.CalledProcessError):
        pass
    return getpass.getuser()


def today() -> str:
    return dt.date.today().isoformat()


def docs_style_name(date: str, drone: str, description: str) -> str:
    return f"{date}-{slug(drone)}-{slug(description)}"


def discover_topics() -> list[tuple[str, str]]:
    try:
        output = subprocess.check_output(
            ["ros2", "topic", "list", "-t"], text=True, stderr=subprocess.STDOUT
        )
    except (OSError, subprocess.CalledProcessError):
        return []

    topics: list[tuple[str, str]] = []
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        match = re.match(r"^(\S+)\s+\[(.+)\]$", line)
        if match:
            topics.append((match.group(1), match.group(2)))
        else:
            topics.append((line, ""))
    return sorted(topics, key=lambda item: item[0])


def is_nitros_topic(topic: str, msg_type: str) -> bool:
    """Return true for NITROS transport topics/types that should not be bagged by default."""
    haystack = f"{topic} {msg_type}".lower()
    return "nitros" in haystack


def default_selected_topics(topics: list[tuple[str, str]]) -> set[str]:
    """Select every discovered topic except NITROS transport topics."""
    return {
        topic for topic, msg_type in topics
        if not is_nitros_topic(topic, msg_type)
    }


class App:
    def __init__(self, stdscr: curses.window) -> None:
        self.stdscr = stdscr
        self.topics = discover_topics()
        self.selected = default_selected_topics(self.topics)
        self.add_missing_selected_topics()
        self.cursor = 0
        self.scroll = 0
        self.filter_text = ""
        self.message = "All non-NITROS topics selected. Space toggles topics. Enter records."

        self.date = today()
        self.person = default_person()
        self.drone = "zephyrus"
        self.description = "bench-all-topics"
        self.notes = ""
        self.output_root = "bags"
        self.zip_after = False
        self.upload_after = False
        self.s3_prefix = DEFAULT_S3_PREFIX

    def topic_exists(self, topic: str) -> bool:
        return any(candidate == topic for candidate, _ in self.topics)

    def add_missing_selected_topics(self) -> None:
        for topic in sorted(self.selected):
            if not self.topic_exists(topic):
                self.topics.append((topic, "manual"))
        self.topics.sort(key=lambda item: item[0])

    @property
    def visible_topics(self) -> list[tuple[str, str]]:
        if not self.filter_text:
            return self.topics
        needle = self.filter_text.lower()
        return [
            item for item in self.topics
            if needle in item[0].lower() or needle in item[1].lower()
        ]

    @property
    def bag_name(self) -> str:
        return docs_style_name(self.date, self.drone, self.description)

    @property
    def bag_dir(self) -> Path:
        return Path(self.output_root).expanduser() / self.bag_name

    @property
    def zip_path(self) -> Path:
        return self.bag_dir.with_suffix(".zip")

    def input_line(self, prompt: str, initial: str = "") -> str:
        height, width = self.stdscr.getmaxyx()
        curses.echo()
        curses.curs_set(1)
        self.stdscr.move(height - 2, 0)
        self.stdscr.clrtoeol()
        text = f"{prompt} [{initial}]: "
        self.stdscr.addnstr(height - 2, 0, text, width - 1)
        self.stdscr.refresh()
        value = self.stdscr.getstr(height - 2, min(len(text), width - 1), 512)
        curses.noecho()
        curses.curs_set(0)
        decoded = value.decode(errors="replace").strip()
        return decoded or initial

    def choose_preset(self) -> None:
        names = list(PRESETS)
        index = 0
        while True:
            self.draw_base()
            self.stdscr.addstr(3, 2, "Choose preset")
            for row, name in enumerate(names, start=5):
                prefix = "> " if row - 5 == index else "  "
                self.stdscr.addstr(row, 4, prefix + name)
            key = self.stdscr.getch()
            if key in (ord("q"), 27):
                return
            if key in (curses.KEY_UP, ord("k")):
                index = max(0, index - 1)
            elif key in (curses.KEY_DOWN, ord("j")):
                index = min(len(names) - 1, index + 1)
            elif key in (10, 13):
                preset = PRESETS[names[index]]
                if preset is None:
                    self.selected = default_selected_topics(self.topics)
                else:
                    self.selected = set(preset)
                self.add_missing_selected_topics()
                self.message = f"Preset applied: {names[index]}"
                return

    def edit_metadata(self) -> None:
        self.date = self.input_line("Date YYYY-MM-DD", self.date)
        self.person = self.input_line("Person", self.person)
        self.drone = self.input_line("Drone", self.drone)
        self.description = self.input_line("Description", self.description)
        self.notes = self.input_line("Notes", self.notes)
        self.output_root = self.input_line("Output directory", self.output_root)
        self.s3_prefix = self.input_line("S3 prefix", self.s3_prefix)
        self.message = f"Bag name: {self.bag_name}"

    def add_manual_topic(self) -> None:
        topic = self.input_line("Topic name", "")
        if topic:
            self.selected.add(topic)
            if not self.topic_exists(topic):
                self.topics.append((topic, "manual"))
                self.topics.sort(key=lambda item: item[0])
            self.message = f"Added {topic}"

    def draw_base(self) -> None:
        self.stdscr.erase()
        height, width = self.stdscr.getmaxyx()
        title = "ETHRC ROS bag recorder"
        self.stdscr.addnstr(0, 0, title, width - 1, curses.A_BOLD)
        self.stdscr.hline(1, 0, "-", max(0, width - 1))
        self.stdscr.addnstr(height - 1, 0, self.message, width - 1, curses.A_REVERSE)

    def draw(self) -> None:
        self.draw_base()
        height, width = self.stdscr.getmaxyx()
        left_width = max(44, width // 2)
        bag_line = f"Name: {self.bag_name}"
        opts_line = (
            f"Zip: {'yes' if self.zip_after else 'no'}  "
            f"Upload: {'yes' if self.upload_after else 'no'}"
        )
        self.stdscr.addnstr(2, 0, bag_line, width - 1)
        self.stdscr.addnstr(3, 0, f"Output: {self.bag_dir}", width - 1)
        self.stdscr.addnstr(4, 0, opts_line, width - 1)
        self.stdscr.addnstr(5, 0, f"Filter: {self.filter_text or '<none>'}", width - 1)

        self.stdscr.vline(2, left_width, "|", max(0, height - 4))
        self.stdscr.addnstr(7, 0, "Topics", left_width - 1, curses.A_BOLD)
        visible = self.visible_topics
        list_height = max(1, height - 10)
        if self.cursor >= len(visible):
            self.cursor = max(0, len(visible) - 1)
        if self.cursor < self.scroll:
            self.scroll = self.cursor
        if self.cursor >= self.scroll + list_height:
            self.scroll = self.cursor - list_height + 1

        for row, (topic, msg_type) in enumerate(visible[self.scroll:self.scroll + list_height]):
            idx = self.scroll + row
            marker = "[x]" if topic in self.selected else "[ ]"
            prefix = "> " if idx == self.cursor else "  "
            line = f"{prefix}{marker} {topic} {msg_type}"
            attr = curses.A_REVERSE if idx == self.cursor else curses.A_NORMAL
            self.stdscr.addnstr(8 + row, 0, line, left_width - 1, attr)

        side_x = left_width + 2
        self.stdscr.addnstr(7, side_x, "Selected topics", width - side_x - 1, curses.A_BOLD)
        for row, topic in enumerate(sorted(self.selected)[:max(0, height - 15)]):
            self.stdscr.addnstr(8 + row, side_x, topic, width - side_x - 1)

        controls = [
            "Enter/r: record",
            "Space: toggle topic",
            "/: filter",
            "a: add manual topic",
            "n: edit name/metadata",
            "p: preset",
            "z: toggle zip",
            "u: toggle upload",
            "R: refresh topics",
            "q: quit",
        ]
        controls_y = max(8, height - len(controls) - 1)
        for row, control in enumerate(controls):
            self.stdscr.addnstr(controls_y + row, side_x, control, width - side_x - 1)
        self.stdscr.refresh()

    def toggle_current(self) -> None:
        visible = self.visible_topics
        if not visible:
            return
        topic = visible[self.cursor][0]
        if topic in self.selected:
            self.selected.remove(topic)
        else:
            self.selected.add(topic)

    def validate(self) -> bool:
        if not self.selected:
            self.message = "Select at least one topic."
            return False
        if not re.match(r"^\d{4}-\d{2}-\d{2}$", self.date):
            self.message = "Date must be YYYY-MM-DD."
            return False
        if self.bag_dir.exists() or self.zip_path.exists():
            self.message = f"Output already exists: {self.bag_dir} or {self.zip_path}"
            return False
        return True

    def run(self) -> bool:
        curses.curs_set(0)
        self.stdscr.keypad(True)
        while True:
            self.draw()
            key = self.stdscr.getch()
            visible = self.visible_topics
            if key in (ord("q"), 27):
                return False
            if key in (curses.KEY_UP, ord("k")):
                self.cursor = max(0, self.cursor - 1)
            elif key in (curses.KEY_DOWN, ord("j")):
                self.cursor = min(max(0, len(visible) - 1), self.cursor + 1)
            elif key in (ord(" "),):
                self.toggle_current()
            elif key in (10, 13, ord("r")):
                if self.validate():
                    return True
            elif key == ord("/"):
                self.filter_text = self.input_line("Filter", self.filter_text)
                self.cursor = 0
                self.scroll = 0
            elif key == ord("a"):
                self.add_manual_topic()
            elif key == ord("n"):
                self.edit_metadata()
            elif key == ord("p"):
                self.choose_preset()
            elif key == ord("z"):
                self.zip_after = not self.zip_after
            elif key == ord("u"):
                self.upload_after = not self.upload_after
                if self.upload_after:
                    self.zip_after = True
            elif key == ord("R"):
                previous = set(self.selected)
                self.topics = discover_topics()
                self.selected = previous
                self.add_missing_selected_topics()
                self.message = f"Refreshed: {len(self.topics)} topics"


def write_metadata(app: App, start_time: str, end_time: str) -> None:
    metadata = {
        "name": app.bag_name,
        "date": app.date,
        "person": app.person,
        "drone": app.drone,
        "description": app.description,
        "notes": app.notes,
        "topics": sorted(app.selected),
        "started_at": start_time,
        "ended_at": end_time,
        "s3_prefix": app.s3_prefix,
        "docs_naming_scheme": "YYYY-MM-DD-DRONE_NAME-DESCRIPTION",
    }
    path = app.bag_dir / "ethrc_bag_metadata.json"
    path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def record_bag(app: App) -> int:
    app.bag_dir.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ros2", "bag", "record", "-o", str(app.bag_dir), *sorted(app.selected)]
    print("[INFO] Recording ROS bag.")
    print(f"[INFO] Output: {app.bag_dir}")
    print("[INFO] Topics:")
    for topic in sorted(app.selected):
        print(f"  {topic}")
    print("[INFO] Press Ctrl+C to stop recording cleanly.")
    print()

    start_time = dt.datetime.now(dt.timezone.utc).isoformat()
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    try:
        return_code = proc.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Stopping ros2 bag record...")
        os.killpg(proc.pid, signal.SIGINT)
        try:
            return_code = proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            return_code = proc.wait()
    end_time = dt.datetime.now(dt.timezone.utc).isoformat()

    if app.bag_dir.exists():
        write_metadata(app, start_time, end_time)
    return return_code


def zip_bag(app: App) -> Path:
    if app.zip_path.exists():
        raise FileExistsError(app.zip_path)
    print(f"[INFO] Zipping {app.bag_dir} -> {app.zip_path}")
    with zipfile.ZipFile(app.zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in app.bag_dir.rglob("*"):
            archive.write(path, path.relative_to(app.bag_dir.parent))
    return app.zip_path


def upload_zip(app: App, zip_path: Path) -> int:
    destination = f"{app.s3_prefix.rstrip('/')}/{zip_path.name}"
    cmd = ["aws", "s3", "cp", str(zip_path), destination]
    print(f"[INFO] Uploading {zip_path} -> {destination}")
    return subprocess.call(cmd)


def print_summary(app: App, return_code: int, zip_path: Path | None, upload_code: int | None) -> None:
    print()
    print("[SUMMARY]")
    print(f"Bag: {app.bag_dir}")
    print(f"ros2 bag record exit code: {return_code}")
    if zip_path:
        print(f"Zip: {zip_path}")
    if upload_code is not None:
        print(f"Upload exit code: {upload_code}")
        print(f"S3: {app.s3_prefix.rstrip('/')}/{app.zip_path.name}")
    print()
    print("Registry row draft:")
    topics = ", ".join(sorted(app.selected))
    s3_location = f"{app.s3_prefix.rstrip('/')}/{app.zip_path.name}" if zip_path else ""
    print(f"| {app.date} | {app.person} | {app.drone} | `{s3_location}` | {app.notes or app.description} | {topics} |")


def ensure_tools(app: App) -> bool:
    missing = [tool for tool in ["ros2"] if shutil.which(tool) is None]
    if app.upload_after and shutil.which("aws") is None:
        missing.append("aws")
    if missing:
        print("[ERROR] Missing required command(s): " + ", ".join(missing), file=sys.stderr)
        return False
    return True


def main() -> int:
    app_holder: dict[str, App] = {}

    def run_tui(stdscr: curses.window) -> bool:
        app = App(stdscr)
        app_holder["app"] = app
        return app.run()

    should_record = curses.wrapper(run_tui)
    app = app_holder.get("app")
    if not should_record or app is None:
        print("[INFO] Cancelled.")
        return 0
    if not ensure_tools(app):
        return 1

    return_code = record_bag(app)
    if return_code != 0:
        print(f"[WARN] ros2 bag record exited with {return_code}")

    zip_path: Path | None = None
    upload_code: int | None = None
    if app.zip_after and app.bag_dir.exists():
        zip_path = zip_bag(app)
    if app.upload_after and zip_path is not None:
        upload_code = upload_zip(app, zip_path)

    print_summary(app, return_code, zip_path, upload_code)
    return return_code if return_code != 0 else (upload_code or 0)


if __name__ == "__main__":
    raise SystemExit(main())
