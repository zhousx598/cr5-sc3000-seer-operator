import json
import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from seer_agv_driver.seer_client import DEFAULT_HOST, SeerClient


class SeerAgvPanel(Node):
    def __init__(self) -> None:
        super().__init__("seer_agv_panel")
        self.declare_parameter("host", DEFAULT_HOST)
        self.declare_parameter("cmd_vel_topic", "/seer_agv/cmd_vel")
        self.declare_parameter("forward_speed", 0.02)
        self.declare_parameter("backward_speed", 0.03)
        self.declare_parameter("angular_speed", 0.10)

        self.host = str(self.get_parameter("host").value)
        self.client = SeerClient(self.host, timeout=3.0)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.root = tk.Tk()
        self.root.title("SEER AGV Control Panel")
        self.root.geometry("560x600+80+80")
        self.root.attributes("-topmost", True)
        self.root.after(2500, lambda: self.root.attributes("-topmost", False))
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.keyboard_enabled = tk.BooleanVar(value=False)
        self.selected_station = tk.StringVar(value="")
        self.status_text = tk.StringVar(value="Connecting...")
        self.pose_text = tk.StringVar(value="")
        self.nav_text = tk.StringVar(value="")
        self.teleop_text = tk.StringVar(value="Keyboard OFF")
        self.forward_speed = tk.DoubleVar(value=float(self.get_parameter("forward_speed").value))
        self.backward_speed = tk.DoubleVar(value=float(self.get_parameter("backward_speed").value))
        self.angular_speed = tk.DoubleVar(value=float(self.get_parameter("angular_speed").value))
        self.nav_speed = tk.DoubleVar(value=0.08)

        self._pressed: set[str] = set()
        self._busy = False
        self._closed = False
        self._last_status = None
        self._stations: list[dict] = []

        self._build_ui()
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.after(100, self._refresh_stations)
        self.root.after(200, self._poll_status)
        self.root.after(100, self._publish_teleop)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text="SEER AGV Control Panel", font=("", 14, "bold")).pack(anchor=tk.W)

        status = ttk.LabelFrame(main, text="Status", padding=10)
        status.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(status, textvariable=self.status_text, font=("", 11, "bold")).pack(anchor=tk.W)
        ttk.Label(status, textvariable=self.pose_text).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(status, textvariable=self.nav_text).pack(anchor=tk.W, pady=(2, 0))

        nav = ttk.LabelFrame(main, text="Station Navigation", padding=10)
        nav.pack(fill=tk.X, pady=(12, 0))
        row = ttk.Frame(nav)
        row.pack(fill=tk.X)
        ttk.Label(row, text="Target").pack(side=tk.LEFT)
        self.station_combo = ttk.Combobox(row, textvariable=self.selected_station, state="readonly", width=16)
        self.station_combo.pack(side=tk.LEFT, padx=(8, 12))
        ttk.Label(row, text="max m/s").pack(side=tk.LEFT)
        ttk.Spinbox(row, from_=0.02, to=0.20, increment=0.01, textvariable=self.nav_speed, width=6).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        buttons = ttk.Frame(nav)
        buttons.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(buttons, text="Go", command=self._goto_selected).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(buttons, text="Cancel Nav", command=self._cancel_nav).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0))
        ttk.Button(buttons, text="Refresh Stations", command=self._refresh_stations).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0)
        )

        teleop = ttk.LabelFrame(main, text="Keyboard Teleop", padding=10)
        teleop.pack(fill=tk.X, pady=(12, 0))
        ttk.Checkbutton(
            teleop,
            textvariable=self.teleop_text,
            variable=self.keyboard_enabled,
            command=self._toggle_keyboard,
        ).pack(anchor=tk.W)

        speeds = ttk.Frame(teleop)
        speeds.pack(fill=tk.X, pady=(10, 0))
        self._speed_control(speeds, "Forward", self.forward_speed, 0)
        self._speed_control(speeds, "Backward", self.backward_speed, 1)
        self._speed_control(speeds, "Turn", self.angular_speed, 2)

        ttk.Label(
            teleop,
            text="Arrow keys move while Keyboard ON. Space sends stop. q turns keyboard off.",
        ).pack(anchor=tk.W, pady=(10, 0))
        ttk.Button(teleop, text="Stop Motion", command=self._stop_motion).pack(fill=tk.X, pady=(10, 0))

        log_box = ttk.LabelFrame(main, text="Log", padding=8)
        log_box.pack(fill=tk.BOTH, expand=True, pady=(12, 0))
        self.log = tk.Text(log_box, height=9, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

    def _speed_control(self, parent: ttk.Frame, label: str, var: tk.DoubleVar, column: int) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 8, 0))
        parent.columnconfigure(column, weight=1)
        ttk.Label(frame, text=label).pack(anchor=tk.W)
        ttk.Spinbox(frame, from_=0.0, to=0.2, increment=0.01, textvariable=var, width=8).pack(fill=tk.X)

    def _log(self, text: str) -> None:
        self.log.insert(tk.END, time.strftime("%H:%M:%S ") + text + "\n")
        self.log.see(tk.END)

    def _run_background(self, target) -> None:
        if self._busy:
            self._log("Busy, wait for current command to finish")
            return

        def wrapper():
            self._busy = True
            try:
                target()
            except Exception as exc:
                self.root.after(0, lambda: self._log(f"ERROR: {exc}"))
            finally:
                self._busy = False

        threading.Thread(target=wrapper, daemon=True).start()

    def _refresh_stations(self) -> None:
        def work():
            stations = self.client.get_stations().get("stations", [])
            stations = [s for s in stations if isinstance(s, dict) and s.get("id")]
            ids = [str(s["id"]) for s in stations]

            def update():
                self._stations = stations
                self.station_combo["values"] = ids
                if ids and self.selected_station.get() not in ids:
                    self.selected_station.set(ids[0])
                self._log(f"Loaded stations: {', '.join(ids)}")

            self.root.after(0, update)

        self._run_background(work)

    def _poll_status(self) -> None:
        if self._closed:
            return

        def work():
            safety, raw = self.client.read_safety_state()
            pose = self.client.get_pose()
            self._last_status = safety
            nav = raw.get("nav_status", {})
            speed = raw.get("speed", {})
            station = pose.get("current_station", "")
            text = "SAFE" if safety.safe_to_move else f"INHIBITED: {safety.reason}"
            pose_line = (
                f"x={pose.get('x', 0.0):.3f} y={pose.get('y', 0.0):.3f} "
                f"yaw={pose.get('angle', 0.0):.3f} station={station!r}"
            )
            nav_line = (
                f"task_status={nav.get('task_status')} "
                f"vx={speed.get('vx')} w={speed.get('w')} stopped={speed.get('is_stop')}"
            )
            self.root.after(0, lambda: self.status_text.set(text))
            self.root.after(0, lambda: self.pose_text.set(pose_line))
            self.root.after(0, lambda: self.nav_text.set(nav_line))

        threading.Thread(target=work, daemon=True).start()
        self.root.after(500, self._poll_status)

    def _goto_selected(self) -> None:
        station_id = self.selected_station.get()
        if not station_id:
            messagebox.showwarning("SEER AGV", "No target station selected")
            return
        if not messagebox.askyesno("Confirm Navigation", f"Navigate to {station_id}?"):
            return

        def work():
            safety, _ = self.client.read_safety_state()
            if not safety.safe_to_move:
                self.root.after(0, lambda: self._log(f"Navigation blocked: {safety.reason}"))
                return
            task_id = "ui_" + station_id + "_" + time.strftime("%Y%m%d_%H%M%S")
            response = self.client.goto_station(
                station_id,
                task_id=task_id,
                max_speed=self.nav_speed.get(),
                max_wspeed=0.2,
                max_acc=0.1,
                max_wacc=0.1,
            )
            self.root.after(0, lambda: self._log(f"3051 {station_id}: {json.dumps(response, ensure_ascii=False)}"))

        self._run_background(work)

    def _cancel_nav(self) -> None:
        def work():
            response = self.client.cancel_nav()
            self.root.after(0, lambda: self._log(f"3003 cancel_nav: {json.dumps(response, ensure_ascii=False)}"))

        self._run_background(work)

    def _toggle_keyboard(self) -> None:
        if self.keyboard_enabled.get():
            self.teleop_text.set("Keyboard ON")
            self.root.focus_set()
            self._log("Keyboard teleop enabled")
        else:
            self.teleop_text.set("Keyboard OFF")
            self._pressed.clear()
            self._publish_zero()
            self._log("Keyboard teleop disabled")

    def _on_key_press(self, event) -> None:
        key = event.keysym
        if key == "q":
            self.keyboard_enabled.set(False)
            self._toggle_keyboard()
            return
        if key == "space":
            self._stop_motion()
            return
        if self.keyboard_enabled.get() and key in {"Up", "Down", "Left", "Right"}:
            self._pressed.add(key)

    def _on_key_release(self, event) -> None:
        self._pressed.discard(event.keysym)

    def _publish_teleop(self) -> None:
        if self._closed:
            return
        if self.keyboard_enabled.get():
            msg = Twist()
            if "Up" in self._pressed:
                msg.linear.x += abs(self.forward_speed.get())
            if "Down" in self._pressed:
                msg.linear.x -= abs(self.backward_speed.get())
            if "Left" in self._pressed:
                msg.angular.z += abs(self.angular_speed.get())
            if "Right" in self._pressed:
                msg.angular.z -= abs(self.angular_speed.get())
            self.cmd_pub.publish(msg)
        self.root.after(100, self._publish_teleop)

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def _stop_motion(self) -> None:
        self._pressed.clear()
        self._publish_zero()

        def work():
            response = self.client.stop()
            self.root.after(0, lambda: self._log(f"2000 stop: {json.dumps(response, ensure_ascii=False)}"))

        self._run_background(work)

    def _on_close(self) -> None:
        self._closed = True
        self.keyboard_enabled.set(False)
        self._publish_zero()
        try:
            self.client.stop()
        except Exception:
            pass
        self.client.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args: list[str] | None = None) -> None:
    del args
    raise SystemExit(
        "The standalone SEER panel is disabled because it opens a second set "
        "of robot TCP connections. Use the integrated dobot_operator_gui, "
        "which controls /seer_agv_node only through ROS topics and services."
    )


if __name__ == "__main__":
    main()
