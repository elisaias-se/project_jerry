"""Desktop controller for the Octopus Pro arm firmware."""

from __future__ import annotations

import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


BAUD_RATE = 115200
AXES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_rotate", "gripper")
HOME_POSE = (90, 90, 90, 90, 90, 120)
INFO_PATTERN = re.compile(
    r"^INFO BOARD=(\S+) PROTOCOL=(\d+) ACTIVE=(\S+)"
    r"(?: UART_READBACK=(\S+)| UART=(\S+))?$"
)


class ArmSerialGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Octopus Pro Arm Controller")
        self.root.geometry("760x650")
        self.root.minsize(700, 580)

        self.serial: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_reader = threading.Event()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.active_axes: set[str] = set()
        self.pose_vars = [tk.IntVar(value=value) for value in HOME_POSE]
        self.axis_controls: dict[str, list[tk.Widget]] = {}

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Disconnected")
        self.board_var = tk.StringVar(value="Board: not detected")
        self.current_var = tk.IntVar(value=1000)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self._process_events)
        self.root.after(250, self.auto_detect)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(outer, text="USB serial connection", padding=10)
        connection.pack(fill="x")
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=42)
        self.port_box.grid(row=0, column=0, padx=(0, 8), sticky="ew")
        ttk.Button(connection, text="Scan", command=self.scan_ports).grid(row=0, column=1, padx=4)
        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=4)
        ttk.Label(connection, textvariable=self.connection_var).grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(connection, textvariable=self.board_var).grid(row=1, column=1, columnspan=2, sticky="e", pady=(8, 0))
        connection.columnconfigure(0, weight=1)

        pose_frame = ttk.LabelFrame(outer, text="POSE values (degrees)", padding=10)
        pose_frame.pack(fill="x", pady=12)
        for row, (axis, variable) in enumerate(zip(AXES, self.pose_vars)):
            label = ttk.Label(pose_frame, text=axis.replace("_", " ").title(), width=14)
            scale = ttk.Scale(
                pose_frame,
                from_=0,
                to=180,
                variable=variable,
                command=lambda value, var=variable: var.set(round(float(value))),
            )
            spin = ttk.Spinbox(pose_frame, from_=0, to=180, textvariable=variable, width=6)
            label.grid(row=row, column=0, sticky="w", pady=3)
            scale.grid(row=row, column=1, sticky="ew", padx=8, pady=3)
            spin.grid(row=row, column=2, pady=3)
            self.axis_controls[axis] = [label, scale, spin]
        pose_frame.columnconfigure(1, weight=1)

        actions = ttk.Frame(outer)
        actions.pack(fill="x")
        ttk.Button(actions, text="Send POSE", command=self.send_pose).pack(side="left", padx=(0, 6))
        ttk.Button(actions, text="Base −5°", command=lambda: self.jog_base(-5)).pack(side="left", padx=3)
        ttk.Button(actions, text="Base +5°", command=lambda: self.jog_base(5)).pack(side="left", padx=3)
        ttk.Button(actions, text="HOME", command=lambda: self.send_command("HOME")).pack(side="left", padx=3)
        ttk.Button(actions, text="STATUS", command=lambda: self.send_command("STATUS")).pack(side="left", padx=3)
        ttk.Button(actions, text="Disable motors", command=lambda: self.send_command("DISABLE")).pack(side="right")

        diagnostics = ttk.Frame(outer)
        diagnostics.pack(fill="x", pady=(8, 0))
        ttk.Label(diagnostics, text="Raw MOTOR0 diagnostic:").pack(side="left", padx=(0, 6))
        ttk.Button(diagnostics, text="Raw CW", command=lambda: self.send_command("RAW_CW")).pack(side="left", padx=3)
        ttk.Button(diagnostics, text="Raw CCW", command=lambda: self.send_command("RAW_CCW")).pack(side="left", padx=3)
        ttk.Label(diagnostics, text="Run current (mA):").pack(side="left", padx=(16, 4))
        ttk.Spinbox(diagnostics, from_=300, to=1400, increment=50, textvariable=self.current_var, width=6).pack(side="left")
        ttk.Button(diagnostics, text="Set current", command=self.set_current).pack(side="left", padx=4)

        log_frame = ttk.LabelFrame(outer, text="Serial log", padding=8)
        log_frame.pack(fill="both", expand=True, pady=(12, 0))
        self.log = tk.Text(log_frame, height=12, state="disabled", wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self._set_active_axes(set())

    def scan_ports(self) -> list[str]:
        ports = sorted(port.device for port in list_ports.comports())
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self._append_log("SYSTEM", f"Found {len(ports)} serial port(s)")
        return ports

    def auto_detect(self) -> None:
        if self.serial and self.serial.is_open:
            return
        ports = self.scan_ports()
        likely = [port for port in ports if "usbmodem" in port.lower() or "ttyacm" in port.lower()]
        for port in likely + [port for port in ports if port not in likely]:
            if self._connect(port, quiet=True):
                return
        self.connection_var.set("No compatible Octopus firmware detected")

    def toggle_connection(self) -> None:
        if self.serial and self.serial.is_open:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return
        self._connect(port)

    def _connect(self, port: str, quiet: bool = False) -> bool:
        try:
            connection = serial.Serial(port, BAUD_RATE, timeout=0.2, write_timeout=1)
            time.sleep(0.8)
            connection.reset_input_buffer()
            connection.write(b"INFO\n")
            connection.flush()
            deadline = time.monotonic() + 1.5
            response = ""
            legacy_firmware = False
            while time.monotonic() < deadline:
                line = connection.readline().decode(errors="replace").strip()
                if line:
                    response = line
                    if INFO_PATTERN.match(line):
                        break
                    if line == "ERROR Unknown command":
                        legacy_firmware = True
                        break
            match = INFO_PATTERN.match(response)
            if not match and not legacy_firmware:
                connection.close()
                if not quiet:
                    messagebox.showerror("Not detected", f"No Octopus INFO response from {port}.")
                return False

            self.serial = connection
            self.port_var.set(port)
            if match:
                board, protocol, active, uart_readback, legacy_uart = match.groups()
                uart_status = uart_readback or legacy_uart
                active_axes = {name.lower() for name in active.split(",") if name != "NONE"}
            else:
                board, protocol = "OCTOPUS_PRO_V1_1_LEGACY", "legacy"
                active_axes = {"base"}
            self._set_active_axes(active_axes)
            self.connection_var.set(f"Connected: {port}")
            uart_label = f" · UART {uart_status}" if match and uart_status else ""
            self.board_var.set(f"Board: {board} · protocol {protocol}{uart_label}")
            self.connect_button.configure(text="Disconnect")
            self._append_log("RX", response)
            self._start_reader()
            self.send_command("STATUS")
            return True
        except (OSError, serial.SerialException) as error:
            if not quiet:
                messagebox.showerror("Serial error", str(error))
            return False

    def _start_reader(self) -> None:
        self.stop_reader.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def _reader_loop(self) -> None:
        while not self.stop_reader.is_set():
            connection = self.serial
            if not connection or not connection.is_open:
                return
            try:
                line = connection.readline().decode(errors="replace").strip()
                if line:
                    self.events.put(("line", line))
            except (OSError, serial.SerialException) as error:
                self.events.put(("error", str(error)))
                return

    def _process_events(self) -> None:
        try:
            while True:
                event, value = self.events.get_nowait()
                if event == "line":
                    self._append_log("RX", str(value))
                elif event == "error":
                    self._append_log("ERROR", str(value))
                    self.disconnect()
        except queue.Empty:
            pass
        self.root.after(100, self._process_events)

    def _set_active_axes(self, active_axes: set[str]) -> None:
        self.active_axes = active_axes
        for axis, controls in self.axis_controls.items():
            state = "normal" if axis in active_axes else "disabled"
            for control in controls[1:]:
                control.configure(state=state)

    def send_pose(self) -> None:
        values = [variable.get() for variable in self.pose_vars]
        self.send_command("POSE " + " ".join(str(value) for value in values))

    def jog_base(self, amount: int) -> None:
        value = max(0, min(180, self.pose_vars[0].get() + amount))
        self.pose_vars[0].set(value)
        self.send_pose()

    def set_current(self) -> None:
        value = self.current_var.get()
        if not 300 <= value <= 1400:
            messagebox.showerror("Invalid current", "Choose a current from 300 to 1400 mA.")
            return
        self.send_command(f"CURRENT {value}")

    def send_command(self, command: str) -> None:
        connection = self.serial
        if not connection or not connection.is_open:
            messagebox.showerror("Not connected", "Connect to the Octopus first.")
            return
        try:
            connection.write((command + "\n").encode())
            connection.flush()
            self._append_log("TX", command)
        except (OSError, serial.SerialException) as error:
            self._append_log("ERROR", str(error))
            self.disconnect()

    def _append_log(self, direction: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{timestamp}] {direction}: {message}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def disconnect(self) -> None:
        self.stop_reader.set()
        connection, self.serial = self.serial, None
        if connection and connection.is_open:
            connection.close()
        self.connection_var.set("Disconnected")
        self.board_var.set("Board: not detected")
        self.connect_button.configure(text="Connect")
        self._set_active_axes(set())

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ArmSerialGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
