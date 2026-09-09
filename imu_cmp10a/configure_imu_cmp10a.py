#!/usr/bin/env python3
"""CMP10A configuration UI.

    python configure_imu_cmp10a.py

Scans every serial port and baud rate, shows what the module is set to, and
lets you change it.  "Recommended" fills in settings suited to a legged robot.
Nothing is written until you press "Apply and Save".

Needs membership of the dialout group:
    sudo usermod -aG dialout $USER && newgrp dialout
"""

import argparse
import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

import serial

from measure_imu_cmp10a import BAUDS, find_ports, parse, try_baud

# ---------------------------------------------------------------- registers --
REG_SAVE, REG_CALSW, REG_RSW, REG_RRATE, REG_BAUD = 0x00, 0x01, 0x02, 0x03, 0x04
REG_LEDOFF, REG_BANDWIDTH, REG_SLEEP = 0x1B, 0x1F, 0x22
REG_ORIENT, REG_AXIS6, REG_GYROCALITHR = 0x23, 0x24, 0x61

UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])

RATES = {"0.2 Hz": 0x01, "0.5 Hz": 0x02, "1 Hz": 0x03, "2 Hz": 0x04, "5 Hz": 0x05,
         "10 Hz": 0x06, "20 Hz": 0x07, "50 Hz": 0x08, "100 Hz": 0x09, "200 Hz": 0x0B}
RATE_HZ = {"0.2 Hz": 0.2, "0.5 Hz": 0.5, "1 Hz": 1, "2 Hz": 2, "5 Hz": 5,
           "10 Hz": 10, "20 Hz": 20, "50 Hz": 50, "100 Hz": 100, "200 Hz": 200}
BAUDRATES = {"4800": 0x01, "9600": 0x02, "19200": 0x03, "38400": 0x04, "57600": 0x05,
             "115200": 0x06, "230400": 0x07, "460800": 0x08, "921600": 0x09}
BANDWIDTHS = {"256 Hz": 0x00, "188 Hz": 0x01, "98 Hz": 0x02, "42 Hz": 0x03,
              "20 Hz": 0x04, "10 Hz": 0x05, "5 Hz": 0x06}
ORIENTS = {"horizontal": 0x00, "vertical": 0x01}
ALGOS = {"9-axis (uses magnetometer)": 0x00, "6-axis (ignores magnetometer)": 0x01}

# RSW bit -> (label, packet id).  Default 0x1E = ACC + GYRO + ANGLE + MAG.
RSW_BITS = [(0, "time", 0x50), (1, "acceleration", 0x51), (2, "angular velocity", 0x52),
            (3, "angle", 0x53), (4, "magnetic field", 0x54), (5, "port status", 0x55),
            (6, "pressure / height", 0x56), (7, "GPS", 0x57), (8, "velocity", 0x58),
            (9, "quaternion", 0x59), (10, "GPS accuracy", 0x5A)]

BYTES_PER_PACKET = 11
RECOMMENDED = {"rate": "200 Hz", "baud": "230400", "bandwidth": "188 Hz",
               "orient": "horizontal", "algo": "9-axis (uses magnetometer)",
               "still_thr": "0", "led_off": False,
               "rsw": {1, 2, 3, 9}}     # acceleration, angular velocity, angle, quaternion


def cmd(reg, value):
    return bytes([0xFF, 0xAA, reg, value & 0xFF, (value >> 8) & 0xFF])


class App:
    def __init__(self, root):
        self.root = root
        root.title("CMP10A configuration")
        self.port = None
        self.baud = None
        self.q = queue.Queue()

        pad = {"padx": 10, "pady": 6}
        main = ttk.Frame(root, padding=16)
        main.grid(sticky="nsew")

        # ---------------- connection ----------------
        con = ttk.LabelFrame(main, text="Connection", padding=14)
        con.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)
        self.port_var, self.baud_var = tk.StringVar(), tk.StringVar()
        ttk.Label(con, text="port").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(con, textvariable=self.port_var, width=24, state="readonly")
        self.port_box.grid(row=0, column=1, **pad)
        ttk.Label(con, text="baud").grid(row=0, column=2, sticky="w")
        self.baud_box = ttk.Combobox(con, textvariable=self.baud_var, width=14,
                                     values=[str(b) for b in BAUDS], state="readonly")
        self.baud_box.grid(row=0, column=3, **pad)
        ttk.Button(con, text="Detect", command=self.on_detect).grid(row=0, column=4, **pad)
        ttk.Button(con, text="Measure rate", command=self.on_measure).grid(row=0, column=5, **pad)
        self.status = ttk.Label(con, text="not connected", foreground="#a33")
        self.status.grid(row=1, column=0, columnspan=6, sticky="w", padx=6)

        # ---------------- settings ----------------
        st = ttk.LabelFrame(main, text="Settings", padding=14)
        st.grid(row=1, column=0, sticky="nsew", **pad)
        self.rate_var = self._combo(st, 0, "output rate", list(RATES), "200 Hz")
        self.baud_set_var = self._combo(st, 1, "baud rate", list(BAUDRATES), "230400")
        self.bw_var = self._combo(st, 2, "internal bandwidth", list(BANDWIDTHS), "188 Hz")
        self.orient_var = self._combo(st, 3, "mounting", list(ORIENTS), "horizontal")
        self.algo_var = self._combo(st, 4, "algorithm", list(ALGOS),
                                    "9-axis (uses magnetometer)", width=32)
        ttk.Label(st, text="gyro still threshold").grid(row=5, column=0, sticky="w", **pad)
        self.thr_var = tk.StringVar(value="0")
        ttk.Entry(st, textvariable=self.thr_var, width=14).grid(row=5, column=1, sticky="w", **pad)
        self.led_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(st, text="turn LED off", variable=self.led_var).grid(
            row=6, column=0, columnspan=2, sticky="w", **pad)
        ttk.Label(st, foreground="#666", wraplength=340, justify="left",
                  text="Gyro range (2000 deg/s) and accelerometer range (16 g) are "
                       "fixed in firmware and cannot be changed.").grid(
            row=7, column=0, columnspan=2, sticky="w", padx=6, pady=(8, 0))

        # ---------------- output content ----------------
        out = ttk.LabelFrame(main, text="Output content", padding=14)
        out.grid(row=1, column=1, sticky="nsew", **pad)
        self.rsw_vars = {}
        for i, (bit, label, pid) in enumerate(RSW_BITS):
            v = tk.BooleanVar(value=bit in RECOMMENDED["rsw"])
            self.rsw_vars[bit] = v
            ttk.Checkbutton(out, text=f"{label}   (0x{pid:02X})", variable=v,
                            command=self.update_bandwidth_note).grid(
                row=i, column=0, sticky="w", padx=8, pady=3)
        self.bw_note = ttk.Label(out, foreground="#666", wraplength=300, justify="left")
        self.bw_note.grid(row=len(RSW_BITS), column=0, sticky="w", padx=6, pady=(8, 0))

        # ---------------- actions ----------------
        act = ttk.Frame(main)
        act.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)
        ttk.Button(act, text="Recommended", command=self.on_recommended).pack(side="left", padx=6, ipadx=8, ipady=4)
        ttk.Button(act, text="Apply and Save", command=self.on_apply).pack(side="left", padx=6, ipadx=8, ipady=4)
        ttk.Button(act, text="Calibrate accelerometer",
                   command=lambda: self.on_calsw(0x01, "accelerometer calibration",
                                                 "Lay the module flat and still, then press OK.")
                   ).pack(side="left", padx=6, ipadx=8, ipady=4)
        ttk.Button(act, text="Reset heading",
                   command=lambda: self.on_calsw(0x04, "heading reset",
                                                 "This sets the current heading as zero.")
                   ).pack(side="left", padx=6, ipadx=8, ipady=4)
        ttk.Button(act, text="Factory reset", command=self.on_factory).pack(side="right", padx=6, ipadx=8, ipady=4)

        # ---------------- log ----------------
        self.log = tk.Text(main, height=14, width=96, font="TkFixedFont",
                           wrap="none", relief="sunken", borderwidth=1)
        self.log.grid(row=3, column=0, columnspan=2, sticky="nsew", **pad)
        main.rowconfigure(3, weight=1)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        for var in (self.rate_var, self.baud_set_var):
            var.trace_add("write", lambda *_: self.update_bandwidth_note())
        self.update_bandwidth_note()
        self.refresh_ports()
        self.root.after(100, self.drain)

    # ------------------------------------------------------------------ ui --
    def _combo(self, parent, row, label, values, default, width=28):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=6, pady=3)
        var = tk.StringVar(value=default)
        ttk.Combobox(parent, textvariable=var, values=values, width=width,
                     state="readonly").grid(row=row, column=1, sticky="w", padx=6, pady=3)
        return var

    def say(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def drain(self):
        while not self.q.empty():
            kind, payload = self.q.get()
            if kind == "log":
                self.say(payload)
            elif kind == "status":
                self.status.config(text=payload[0], foreground=payload[1])
            elif kind == "found":
                self.port, self.baud = payload
                self.port_var.set(self.port)
                self.baud_var.set(str(self.baud))
        self.root.after(100, self.drain)

    def refresh_ports(self):
        try:
            ports = find_ports()
        except SystemExit:
            ports = []
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def update_bandwidth_note(self):
        n = sum(1 for b, v in self.rsw_vars.items() if v.get())
        try:
            hz = RATE_HZ[self.rate_var.get()]
            baud = int(self.baud_set_var.get())
        except (KeyError, ValueError):
            return
        needed = n * BYTES_PER_PACKET * hz * 10
        ok = needed <= baud * 0.8
        self.bw_note.config(
            text=f"{n} packet types selected\n{needed:,.0f} bps needed, {baud:,} available"
                 + ("" if ok else "\nTOO MUCH -- raise the baud rate or send fewer packets"),
            foreground="#666" if ok else "#a33")
        return ok

    # -------------------------------------------------------------- serial --
    def require_port(self):
        if not self.port_var.get() or not self.baud_var.get():
            messagebox.showwarning("No device", "Press Detect first.")
            return None
        return self.port_var.get(), int(self.baud_var.get())

    def on_detect(self):
        self.refresh_ports()
        self.q.put(("status", ("scanning ...", "#666")))
        threading.Thread(target=self._detect, daemon=True).start()

    def _detect(self):
        try:
            ports = find_ports()
        except SystemExit as e:
            self.q.put(("log", str(e)))
            self.q.put(("status", ("no serial port found", "#a33")))
            return
        best = (0, None, None)
        for p in ports:
            for b in BAUDS:
                try:
                    n = try_baud(p, b)
                except SystemExit as e:
                    self.q.put(("log", str(e)))
                    self.q.put(("status", ("permission denied", "#a33")))
                    return
                if n:
                    self.q.put(("log", f"  {p} @ {b:>7} -> {n} valid frames"))
                if n > best[0]:
                    best = (n, p, b)
        if best[1]:
            self.q.put(("found", (best[1], best[2])))
            self.q.put(("status", (f"connected: {best[1]} @ {best[2]}", "#2a7")))
        else:
            self.q.put(("status", ("no valid frames on any port/baud", "#a33")))

    def on_measure(self):
        pb = self.require_port()
        if pb:
            threading.Thread(target=self._measure, args=pb, daemon=True).start()

    def _measure(self, port, baud):
        self.q.put(("log", f"measuring rate on {port} @ {baud} ..."))
        try:
            with serial.Serial(port, baud, timeout=0.1) as ser:
                ser.reset_input_buffer()
                buf, stamps, t0 = b"", [], time.time()
                while time.time() - t0 < 3.0:
                    buf += ser.read(4096)
                    frames, buf = parse(buf)
                    now = time.time()
                    stamps += [now for k, _ in frames if k == 0x51]
        except (serial.SerialException, OSError) as e:
            self.q.put(("log", f"  failed: {e}"))
            return
        if len(stamps) < 3:
            self.q.put(("log", "  no acceleration packets -- is that output enabled?"))
            return
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        self.q.put(("log", f"  {hz:.1f} Hz  ({len(stamps)} frames in 3 s)"))

    def write_all(self, pairs, port, baud, new_baud=None):
        """Send unlock + each (register, value), then save."""
        try:
            with serial.Serial(port, baud, timeout=0.2) as ser:
                for reg, val in pairs:
                    ser.write(UNLOCK)
                    ser.flush()
                    time.sleep(0.12)
                    packet = cmd(reg, val)
                    ser.write(packet)
                    ser.flush()
                    self.q.put(("log", f"  0x{reg:02X} <- 0x{val:04X}   {packet.hex(' ').upper()}"))
                    time.sleep(0.15)
                if new_baud:
                    ser.baudrate = new_baud
                    time.sleep(0.3)
                ser.write(UNLOCK)
                ser.flush()
                time.sleep(0.12)
                ser.write(cmd(REG_SAVE, 0x0000))
                ser.flush()
                self.q.put(("log", "  saved to flash"))
                time.sleep(0.8)
            return True
        except (serial.SerialException, OSError) as e:
            self.q.put(("log", f"  failed: {e}"))
            return False

    # ------------------------------------------------------------- actions --
    def on_recommended(self):
        self.rate_var.set(RECOMMENDED["rate"])
        self.baud_set_var.set(RECOMMENDED["baud"])
        self.bw_var.set(RECOMMENDED["bandwidth"])
        self.orient_var.set(RECOMMENDED["orient"])
        self.algo_var.set(RECOMMENDED["algo"])
        self.thr_var.set(RECOMMENDED["still_thr"])
        self.led_var.set(RECOMMENDED["led_off"])
        for bit, v in self.rsw_vars.items():
            v.set(bit in RECOMMENDED["rsw"])
        self.update_bandwidth_note()
        self.say("Recommended settings loaded (not written yet).")
        self.say("  200 Hz so foot impacts are visible; 230400 baud to carry it;")
        self.say("  188 Hz bandwidth so the internal filter does not blunt the signal;")
        self.say("  acceleration + angular velocity + angle + quaternion.")
        self.say("  Press 'Apply and Save' to write them.")

    def on_apply(self):
        pb = self.require_port()
        if not pb:
            return
        if not self.update_bandwidth_note():
            messagebox.showerror("Too much data",
                                 "The selected rate and packets exceed the baud rate.")
            return
        rsw = sum(1 << b for b, v in self.rsw_vars.items() if v.get())
        if not rsw:
            messagebox.showerror("Nothing selected", "Select at least one output packet.")
            return
        try:
            thr = int(self.thr_var.get())
        except ValueError:
            messagebox.showerror("Bad value", "Gyro still threshold must be an integer.")
            return

        new_baud = int(self.baud_set_var.get())
        pairs = [
            (REG_RSW, rsw),
            (REG_BANDWIDTH, BANDWIDTHS[self.bw_var.get()]),
            (REG_ORIENT, ORIENTS[self.orient_var.get()]),
            (REG_AXIS6, ALGOS[self.algo_var.get()]),
            (REG_GYROCALITHR, thr),
            (REG_LEDOFF, 1 if self.led_var.get() else 0),
            (REG_RRATE, RATES[self.rate_var.get()]),
            (REG_BAUD, BAUDRATES[self.baud_set_var.get()]),
        ]
        self.say(f"\nWriting to {pb[0]} @ {pb[1]} ...")
        threading.Thread(target=self._apply, args=(pairs, pb[0], pb[1], new_baud),
                         daemon=True).start()

    def _apply(self, pairs, port, baud, new_baud):
        if self.write_all(pairs, port, baud, new_baud):
            self.q.put(("found", (port, new_baud)))
            self.q.put(("status", (f"connected: {port} @ {new_baud}", "#2a7")))
            self.q.put(("log", "Done.  Press 'Measure rate' to confirm, then run "
                               "measure_imu_cmp10a.py for the Isaac parameters."))

    def on_calsw(self, mode, name, hint):
        pb = self.require_port()
        if not pb:
            return
        if not messagebox.askokcancel(name, hint):
            return
        self.say(f"\n{name} ...")

        def run():
            if self.write_all([(REG_CALSW, mode)], *pb):
                time.sleep(3.0)
                self.write_all([(REG_CALSW, 0x00)], *pb)
                self.q.put(("log", f"  {name} finished"))
        threading.Thread(target=run, daemon=True).start()

    def on_factory(self):
        pb = self.require_port()
        if not pb:
            return
        if not messagebox.askokcancel(
                "Factory reset",
                "This restores every register to its factory default, including "
                "the baud rate (back to 9600) and output rate (back to 10 Hz).\n\n"
                "Continue?"):
            return
        self.say("\nFactory reset ...")

        def run():
            try:
                with serial.Serial(pb[0], pb[1], timeout=0.2) as ser:
                    ser.write(UNLOCK)
                    ser.flush()
                    time.sleep(0.12)
                    ser.write(cmd(REG_SAVE, 0x0001))
                    ser.flush()
                    self.q.put(("log", "  FF AA 00 01 00   factory reset sent"))
                    time.sleep(1.5)
            except (serial.SerialException, OSError) as e:
                self.q.put(("log", f"  failed: {e}"))
                return
            self.q.put(("status", ("reset -- press Detect to find it again", "#666")))
            self.q.put(("log", "  Done.  The module is back on 9600 baud / 10 Hz."))
            self.q.put(("log", "  Press Detect to reconnect."))
        threading.Thread(target=run, daemon=True).start()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scale", type=float, default=1.2,
                    help="UI scale factor. Default 1.2; try 1.5 or 2.0 on a "
                         "HiDPI screen, or 1.0 for a compact window.")
    args = ap.parse_args()

    root = tk.Tk()

    # Tk does not follow the desktop's HiDPI setting on Linux, so widgets come
    # out small on a high resolution screen.  Pass --scale to adjust.
    scale = max(0.8, min(3.0, args.scale))

    root.tk.call("tk", "scaling", 1.333 * scale)
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
        f = tkfont.nametofont(name)
        f.configure(size=max(9, int(round(10 * scale))))
    mono = tkfont.nametofont("TkFixedFont")
    mono.configure(size=max(8, int(round(9 * scale))))

    style = ttk.Style(root)
    style.configure("TButton", padding=(int(10 * scale), int(6 * scale)))
    base = tkfont.nametofont("TkDefaultFont")
    style.configure("TLabelframe.Label",
                    font=(base.actual("family"), max(10, int(round(11 * scale))), "bold"))

    root.geometry(f"{int(1000 * scale)}x{int(760 * scale)}")
    root.minsize(int(880 * scale), int(640 * scale))

    App(root)
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    root.mainloop()


if __name__ == "__main__":
    main()
