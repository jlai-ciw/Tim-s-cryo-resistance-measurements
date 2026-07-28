"""
Cryo Resistance Logger
======================
Logs time, temperature (Cryo-con 22C), and resistance to a user-selected file
at a user-selected interval, with live plots. Resistance source is selectable
between a Keithley 6221 + 2182A (Delta mode or plain DC sourcing, via
Ethernet) and a Keithley 2400-series SourceMeter (via serial).

Requires: pyvisa, pyserial, matplotlib (and an installed VISA backend, e.g. NI-VISA)

Run:  python cryo_resistance_logger.py
"""

import csv
import math
import os
import queue
import random
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import ScalarFormatter

try:
    import pyvisa
except ImportError:
    pyvisa = None

try:
    import serial.tools.list_ports as _list_ports
except ImportError:
    _list_ports = None



# ---------------------------------------------------------------------------
# Instrument wrappers
# ---------------------------------------------------------------------------

# Keithley overflow / compliance-tripped sentinel (e.g. 9.91E+37), returned in
# place of a real reading by the 2182A (delta/DC voltage) and 2400-series
# (on-instrument resistance) alike whenever compliance trips or a sensor is
# over-range.
OVERFLOW_THRESHOLD = 9e37

# 2182A channel-1 voltage ranges (None = autorange). Autorange can lag behind
# delta mode's fast polarity reversals and clip mid-transition, showing up as
# intermittent overflow readings that a fixed range avoids.
NANOVOLTMETER_RANGES = {
    "Auto": None,
    "10 mV": 10e-3,
    "100 mV": 100e-3,
    "1 V": 1.0,
    "10 V": 10.0,
    "100 V": 100.0,
}

# 2182A digital averaging-filter type (:SENS:VOLT:AVER:TCON). Moving average
# continuously slides a window of N readings; repeating average waits for a
# full fresh window each cycle so a filtered value never blends pre- and
# post-transition readings; Off disables the filter entirely.
FILTER_TYPES = {
    "Moving Average": "MOV",
    "Repeating Average": "REP",
    "Off": "OFF",
}

# Sentinel for update_delta_settings' voltage_range kwarg: unlike this
# function's other kwargs (where None means "leave unchanged"), None is a
# valid target value here (it means "switch to autorange").
_UNSET = object()


class Keithley6221:
    """Keithley 6221 AC/DC current source + 2182A nanovoltmeter in delta mode via Ethernet."""

    def __init__(self, host, delta_current=1e-3, delay=0.002, count=1,
                 nplc=1.0, compliance=10.0, mode="delta", voltage_range=None,
                 filter_type="MOV", filter_window=None, filter_count=10,
                 simulate=False):
        self.simulate = simulate
        self._delta_current = delta_current
        self._delay = delay
        self._count = max(1, int(count))
        self._nplc = nplc
        self._compliance = compliance
        self._mode = mode  # "delta" or "dc"
        self._voltage_range = voltage_range  # None = autorange; else fixed volts
        self._filter_type = filter_type  # "MOV", "REP", or "OFF"
        self._filter_window = filter_window  # None = unset (device default), else 0-10 %
        self._filter_count = max(2, int(filter_count))
        self.inst = None
        # Guards every access to self.inst: the GUI thread (mode switches,
        # settings changes) and the background logging thread (read_resistance)
        # both talk to the same VISA socket, and interleaved writes from two
        # threads corrupt the SCPI command stream (seen as -113 errors).
        self._lock = threading.RLock()
        if simulate:
            return
        rm = pyvisa.ResourceManager()
        # Keithley 6221 raw socket on port 1394
        self.inst = rm.open_resource(f"TCPIP0::{host}::1394::SOCKET")
        self.inst.timeout = 30000  # 30 s — delta mode can be slow
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        with self._lock:
            if self._mode == "delta":
                self._setup_delta()
            else:
                self._setup_dc()

    def _configure_2182a(self, line_sync=False):
        """Configure the 2182A via the 6221's RS-232 pass-through port.
        Requires a null-modem RS-232 cable between the two instruments."""
        cmds = [
            "SYST:PRES",
            ":SENS:FUNC 'VOLT'",
        ]
        if self._voltage_range is None:
            cmds.append(":SENS:VOLT:CHAN1:RANG:AUTO ON")
        else:
            cmds.append(":SENS:VOLT:CHAN1:RANG:AUTO OFF")
            cmds.append(f":SENS:VOLT:CHAN1:RANG {self._voltage_range:.6e}")
        cmds.append(f":SENS:VOLT:NPLC {self._nplc:.3g}")
        # Digital averaging filter: MOV (moving average) continuously slides
        # a window of N readings; REP (repeating average) waits for a full
        # fresh window each time, so a filtered value never blends pre- and
        # post-transition readings. Must be configured before delta mode is
        # armed -- this runs ahead of SOUR:DELT:ARM in _setup_delta.
        if self._filter_type != "OFF":
            cmds.append(f":SENS:VOLT:AVER:TCON {self._filter_type}")
            if self._filter_window is not None:
                cmds.append(f":SENS:VOLT:AVER:WIND {self._filter_window:.6g}")
            cmds.append(f":SENS:VOLT:AVER:COUN {self._filter_count}")
            cmds.append(":SENS:VOLT:AVER:STAT ON")
        else:
            cmds.append(":SENS:VOLT:AVER:STAT OFF")
        cmds.append(":DISP:ENAB OFF")
        if line_sync:
            # Delta mode only: syncs 2182A readings to the AC line to reject
            # line-frequency noise between polarity reversals.
            cmds.append(":SYST:LSYN:STAT ON")
        for cmd in cmds:
            self.inst.write(f'SYST:COMM:SER:SEND "{cmd}"')
            time.sleep(0.1)
        time.sleep(0.5)

    def _set_2182a_range(self, voltage_range):
        """Push a new channel-1 voltage range to an already-connected 2182A
        (None = autorange). Used for live updates outside full setup."""
        if voltage_range is None:
            self.inst.write('SYST:COMM:SER:SEND ":SENS:VOLT:CHAN1:RANG:AUTO ON"')
        else:
            self.inst.write('SYST:COMM:SER:SEND ":SENS:VOLT:CHAN1:RANG:AUTO OFF"')
            time.sleep(0.1)
            self.inst.write(f'SYST:COMM:SER:SEND ":SENS:VOLT:CHAN1:RANG {voltage_range:.6e}"')
        time.sleep(0.1)

    def _set_2182a_filter(self, filter_type, filter_window, filter_count):
        """Push new digital averaging-filter settings to an already-connected
        2182A (type + window + count). Used for live updates outside full
        setup. Caller must send this before re-arming delta mode (SOUR:DELT:
        ARM) -- update_delta_settings only re-arms after calling this."""
        if filter_type != "OFF":
            self.inst.write(f'SYST:COMM:SER:SEND ":SENS:VOLT:AVER:TCON {filter_type}"')
            time.sleep(0.1)
            if filter_window is not None:
                self.inst.write(f'SYST:COMM:SER:SEND ":SENS:VOLT:AVER:WIND {filter_window:.6g}"')
                time.sleep(0.1)
            self.inst.write(f'SYST:COMM:SER:SEND ":SENS:VOLT:AVER:COUN {filter_count}"')
            time.sleep(0.1)
            self.inst.write('SYST:COMM:SER:SEND ":SENS:VOLT:AVER:STAT ON"')
        else:
            self.inst.write('SYST:COMM:SER:SEND ":SENS:VOLT:AVER:STAT OFF"')
        time.sleep(0.1)

    def _setup_delta(self):
        """Configure delta mode and arm. Called once on connect."""
        self.inst.write("*RST")
        time.sleep(1.5)
        self.inst.write("*CLS")
        self._configure_2182a(line_sync=True)
        # Restrict TRAC:DATA? to just the reading value. Without this, the
        # buffer's default format also includes a timestamp and reading
        # number per point, comma-separated alongside the voltage -- and
        # averaging those in with the real reading produces garbage that
        # drifts every cycle as the timestamp/index grow.
        self.inst.write("FORM:ELEM READ")
        # Compliance voltage limits the source — set before arming delta mode
        self.inst.write(f"CURR:COMP {abs(self._compliance):.6e}")
        # Configure delta mode on 6221
        self.inst.write(f"SOUR:DELT:HIGH {abs(self._delta_current):.6e}")
        self.inst.write(f"SOUR:DELT:LOW {-abs(self._delta_current):.6e}")
        self.inst.write(f"SOUR:DELT:DELAY {self._delay:.6e}")
        self.inst.write(f"SOUR:DELT:COUNT {self._count}")
        # CAB OFF: skip RS-232 calibration fetch from 2182A (avoids abort if
        # the serial link between instruments isn't fully configured yet)
        self.inst.write("SOUR:DELT:CAB OFF")
        # Arm once — each INIT:IMM re-triggers without needing to re-arm
        self.inst.write("SOUR:DELT:ARM")
        time.sleep(0.2)

    def _setup_dc(self):
        """Configure a plain single-polarity DC current output and read the
        2182A directly (no alternating-polarity thermal-EMF cancellation)."""
        self.inst.write("*RST")
        time.sleep(1.5)
        self.inst.write("*CLS")
        self._configure_2182a()
        self.inst.write(f"CURR:COMP {abs(self._compliance):.6e}")
        self.inst.write("SOUR:FUNC CURR")
        self.inst.write(f"SOUR:CURR:RANG {abs(self._delta_current):.6e}")
        self.inst.write(f"SOUR:CURR {self._delta_current:.6e}")
        self.inst.write("OUTP ON")
        time.sleep(0.2)

    def set_mode(self, mode):
        """Switch between delta mode and plain DC sourcing on a connected
        instrument, reconfiguring with the currently-stored settings."""
        if mode == self._mode:
            return
        if self.simulate:
            self._mode = mode
            return
        with self._lock:
            # Tear down whichever mode is currently active before reconfiguring.
            try:
                self.inst.write("SOUR:SWE:ABOR")
            except Exception:
                pass
            try:
                self.inst.write("OUTP OFF")
            except Exception:
                pass
            time.sleep(0.1)
            self._mode = mode
            if mode == "delta":
                self._setup_delta()
            else:
                self._setup_dc()

    def update_delta_settings(self, delta_current=None, delay=None, count=None,
                               nplc=None, compliance=None, voltage_range=_UNSET,
                               filter_type=None, filter_window=_UNSET, filter_count=None):
        """Push new source/measurement parameters to an already-connected
        instrument, in whichever mode (delta or DC) is currently active.

        Changing SOUR:DELT:* values requires disarming and re-arming delta
        mode — armed parameters are locked until SOUR:SWE:ABOR is sent.
        """
        if self.simulate:
            if delta_current is not None:
                self._delta_current = delta_current
            if delay is not None:
                self._delay = delay
            if count is not None:
                self._count = max(1, int(count))
            if nplc is not None:
                self._nplc = nplc
            if compliance is not None:
                self._compliance = compliance
            if voltage_range is not _UNSET:
                self._voltage_range = voltage_range
            if filter_type is not None:
                self._filter_type = filter_type
            if filter_window is not _UNSET:
                self._filter_window = filter_window
            if filter_count is not None:
                self._filter_count = max(2, int(filter_count))
            return

        with self._lock:
            if self._mode == "delta":
                self.inst.write("SOUR:SWE:ABOR")
                time.sleep(0.1)
                if compliance is not None:
                    self._compliance = compliance
                    self.inst.write(f"CURR:COMP {abs(compliance):.6e}")
                if delta_current is not None:
                    self._delta_current = delta_current
                    self.inst.write(f"SOUR:DELT:HIGH {abs(delta_current):.6e}")
                    self.inst.write(f"SOUR:DELT:LOW {-abs(delta_current):.6e}")
                if delay is not None:
                    self._delay = delay
                    self.inst.write(f"SOUR:DELT:DELAY {delay:.6e}")
                if count is not None:
                    self._count = max(1, int(count))
                    self.inst.write(f"SOUR:DELT:COUNT {self._count}")
                if nplc is not None:
                    self._nplc = nplc
                    self.inst.write(f'SYST:COMM:SER:SEND ":SENS:VOLT:NPLC {nplc:.3g}"')
                    time.sleep(0.1)
                if voltage_range is not _UNSET:
                    self._voltage_range = voltage_range
                    self._set_2182a_range(voltage_range)
                if filter_type is not None or filter_window is not _UNSET or filter_count is not None:
                    if filter_type is not None:
                        self._filter_type = filter_type
                    if filter_window is not _UNSET:
                        self._filter_window = filter_window
                    if filter_count is not None:
                        self._filter_count = max(2, int(filter_count))
                    # Must happen before re-arming below (SOUR:DELT:ARM) --
                    # filter settings are locked in once delta mode is armed.
                    self._set_2182a_filter(self._filter_type, self._filter_window, self._filter_count)
                self.inst.write("SOUR:DELT:CAB OFF")
                self.inst.write("SOUR:DELT:ARM")
                time.sleep(0.2)
            else:
                if compliance is not None:
                    self._compliance = compliance
                    self.inst.write(f"CURR:COMP {abs(compliance):.6e}")
                if delta_current is not None:
                    self._delta_current = delta_current
                    self.inst.write(f"SOUR:CURR:RANG {abs(delta_current):.6e}")
                    self.inst.write(f"SOUR:CURR {delta_current:.6e}")
                if nplc is not None:
                    self._nplc = nplc
                    self.inst.write(f'SYST:COMM:SER:SEND ":SENS:VOLT:NPLC {nplc:.3g}"')
                    time.sleep(0.1)
                if voltage_range is not _UNSET:
                    self._voltage_range = voltage_range
                    self._set_2182a_range(voltage_range)
                if filter_type is not None or filter_window is not _UNSET or filter_count is not None:
                    if filter_type is not None:
                        self._filter_type = filter_type
                    if filter_window is not _UNSET:
                        self._filter_window = filter_window
                    if filter_count is not None:
                        self._filter_count = max(2, int(filter_count))
                    self._set_2182a_filter(self._filter_type, self._filter_window, self._filter_count)

    def _flush(self):
        """Drain any unread bytes left in the socket receive buffer."""
        old_to = self.inst.timeout
        self.inst.timeout = 300
        try:
            while True:
                self.inst.read()
        except Exception:
            pass
        self.inst.timeout = old_to

    def identify(self):
        if self.simulate:
            return "SIMULATED Keithley 6221"
        with self._lock:
            return self.inst.query("*IDN?").strip()

    def read_resistance(self):
        if self.simulate:
            return 100.0 + 5.0 * math.sin(time.time() / 30.0) + random.gauss(0, 0.05)
        with self._lock:
            if self._mode == "delta":
                return self._read_resistance_delta()
            return self._read_resistance_dc()

    def _read_resistance_delta(self):
        try:
            # Clear trace buffer so TRAC:DATA? returns only this reading
            self.inst.write("TRAC:CLE")
            self.inst.write("INIT:IMM")
            self.inst.query("*OPC?")   # blocks until measurement completes
            raw = self.inst.query("TRAC:DATA?").strip()
            values = [float(v) for v in raw.split(",") if v.strip()]
            if any(abs(v) >= OVERFLOW_THRESHOLD for v in values):
                return float("nan")  # compliance tripped on this delta cycle
            voltage = sum(values) / len(values)
            # TRAC:DATA? returns the delta-mode *voltage* readings (the 2182A
            # is configured as a voltmeter above) — convert to resistance
            # using the known drive current, same as the DC-mode path.
            if self._delta_current == 0:
                return float("nan")
            return voltage / self._delta_current
        except Exception:
            # Delta mode may have been aborted — flush stale data, re-arm
            self._flush()
            self.inst.write("*CLS")
            self.inst.write("SOUR:DELT:ARM")
            time.sleep(0.2)
            raise

    def _read_resistance_dc(self):
        """Plain DC reading: source is already on continuously (OUTP ON from
        _setup_dc); just take a fresh voltage reading from the 2182A over the
        RS-232 pass-through and divide by the set current. No thermal-EMF
        cancellation, unlike delta mode."""
        self.inst.write('SYST:COMM:SER:SEND ":SENS:DATA:FRESH?"')
        time.sleep(max(self._nplc / 60.0, 0.02) + 0.05)
        raw = self.inst.query("SYST:COMM:SER:ENT?").strip()
        voltage = float(raw)
        if abs(voltage) >= OVERFLOW_THRESHOLD:
            return float("nan")  # compliance tripped, or sensor over-range
        if self._delta_current == 0:
            return float("nan")
        return voltage / self._delta_current

    def close(self):
        with self._lock:
            if self.inst is not None:
                try:
                    self.inst.write("SOUR:SWE:ABOR")
                    time.sleep(0.1)
                except Exception:
                    pass
                try:
                    self.inst.write("OUTP OFF")
                except Exception:
                    pass
                try:
                    self.inst.close()
                except Exception:
                    pass
                self.inst = None


class Keithley2400SourceMeter:
    """Keithley 2400-series SourceMeter (2400/2401/2410, or a 2450 left in
    2400 SCPI compatibility mode) — an alternative to the 6221+2182A path.
    Sources a fixed DC current over plain serial (RS-232/USB-CDC) and reads
    resistance directly via the instrument's built-in manual-ohms function
    (R = V/I, computed on-instrument from the sourced current), so unlike
    the 6221 paths there's no separate V/I division done in this script."""

    def __init__(self, resource, baud=9600, current=1e-3, compliance=10.0,
                 nplc=1.0, four_wire=False, simulate=False):
        self.simulate = simulate
        self._current = current
        self._compliance = compliance
        self._nplc = nplc
        self._four_wire = four_wire
        self.inst = None
        self._output_on = False
        # Guards every access to self.inst, same reason as Keithley6221's
        # lock: the GUI thread (Apply/settings) and the background logging
        # thread (read_resistance) both touch the same serial port.
        self._lock = threading.RLock()
        if simulate:
            self._output_on = True
            return
        import serial as _serial
        self.inst = _serial.Serial(
            port=resource, baudrate=baud, bytesize=8,
            parity="N", stopbits=1, timeout=5,
            dsrdtr=False, rtscts=False, xonxoff=False,
        )
        time.sleep(0.1)
        self.inst.reset_input_buffer()
        with self._lock:
            self._setup()
        self._output_on = True

    def _write(self, cmd):
        self.inst.write((cmd + "\n").encode())

    def _query(self, cmd):
        self._write(cmd)
        return self.inst.readline().decode("utf-8", errors="ignore").strip()

    def _check_error(self, context):
        """Query the instrument's error queue and raise with the offending
        command if anything is queued — otherwise a bad command here just
        surfaces as an opaque "-102" with no indication of which SCPI line
        caused it."""
        err = self._query(":SYST:ERR?")
        code_str = err.split(",", 1)[0].strip()
        try:
            code = int(code_str)
        except ValueError:
            return
        if code != 0:
            raise RuntimeError(f"Keithley 2400 rejected {context!r}: {err}")

    def _setup(self):
        self._write("*RST")
        time.sleep(0.5)
        self._write("*CLS")
        cmds = [
            f":SYST:RSEN {'ON' if self._four_wire else 'OFF'}",
            ":SOUR:FUNC CURR",
            ":SOUR:CURR:MODE FIXED",
            ":SOUR:CURR:RANG:AUTO ON",
            f":SOUR:CURR {self._current:.6e}",
            # Compliance for a current source is a voltage limit (aka "protection").
            f":SENS:VOLT:PROT {abs(self._compliance):.6e}",
            # Output must be on *before* manual ohms mode/range are configured
            # below: manual ohms derives resistance from the actual sourced
            # current, so configuring it while output is off leaves the ohms
            # range in a bad state that turning output on afterward doesn't
            # fix -- the first :READ? then fails with error 803 ("not
            # permitted with output off"), even though output is on by then.
            ":OUTP ON",
            ":SENS:FUNC 'RES'",
            # Manual ohms computes R = V/I using our sourced current, rather
            # than AUTO ohms which uses the instrument's own internal current
            # ranges. Some 2400 firmware rejects the partial abbreviation
            # "MANU" (-102 Syntax error) and only accepts "MAN" or the full
            # "MANUAL" — use the full word to be safe across firmware versions.
            ":SENS:RES:MODE MANUAL",
            ":SENS:RES:RANG:AUTO ON",
            f":SENS:RES:NPLC {self._nplc:.3g}",
            # Restrict :READ? to just the resistance value. The default format
            # bundles voltage, current, resistance, timestamp, and status all
            # comma-separated together -- parsing that without restricting it
            # is exactly the bug that corrupted the 6221's delta-mode readings.
            ":FORM:ELEM RES",
        ]
        for cmd in cmds:
            self._write(cmd)
            self._check_error(cmd)
        time.sleep(0.2)

    def identify(self):
        if self.simulate:
            return "SIMULATED Keithley 2400"
        with self._lock:
            return self._query("*IDN?")

    def read_resistance(self):
        if self.simulate:
            return 100.0 + 5.0 * math.sin(time.time() / 30.0) + random.gauss(0, 0.05)
        with self._lock:
            raw = self._query(":READ?")
            value = float(raw.split(",")[0])
            if abs(value) >= OVERFLOW_THRESHOLD:
                return float("nan")  # compliance tripped, or sensor over-range
            return value

    def update_settings(self, current=None, compliance=None, nplc=None, four_wire=None):
        """Push new source/measurement parameters to an already-connected
        instrument. Unlike the 6221's delta mode, nothing here needs to be
        disarmed first -- these commands take effect immediately."""
        if self.simulate:
            if current is not None:
                self._current = current
            if compliance is not None:
                self._compliance = compliance
            if nplc is not None:
                self._nplc = nplc
            if four_wire is not None:
                self._four_wire = four_wire
            return
        with self._lock:
            if four_wire is not None:
                self._four_wire = four_wire
                self._write(f":SYST:RSEN {'ON' if four_wire else 'OFF'}")
            if compliance is not None:
                self._compliance = compliance
                self._write(f":SENS:VOLT:PROT {abs(compliance):.6e}")
            if current is not None:
                self._current = current
                self._write(f":SOUR:CURR {current:.6e}")
            if nplc is not None:
                self._nplc = nplc
                self._write(f":SENS:RES:NPLC {nplc:.3g}")

    def set_output(self, on):
        """Turn the source output on or off. Manual ohms mode (set up in
        _setup) derives resistance from the sourced current, so :READ? will
        error while output is off -- callers should stop logging first."""
        if self.simulate:
            self._output_on = on
            return
        with self._lock:
            self._write(f":OUTP {'ON' if on else 'OFF'}")
            self._output_on = on

    def close(self):
        with self._lock:
            if self.inst is not None:
                try:
                    self._write(":OUTP OFF")
                except Exception:
                    pass
                self._output_on = False
                try:
                    self.inst.close()
                except Exception:
                    pass
                self.inst = None


class Cryocon22C:
    """Cryo-con Model 22C temperature controller — communicates via pyserial."""

    def __init__(self, resource, channel="A", baud=115200, simulate=False):
        self.simulate = simulate
        self.channel = channel.strip().upper()
        self.inst = None
        if simulate:
            return
        import serial as _serial
        self.inst = _serial.Serial(
            port=resource, baudrate=baud, bytesize=8,
            parity="N", stopbits=1, timeout=5,
            dsrdtr=False, rtscts=False, xonxoff=False,
        )
        time.sleep(0.1)
        self.inst.reset_input_buffer()

    def _query(self, cmd):
        self.inst.write((cmd + "\n").encode())
        return self.inst.readline().decode("utf-8").strip()

    def identify(self):
        if self.simulate:
            return "SIMULATED Cryo-con 22C"
        return self._query("*IDN?")

    def read_temperature(self):
        if self.simulate:
            # Fake a slow cooldown toward 4 K
            return 4.0 + 290.0 * math.exp(-(time.time() % 3600) / 600.0) + random.gauss(0, 0.02)
        reply = self._query(f"INP? {self.channel}")
        # Cryo-con returns e.g. "295.421" or dashes/letters if sensor fault
        try:
            return float(reply)
        except ValueError:
            return float("nan")

    def close(self):
        if self.inst is not None:
            try:
                self.inst.close()
            except Exception:
                pass
            self.inst = None


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

MAX_PLOT_POINTS = 5000  # cap in-memory/plotted history; full data always still goes to the CSV


class LoggerApp:
    def __init__(self, root):
        self.root = root
        root.title("Cryo Resistance Logger  —  Keithley 6221/2400 + Cryo-con 22C")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.keithley = None
        self.cryocon = None
        self.log_thread = None
        self.stop_event = threading.Event()
        self.data_queue = queue.Queue()

        # Data history for plots — capped so long (multi-day) sessions don't
        # grow redraw/memory cost without bound; the CSV file keeps every point.
        self.t_hist = deque(maxlen=MAX_PLOT_POINTS)
        self.temp_hist = deque(maxlen=MAX_PLOT_POINTS)
        self.res_hist = deque(maxlen=MAX_PLOT_POINTS)
        self.total_points = 0
        self.start_time = None

        self._build_gui()
        self.refresh_resources()
        self.root.after(200, self._poll_queue)

    # ------------------------------------------------------------------ GUI
    def _build_gui(self):
        pad = {"padx": 5, "pady": 3}

        left = ttk.Frame(self.root)
        left.grid(row=0, column=0, sticky="ns", **pad)
        right = ttk.Frame(self.root)
        right.grid(row=0, column=1, sticky="nsew", **pad)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # --- Instrument connection frame ---
        conn = ttk.LabelFrame(left, text="Instruments")
        conn.pack(fill="x", **pad)

        BAUDS = ["1200", "2400", "4800", "9600", "19200", "38400", "115200"]

        # --- Resistance source selector ---
        self.source_var = tk.StringVar(value="6221")
        srcf = ttk.Frame(conn)
        srcf.grid(row=0, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Label(srcf, text="Resistance source:").pack(side="left", padx=(0, 5))
        self.source_6221_radio = ttk.Radiobutton(
            srcf, text="Keithley 6221 (Delta/DC)", value="6221",
            variable=self.source_var, command=self._on_source_change)
        self.source_6221_radio.pack(side="left")
        self.source_2400_radio = ttk.Radiobutton(
            srcf, text="Keithley 2400-series (SourceMeter)", value="2400",
            variable=self.source_var, command=self._on_source_change)
        self.source_2400_radio.pack(side="left")

        # --- Keithley 6221 address (Ethernet) ---
        self.k6221_conn = ttk.Frame(conn)
        ttk.Label(self.k6221_conn, text="Keithley 6221 IP:").grid(row=0, column=0, sticky="e", **pad)
        self.keithley_addr = ttk.Entry(self.k6221_conn, width=20)
        self.keithley_addr.insert(0, "192.168.0.2")
        self.keithley_addr.grid(row=0, column=1, sticky="w", **pad)

        # --- Keithley 2400 address (serial) ---
        self.k2400_conn = ttk.Frame(conn)
        ttk.Label(self.k2400_conn, text="Keithley 2400 COM port:").grid(row=0, column=0, sticky="e", **pad)
        self.k2400_addr = ttk.Combobox(self.k2400_conn, width=14)
        self.k2400_addr.grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(self.k2400_conn, text="Baud rate:").grid(row=1, column=0, sticky="e", **pad)
        self.k2400_baud_var = tk.StringVar(value="9600")
        ttk.Combobox(self.k2400_conn, textvariable=self.k2400_baud_var, values=BAUDS,
                     width=8, state="readonly").grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(conn, text="Cryo-con 22C:").grid(row=2, column=0, sticky="e", **pad)
        self.cryocon_addr = ttk.Combobox(conn, width=14)
        self.cryocon_addr.grid(row=2, column=1, sticky="w", **pad)

        ttk.Label(conn, text="Cryo-con channel:").grid(row=3, column=0, sticky="e", **pad)
        self.cryocon_chan = ttk.Combobox(conn, width=5, values=["A", "B"], state="readonly")
        self.cryocon_chan.set("A")
        self.cryocon_chan.grid(row=3, column=1, sticky="w", **pad)

        self.simulate_keithley_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(conn, text="Simulate Keithley",
                        variable=self.simulate_keithley_var).grid(row=4, column=0,
                                                                   sticky="w", **pad)

        self.simulate_cryocon_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(conn, text="Simulate Cryo-con",
                        variable=self.simulate_cryocon_var).grid(row=4, column=1,
                                                                  sticky="w", **pad)

        # --- Cryo-con serial settings ---
        ser = ttk.LabelFrame(conn, text="Cryo-con Serial")
        ser.grid(row=5, column=0, columnspan=3, sticky="ew", **pad)

        ttk.Label(ser, text="Baud rate:").grid(row=0, column=0, sticky="e", **pad)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(ser, textvariable=self.baud_var, values=BAUDS,
                     width=8, state="readonly").grid(row=0, column=1, sticky="w", **pad)

        # --- Keithley 6221 source settings ---
        kdelta = ttk.LabelFrame(conn, text="Keithley 6221 — Source Settings")
        self.kdelta = kdelta

        self.mode_var = tk.StringVar(value="delta")
        modef = ttk.Frame(kdelta)
        modef.grid(row=0, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Label(modef, text="Mode:").pack(side="left", padx=(0, 5))
        self.mode_btn = ttk.Button(modef, text="Delta Mode (click for DC)",
                                   command=self.toggle_mode)
        self.mode_btn.pack(side="left")

        ttk.Label(kdelta, text="Current:").grid(row=1, column=0, sticky="e", **pad)
        self.delta_current_var = tk.StringVar(value="1.0")
        ttk.Entry(kdelta, textvariable=self.delta_current_var, width=8).grid(
                  row=1, column=1, **pad)
        ttk.Label(kdelta, text="mA  (±I in Delta; DC level in DC mode)").grid(
                  row=1, column=2, sticky="w")

        ttk.Label(kdelta, text="Settling delay:").grid(row=2, column=0, sticky="e", **pad)
        self.delta_delay_var = tk.StringVar(value="2.0")
        ttk.Entry(kdelta, textvariable=self.delta_delay_var, width=8).grid(
                  row=2, column=1, **pad)
        ttk.Label(kdelta, text="ms  (Delta mode only)").grid(row=2, column=2, sticky="w")

        ttk.Label(kdelta, text="Readings/point:").grid(row=3, column=0, sticky="e", **pad)
        self.delta_count_var = tk.StringVar(value="1")
        ttk.Entry(kdelta, textvariable=self.delta_count_var, width=8).grid(
                  row=3, column=1, **pad)
        ttk.Label(kdelta, text="(averaged, Delta mode only)").grid(row=3, column=2, sticky="w")

        ttk.Label(kdelta, text="Rate (NPLC):").grid(row=4, column=0, sticky="e", **pad)
        self.delta_nplc_var = tk.StringVar(value="5.0")
        ttk.Entry(kdelta, textvariable=self.delta_nplc_var, width=8).grid(
                  row=4, column=1, **pad)
        ttk.Label(kdelta, text="power line cycles (2182A integration time)").grid(
                  row=4, column=2, sticky="w")

        ttk.Label(kdelta, text="Compliance voltage:").grid(row=6, column=0, sticky="e", **pad)
        self.compliance_var = tk.StringVar(value="10.0")
        ttk.Entry(kdelta, textvariable=self.compliance_var, width=8).grid(
                  row=6, column=1, **pad)
        ttk.Label(kdelta, text="V").grid(row=6, column=2, sticky="w")

        ttk.Label(kdelta, text="NV meter voltage range:").grid(row=7, column=0, sticky="e", **pad)
        self.k6221_range_var = tk.StringVar(value="Auto")
        ttk.Combobox(kdelta, textvariable=self.k6221_range_var,
                     values=list(NANOVOLTMETER_RANGES.keys()), width=8,
                     state="readonly").grid(row=7, column=1, sticky="w", **pad)
        ttk.Label(kdelta, text="(2182A ch.1; fix if autorange overflows)").grid(
                  row=7, column=2, sticky="w")

        ttk.Label(kdelta, text="Filter type:").grid(row=8, column=0, sticky="e", **pad)
        self.filter_type_var = tk.StringVar(value="Moving Average")
        ttk.Combobox(kdelta, textvariable=self.filter_type_var,
                     values=list(FILTER_TYPES.keys()), width=16,
                     state="readonly").grid(row=8, column=1, sticky="w", **pad)
        ttk.Label(kdelta, text="2182A digital averaging filter").grid(
                  row=8, column=2, sticky="w")

        ttk.Label(kdelta, text="Filter window:").grid(row=9, column=0, sticky="e", **pad)
        self.filter_window_var = tk.StringVar(value="None")
        ttk.Entry(kdelta, textvariable=self.filter_window_var, width=8).grid(
                  row=9, column=1, **pad)
        ttk.Label(kdelta, text="% of range, 0-10, or None (2182A default)").grid(
                  row=9, column=2, sticky="w")

        ttk.Label(kdelta, text="Filter count:").grid(row=10, column=0, sticky="e", **pad)
        self.filter_count_var = tk.StringVar(value="10")
        ttk.Entry(kdelta, textvariable=self.filter_count_var, width=8).grid(
                  row=10, column=1, **pad)
        ttk.Label(kdelta, text="readings averaged (2-100)").grid(
                  row=10, column=2, sticky="w")

        self.apply_delta_btn = ttk.Button(kdelta, text="Apply to instrument",
                                          command=self.apply_delta_settings,
                                          state="disabled")
        self.apply_delta_btn.grid(row=11, column=0, columnspan=3, **pad)

        # --- Keithley 2400 source settings ---
        k2400 = ttk.LabelFrame(conn, text="Keithley 2400 — Source Settings")
        self.k2400_settings = k2400

        ttk.Label(k2400, text="Current:").grid(row=0, column=0, sticky="e", **pad)
        self.k2400_current_var = tk.StringVar(value="1.0")
        ttk.Entry(k2400, textvariable=self.k2400_current_var, width=8).grid(
                  row=0, column=1, **pad)
        ttk.Label(k2400, text="mA").grid(row=0, column=2, sticky="w")

        ttk.Label(k2400, text="Compliance voltage:").grid(row=1, column=0, sticky="e", **pad)
        self.k2400_compliance_var = tk.StringVar(value="10.0")
        ttk.Entry(k2400, textvariable=self.k2400_compliance_var, width=8).grid(
                  row=1, column=1, **pad)
        ttk.Label(k2400, text="V").grid(row=1, column=2, sticky="w")

        ttk.Label(k2400, text="NPLC:").grid(row=2, column=0, sticky="e", **pad)
        self.k2400_nplc_var = tk.StringVar(value="1.0")
        ttk.Entry(k2400, textvariable=self.k2400_nplc_var, width=8).grid(
                  row=2, column=1, **pad)
        ttk.Label(k2400, text="(integration time)").grid(row=2, column=2, sticky="w")

        self.k2400_4wire_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(k2400, text="4-wire (Kelvin) sensing",
                        variable=self.k2400_4wire_var).grid(row=3, column=0,
                                                             columnspan=2, sticky="w", **pad)

        self.k2400_output_var = tk.BooleanVar(value=True)
        self.k2400_output_btn = ttk.Button(k2400, text="Output: ON (click to turn OFF)",
                                           command=self.toggle_k2400_output,
                                           state="disabled")
        self.k2400_output_btn.grid(row=4, column=0, columnspan=3, **pad)

        self.apply_k2400_btn = ttk.Button(k2400, text="Apply to instrument",
                                          command=self.apply_k2400_settings,
                                          state="disabled")
        self.apply_k2400_btn.grid(row=5, column=0, columnspan=3, **pad)

        btnrow = ttk.Frame(conn)
        btnrow.grid(row=7, column=0, columnspan=3, **pad)
        ttk.Button(btnrow, text="Rescan bus", command=self.refresh_resources).pack(side="left", padx=3)
        self.connect_btn = ttk.Button(btnrow, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left", padx=3)
        self.disconnect_btn = ttk.Button(btnrow, text="Disconnect",
                                         command=self.disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=3)

        self._on_source_change()

        # --- Data file frame ---
        filef = ttk.LabelFrame(left, text="Data file")
        filef.pack(fill="x", **pad)
        self.file_var = tk.StringVar(value="")
        ttk.Entry(filef, textvariable=self.file_var, width=38).grid(row=0, column=0, **pad)
        ttk.Button(filef, text="Browse…", command=self.choose_file).grid(row=0, column=1, **pad)

        # --- Logging control frame ---
        ctrl = ttk.LabelFrame(left, text="Logging")
        ctrl.pack(fill="x", **pad)
        ttk.Label(ctrl, text="Interval (s):").grid(row=0, column=0, sticky="e", **pad)
        self.interval_var = tk.StringVar(value="5")
        ttk.Entry(ctrl, textvariable=self.interval_var, width=8).grid(row=0, column=1,
                                                                      sticky="w", **pad)
        ttk.Label(ctrl, text="(can be changed while running)").grid(row=0, column=2,
                                                                    sticky="w", **pad)
        self.start_btn = ttk.Button(ctrl, text="Start logging", command=self.start_logging,
                                    state="disabled")
        self.start_btn.grid(row=1, column=0, columnspan=1, **pad)
        self.stop_btn = ttk.Button(ctrl, text="Stop", command=self.stop_logging,
                                   state="disabled")
        self.stop_btn.grid(row=1, column=1, **pad)
        self.clear_btn = ttk.Button(ctrl, text="Clear plots", command=self.clear_plots)
        self.clear_btn.grid(row=1, column=2, **pad)

        ttk.Label(ctrl, text="Resistance scale:").grid(row=2, column=0, sticky="e", **pad)
        self.res_scale_var = tk.StringVar(value="linear")
        scalef = ttk.Frame(ctrl)
        scalef.grid(row=2, column=1, columnspan=2, sticky="w", **pad)
        ttk.Radiobutton(scalef, text="Linear", value="linear", variable=self.res_scale_var,
                        command=self._on_scale_change).pack(side="left")
        ttk.Radiobutton(scalef, text="Log", value="log", variable=self.res_scale_var,
                        command=self._on_scale_change).pack(side="left")

        # --- Axis range frame ---
        rng = ttk.LabelFrame(left, text="Axis ranges")
        rng.pack(fill="x", **pad)

        self.temp_auto_var = tk.BooleanVar(value=True)
        self.temp_min_var = tk.StringVar(value="")
        self.temp_max_var = tk.StringVar(value="")
        self.res_auto_var = tk.BooleanVar(value=True)
        self.res_min_var = tk.StringVar(value="")
        self.res_max_var = tk.StringVar(value="")
        self.time_auto_var = tk.BooleanVar(value=True)
        self.time_min_var = tk.StringVar(value="")
        self.time_max_var = tk.StringVar(value="")

        def _range_row(row, label, auto_var, min_var, max_var):
            ttk.Label(rng, text=label).grid(row=row, column=0, sticky="e", **pad)
            ttk.Checkbutton(rng, text="Auto", variable=auto_var,
                             command=self._redraw).grid(row=row, column=1, sticky="w")
            f = ttk.Frame(rng)
            f.grid(row=row, column=2, sticky="w", **pad)
            e_min = ttk.Entry(f, textvariable=min_var, width=8)
            e_min.pack(side="left")
            ttk.Label(f, text=" to ").pack(side="left")
            e_max = ttk.Entry(f, textvariable=max_var, width=8)
            e_max.pack(side="left")
            e_min.bind("<Return>", lambda e: self._redraw())
            e_max.bind("<Return>", lambda e: self._redraw())

        _range_row(0, "Temperature (K):", self.temp_auto_var, self.temp_min_var, self.temp_max_var)
        _range_row(1, "Resistance (Ω):", self.res_auto_var, self.res_min_var, self.res_max_var)
        _range_row(2, "Elapsed time (min):", self.time_auto_var, self.time_min_var, self.time_max_var)
        ttk.Button(rng, text="Apply ranges", command=self._redraw).grid(
            row=3, column=0, columnspan=3, **pad)

        # --- Current readings frame ---
        cur = ttk.LabelFrame(left, text="Current reading")
        cur.pack(fill="x", **pad)
        big = ("TkDefaultFont", 16, "bold")
        ttk.Label(cur, text="Time:").grid(row=0, column=0, sticky="e", **pad)
        self.time_lbl = ttk.Label(cur, text="—", font=big)
        self.time_lbl.grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(cur, text="Temperature:").grid(row=1, column=0, sticky="e", **pad)
        self.temp_lbl = ttk.Label(cur, text="—", font=big, foreground="#0055cc")
        self.temp_lbl.grid(row=1, column=1, sticky="w", **pad)
        ttk.Label(cur, text="Resistance:").grid(row=2, column=0, sticky="e", **pad)
        self.res_lbl = ttk.Label(cur, text="—", font=big, foreground="#cc3300")
        self.res_lbl.grid(row=2, column=1, sticky="w", **pad)
        ttk.Label(cur, text="Points logged:").grid(row=3, column=0, sticky="e", **pad)
        self.count_lbl = ttk.Label(cur, text="0")
        self.count_lbl.grid(row=3, column=1, sticky="w", **pad)

        # --- Status bar ---
        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(left, textvariable=self.status_var, wraplength=300,
                  foreground="#555555").pack(fill="x", **pad)

        # --- Plots ---
        # --- R vs T plot (most important — shown first/on top) ---
        self.fig_rt = Figure(figsize=(7, 5), dpi=100)
        self.ax_rt = self.fig_rt.add_subplot(111)
        self.ax_rt.set_xlabel("Temperature (K)")
        self.ax_rt.set_ylabel("Resistance (Ω)")
        self.ax_rt.set_title("R vs T")
        self.rt_line, = self.ax_rt.plot([], [], "g.-", ms=3, lw=0.8)
        self.ax_rt.grid(True, alpha=0.3)
        self._plain_format(self.ax_rt.xaxis)
        self._plain_format(self.ax_rt.yaxis)
        self.fig_rt.tight_layout()
        self.canvas_rt = FigureCanvasTkAgg(self.fig_rt, master=right)
        self.canvas_rt.get_tk_widget().pack(fill="both", expand=True)

        # --- Time-series plots (temperature and resistance vs elapsed time) ---
        self.fig = Figure(figsize=(7, 3), dpi=100)
        self.ax_temp = self.fig.add_subplot(211)
        self.ax_res = self.fig.add_subplot(212, sharex=self.ax_temp)
        self.ax_temp.set_ylabel("Temperature (K)")
        self.ax_res.set_ylabel("Resistance (Ω)")
        self.ax_res.set_xlabel("Elapsed time (min)")
        self.temp_line, = self.ax_temp.plot([], [], "b.-", ms=3, lw=0.8)
        self.res_line, = self.ax_res.plot([], [], "r.-", ms=3, lw=0.8)
        self.ax_temp.grid(True, alpha=0.3)
        self.ax_res.grid(True, alpha=0.3)
        self._plain_format(self.ax_temp.yaxis)
        self._plain_format(self.ax_res.yaxis)
        self._plain_format(self.ax_res.xaxis)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # -------------------------------------------------------------- actions
    def refresh_resources(self):
        resources = []
        # Prefer pyserial to enumerate real COM ports
        if _list_ports is not None:
            try:
                ports = sorted(_list_ports.comports(), key=lambda p: p.device)
                resources = [p.device for p in ports]  # e.g. ["COM3", "COM5"]
            except Exception as e:
                self.status_var.set(f"COM port scan failed: {e}")
        # Fall back to pyvisa ASRL resources if pyserial unavailable
        if not resources and pyvisa is not None:
            try:
                rm = pyvisa.ResourceManager()
                resources = [r for r in rm.list_resources() if r.startswith("ASRL")]
            except Exception as e:
                self.status_var.set(f"VISA scan failed: {e}")
        # Last-resort fallback: offer COM1-COM10
        fallback = [f"COM{n}" for n in range(1, 11)]
        values = resources if resources else fallback
        self.cryocon_addr["values"] = values
        self.k2400_addr["values"] = values
        if resources:
            self.status_var.set(f"Found {len(resources)} COM port(s): {', '.join(resources)}")
        else:
            self.status_var.set("No COM ports found — check cabling, "
                                "or type a port manually (e.g. COM4).")
        if not self.cryocon_addr.get():
            self.cryocon_addr.set(values[-1] if values else "")
        if not self.k2400_addr.get():
            self.k2400_addr.set(values[-1] if values else "")

    def _on_source_change(self):
        """Show only the address/settings frames for the selected resistance
        source instrument."""
        pad = {"padx": 5, "pady": 3}
        if self.source_var.get() == "6221":
            self.k2400_conn.grid_remove()
            self.k2400_settings.grid_remove()
            self.k6221_conn.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)
            self.kdelta.grid(row=6, column=0, columnspan=3, sticky="ew", **pad)
        else:
            self.k6221_conn.grid_remove()
            self.kdelta.grid_remove()
            self.k2400_conn.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)
            self.k2400_settings.grid(row=6, column=0, columnspan=3, sticky="ew", **pad)

    @staticmethod
    def _parse_filter_window(text):
        """Parse the filter-window field: blank/"None" means unset (leave the
        2182A at its own default); otherwise a 0-10 percent value."""
        text = text.strip()
        if not text or text.lower() == "none":
            return None
        return float(text)

    def connect(self):
        source = self.source_var.get()
        baud = int(self.baud_var.get())
        try:
            if source == "6221":
                delta_current = float(self.delta_current_var.get()) * 1e-3  # mA → A
                delta_delay = float(self.delta_delay_var.get()) * 1e-3       # ms → s
                delta_count = int(self.delta_count_var.get())
                delta_nplc = float(self.delta_nplc_var.get())
                compliance = float(self.compliance_var.get())
                filter_window = self._parse_filter_window(self.filter_window_var.get())
                filter_count = int(self.filter_count_var.get())
            else:
                k2400_current = float(self.k2400_current_var.get()) * 1e-3  # mA → A
                k2400_compliance = float(self.k2400_compliance_var.get())
                k2400_nplc = float(self.k2400_nplc_var.get())
        except ValueError:
            messagebox.showerror("Bad value", "Source parameters must be numbers.")
            return
        try:
            if source == "6221":
                self.keithley = Keithley6221(self.keithley_addr.get(),
                                             delta_current=delta_current,
                                             delay=delta_delay,
                                             count=delta_count,
                                             nplc=delta_nplc,
                                             compliance=compliance,
                                             mode=self.mode_var.get(),
                                             voltage_range=NANOVOLTMETER_RANGES[self.k6221_range_var.get()],
                                             filter_type=FILTER_TYPES[self.filter_type_var.get()],
                                             filter_window=filter_window,
                                             filter_count=filter_count,
                                             simulate=self.simulate_keithley_var.get())
                k_id = self.keithley.identify()
            else:
                self.keithley = Keithley2400SourceMeter(
                    self.k2400_addr.get(),
                    baud=int(self.k2400_baud_var.get()),
                    current=k2400_current,
                    compliance=k2400_compliance,
                    nplc=k2400_nplc,
                    four_wire=self.k2400_4wire_var.get(),
                    simulate=self.simulate_keithley_var.get())
                k_id = self.keithley.identify()
            self.cryocon = Cryocon22C(self.cryocon_addr.get(),
                                      channel=self.cryocon_chan.get(),
                                      baud=baud,
                                      simulate=self.simulate_cryocon_var.get())
            c_id = self.cryocon.identify()
        except Exception as e:
            self.disconnect()
            messagebox.showerror("Connection failed", str(e))
            self.status_var.set(f"Connection failed: {e}")
            return
        self.status_var.set(f"Connected.\nKeithley: {k_id}\nCryo-con: {c_id}")
        self.connect_btn["state"] = "disabled"
        self.disconnect_btn["state"] = "normal"
        self.start_btn["state"] = "normal"
        self.source_6221_radio["state"] = "disabled"
        self.source_2400_radio["state"] = "disabled"
        if source == "6221":
            self.apply_delta_btn["state"] = "normal"
        else:
            self.apply_k2400_btn["state"] = "normal"
            self.k2400_output_btn["state"] = "normal"
            self.k2400_output_var.set(True)
            self.k2400_output_btn.config(text="Output: ON (click to turn OFF)")

    def disconnect(self):
        self.stop_logging()
        for dev in (self.keithley, self.cryocon):
            if dev is not None:
                dev.close()
        self.keithley = self.cryocon = None
        self.connect_btn["state"] = "normal"
        self.disconnect_btn["state"] = "disabled"
        self.start_btn["state"] = "disabled"
        self.apply_delta_btn["state"] = "disabled"
        self.apply_k2400_btn["state"] = "disabled"
        self.k2400_output_btn["state"] = "disabled"
        self.k2400_output_var.set(True)
        self.k2400_output_btn.config(text="Output: ON (click to turn OFF)")
        self.source_6221_radio["state"] = "normal"
        self.source_2400_radio["state"] = "normal"
        self.status_var.set("Disconnected.")

    def apply_delta_settings(self):
        if self.keithley is None:
            return
        try:
            delta_current = float(self.delta_current_var.get()) * 1e-3  # mA → A
            delta_delay = float(self.delta_delay_var.get()) * 1e-3       # ms → s
            delta_count = int(self.delta_count_var.get())
            delta_nplc = float(self.delta_nplc_var.get())
            compliance = float(self.compliance_var.get())
            filter_window = self._parse_filter_window(self.filter_window_var.get())
            filter_count = int(self.filter_count_var.get())
        except ValueError:
            messagebox.showerror("Bad value", "Delta mode parameters must be numbers.")
            return
        voltage_range = NANOVOLTMETER_RANGES[self.k6221_range_var.get()]
        filter_type = FILTER_TYPES[self.filter_type_var.get()]
        try:
            self.keithley.update_delta_settings(
                delta_current=delta_current, delay=delta_delay,
                count=delta_count, nplc=delta_nplc, compliance=compliance,
                voltage_range=voltage_range, filter_type=filter_type,
                filter_window=filter_window, filter_count=filter_count)
        except Exception as e:
            messagebox.showerror("Apply failed", str(e))
            self.status_var.set(f"Failed to apply delta settings: {e}")
            return
        self.status_var.set(
            f"Applied: {delta_current*1e3:.3f} mA, {compliance:.2f} V compliance, "
            f"{delta_delay*1e3:.2f} ms delay, {delta_count} readings, NPLC {delta_nplc:.3g}, "
            f"range {self.k6221_range_var.get()}, "
            f"filter {self.filter_type_var.get()} ({filter_count}, window={filter_window})")

    def apply_k2400_settings(self):
        if self.keithley is None:
            return
        try:
            current = float(self.k2400_current_var.get()) * 1e-3  # mA → A
            compliance = float(self.k2400_compliance_var.get())
            nplc = float(self.k2400_nplc_var.get())
        except ValueError:
            messagebox.showerror("Bad value", "Source parameters must be numbers.")
            return
        try:
            self.keithley.update_settings(
                current=current, compliance=compliance, nplc=nplc,
                four_wire=self.k2400_4wire_var.get())
        except Exception as e:
            messagebox.showerror("Apply failed", str(e))
            self.status_var.set(f"Failed to apply settings: {e}")
            return
        self.status_var.set(
            f"Applied: {current*1e3:.3f} mA, {compliance:.2f} V compliance, NPLC {nplc:.3g}")

    def toggle_mode(self):
        if not isinstance(self.keithley, Keithley6221) and self.keithley is not None:
            return
        new_mode = "dc" if self.mode_var.get() == "delta" else "delta"
        if self.keithley is not None:
            try:
                self.keithley.set_mode(new_mode)
            except Exception as e:
                messagebox.showerror("Mode switch failed", str(e))
                self.status_var.set(f"Failed to switch mode: {e}")
                return
            self.status_var.set(f"Switched Keithley 6221 to {new_mode.upper()} mode.")
        self.mode_var.set(new_mode)
        if new_mode == "delta":
            self.mode_btn.config(text="Delta Mode (click for DC)")
        else:
            self.mode_btn.config(text="DC Mode (click for Delta)")

    def toggle_k2400_output(self):
        if not isinstance(self.keithley, Keithley2400SourceMeter):
            return
        new_state = not self.k2400_output_var.get()
        try:
            self.keithley.set_output(new_state)
        except Exception as e:
            messagebox.showerror("Output toggle failed", str(e))
            self.status_var.set(f"Failed to toggle Keithley 2400 output: {e}")
            return
        self.k2400_output_var.set(new_state)
        self.k2400_output_btn.config(
            text=f"Output: {'ON' if new_state else 'OFF'} "
                 f"(click to turn {'OFF' if new_state else 'ON'})")
        self.status_var.set(f"Keithley 2400 output turned {'ON' if new_state else 'OFF'}.")

    def choose_file(self):
        default = datetime.now().strftime("cryo_log_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            title="Choose data file",
            initialfile=default,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("Text files", "*.txt"),
                       ("All files", "*.*")])
        if path:
            self.file_var.set(path)

    def start_logging(self):
        if not self.file_var.get():
            self.choose_file()
            if not self.file_var.get():
                return
        try:
            interval = float(self.interval_var.get())
            if interval <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Bad interval", "Interval must be a positive number of seconds.")
            return
        path = self.file_var.get()
        try:
            new_file = not os.path.exists(path) or os.path.getsize(path) == 0
            self.log_file = open(path, "a", newline="")
            self.log_writer = csv.writer(self.log_file)
            if new_file:
                self.log_writer.writerow(
                    ["timestamp", "temperature_K", "resistance_ohm"])
                self.log_file.flush()
        except OSError as e:
            messagebox.showerror("File error", f"Cannot open file:\n{e}")
            return

        self.stop_event.clear()
        self.start_time = time.time()
        self.log_thread = threading.Thread(target=self._log_loop, daemon=True)
        self.log_thread.start()
        self.start_btn["state"] = "disabled"
        self.stop_btn["state"] = "normal"
        self.status_var.set(f"Logging to {path}")

    def stop_logging(self):
        self.stop_event.set()
        if self.log_thread is not None:
            self.log_thread.join(timeout=10)
            self.log_thread = None
        if getattr(self, "log_file", None):
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None
        self.start_btn["state"] = "normal" if self.keithley else "disabled"
        self.stop_btn["state"] = "disabled"
        if self.keithley:
            self.status_var.set("Logging stopped. Still connected.")

    def clear_plots(self):
        self.t_hist.clear()
        self.temp_hist.clear()
        self.res_hist.clear()
        self.total_points = 0
        self._redraw()
        self.count_lbl["text"] = "0"

    @staticmethod
    def _plain_format(axis):
        """Force plain (non-scientific, no offset) tick labels, e.g. 150000
        instead of 1.5e5 / a "1e5" offset box in the corner."""
        fmt = ScalarFormatter(useOffset=False)
        fmt.set_scientific(False)
        axis.set_major_formatter(fmt)

    def _on_scale_change(self):
        """Switch the resistance axes (R-vs-T and the time-series R plot)
        between linear and log scale. Useful for superconducting transitions,
        where R can span several orders of magnitude — points that are zero
        or negative (noise around a superconducting R≈0) simply don't show
        up on a log axis, same as any other matplotlib log plot."""
        scale = self.res_scale_var.get()
        self.ax_rt.set_yscale(scale)
        self.ax_res.set_yscale(scale)
        if scale == "linear":
            # set_yscale resets the formatter, so plain formatting must be
            # re-applied; log scale keeps matplotlib's own power-of-ten labels.
            self._plain_format(self.ax_rt.yaxis)
            self._plain_format(self.ax_res.yaxis)
        self._redraw()

    @staticmethod
    def _parse_range(min_var, max_var):
        try:
            lo = float(min_var.get())
            hi = float(max_var.get())
        except ValueError:
            return None
        if lo >= hi:
            return None
        return lo, hi

    # ------------------------------------------------------ background loop
    def _log_loop(self):
        """Runs in a background thread; instruments are only touched here."""
        while not self.stop_event.is_set():
            loop_start = time.time()
            try:
                temp = self.cryocon.read_temperature()
                res = self.keithley.read_resistance()
            except Exception as e:
                self.data_queue.put(("error", str(e)))
                break
            now = datetime.now()
            elapsed = time.time() - self.start_time
            row = [now.strftime("%Y-%m-%d %H:%M:%S"), f"{temp:.4f}", f"{res:.6e}"]
            try:
                self.log_writer.writerow(row)
                self.log_file.flush()
            except Exception as e:
                self.data_queue.put(("error", f"File write failed: {e}"))
                break
            self.data_queue.put(("data", now, elapsed, temp, res))
            # Re-read interval each cycle so it can be adjusted live
            try:
                interval = max(0.2, float(self.interval_var.get()))
            except ValueError:
                interval = 5.0
            # Sleep in small chunks so Stop responds quickly
            while not self.stop_event.is_set() and time.time() - loop_start < interval:
                time.sleep(0.1)

    # ------------------------------------------------------------ GUI update
    def _poll_queue(self):
        try:
            while True:
                msg = self.data_queue.get_nowait()
                if msg[0] == "error":
                    self.status_var.set(f"ERROR: {msg[1]} — logging stopped.")
                    self.stop_logging()
                else:
                    _, now, elapsed, temp, res = msg
                    self.time_lbl["text"] = now.strftime("%H:%M:%S")
                    self.temp_lbl["text"] = f"{temp:.3f} K"
                    self.res_lbl["text"] = self._fmt_res(res)
                    self.t_hist.append(elapsed / 60.0)
                    self.temp_hist.append(temp)
                    self.res_hist.append(res)
                    self.total_points += 1
                    self.count_lbl["text"] = str(self.total_points)
                    self._redraw()
        except queue.Empty:
            pass
        self.root.after(200, self._poll_queue)

    @staticmethod
    def _fmt_res(res):
        if math.isnan(res):
            return "—"
        if abs(res) >= 1e6:
            return f"{res / 1e6:.6f} MΩ"
        if abs(res) >= 1e3:
            return f"{res / 1e3:.6f} kΩ"
        if abs(res) < 1:
            return f"{res * 1e3:.6f} mΩ"
        return f"{res:.6f} Ω"

    def _redraw(self):
        self.temp_line.set_data(self.t_hist, self.temp_hist)
        self.res_line.set_data(self.t_hist, self.res_hist)
        for ax in (self.ax_temp, self.ax_res):
            ax.relim()
            ax.autoscale_view()

        if not self.time_auto_var.get():
            r = self._parse_range(self.time_min_var, self.time_max_var)
            if r:
                self.ax_res.set_xlim(*r)  # shared x-axis with ax_temp
        if not self.temp_auto_var.get():
            r = self._parse_range(self.temp_min_var, self.temp_max_var)
            if r:
                self.ax_temp.set_ylim(*r)
        if not self.res_auto_var.get():
            r = self._parse_range(self.res_min_var, self.res_max_var)
            if r:
                self.ax_res.set_ylim(*r)
        self.canvas.draw_idle()

        self.rt_line.set_data(self.temp_hist, self.res_hist)
        self.ax_rt.relim()
        self.ax_rt.autoscale_view()

        if not self.temp_auto_var.get():
            r = self._parse_range(self.temp_min_var, self.temp_max_var)
            if r:
                self.ax_rt.set_xlim(*r)
        if not self.res_auto_var.get():
            r = self._parse_range(self.res_min_var, self.res_max_var)
            if r:
                self.ax_rt.set_ylim(*r)
        self.canvas_rt.draw_idle()

    def on_close(self):
        self.stop_logging()
        self.disconnect()
        self.root.destroy()


def main():
    if pyvisa is None:
        print("WARNING: pyvisa is not installed — only simulation mode will work.")
    root = tk.Tk()
    LoggerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
