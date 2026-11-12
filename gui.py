#!/usr/bin/env python3
"""
SX1280 QO-100 SSB TX Control GUI
=================================
Modern, responsive GUI for controlling the SX1280 SDR transmitter.
Supports all CDC commands and real-time parameter adjustment.

Author: SP8ESA
License: CC BY-NC 4.0
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import queue
import re
from dataclasses import dataclass
from typing import Optional, Callable

# ---- serial
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False


# ============================================================
# CONFIGURATION DATACLASS
# ============================================================

@dataclass
class TxConfig:
    """Mirrors the firmware's audio_cfg_t structure"""
    # RF
    freq_hz: float = 2_400_400_000.0
    ppm: float = 0.0
    tx_power_dbm: int = 13
    tx_enabled: bool = False
    # Enables
    enable_bp: bool = True
    enable_eq: bool = True
    enable_comp: bool = True
    # Bandpass
    bp_lo_hz: float = 50.0
    bp_hi_hz: float = 2700.0
    bp_stages: int = 7
    # EQ (Shelving)
    eq_low_hz: float = 190.0
    eq_low_db: float = -2.0
    eq_high_hz: float = 1700.0
    eq_high_db: float = 13.5
    # Compressor
    comp_thr_db: float = -2.5
    comp_ratio: float = 6.1
    comp_attack_ms: float = 41.1
    comp_release_ms: float = 1595.0
    comp_makeup_db: float = 0.0
    comp_knee_db: float = 16.5
    comp_out_limit: float = 0.940
    # Power shaping
    amp_gain: float = 2.9
    amp_min_a: float = 0.000002
    # MIC AGC
    mic_agc_target: float = 0.75
    mic_agc_max_gain: float = 1.0
    mic_agc_attack: float = 0.01
    mic_agc_release: float = 0.0001
    mic_gate_thresh: float = 0.005
    resampling: str = "adaptive"


# ============================================================
# SERIAL BACKEND (CDC)
# ============================================================

class SerialWorker:
    """Thread-safe serial communication handler"""

    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser: Optional[serial.Serial] = None
        self.thread: Optional[threading.Thread] = None
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int = 115200):
        if not HAS_SERIAL:
            raise RuntimeError("pyserial not installed (pip install pyserial)")
        with self.lock:
            if self.is_connected():
                return
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, write_timeout=0.5)
            self.stop_evt.clear()
            self.thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.thread.start()

    def disconnect(self):
        with self.lock:
            self.stop_evt.set()
            s = self.ser
            self.ser = None
        if s:
            try:
                s.close()
            except Exception:
                pass

    def send_line(self, line: str):
        line = line.strip()
        if not line:
            return
        data = (line + "\r\n").encode("utf-8", errors="replace")
        with self.lock:
            if not self.is_connected():
                raise RuntimeError("Not connected")
            self.ser.write(data)
            self.ser.flush()

    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_evt.is_set():
            with self.lock:
                s = self.ser
            if not s:
                break
            try:
                chunk = s.read(256)
                if chunk:
                    buf.extend(chunk)
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        txt = line.decode("utf-8", errors="replace").rstrip("\r")
                        self.rx_queue.put(txt)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.rx_queue.put(f"[SERIAL ERROR] {e}")
                break


# ============================================================
# UI HELPERS
# ============================================================

def list_serial_ports():
    if not HAS_SERIAL:
        return []
    ports = []
    for p in serial.tools.list_ports.comports():
        if "SX1280" in p.description or "cafe:4073" in str(p.hwid).lower():
            ports.insert(0, (p.device, f"\u2605 {p.device} ({p.description})"))
        else:
            ports.append((p.device, f"{p.device} ({p.description})"))
    return ports


class Debouncer:
    def __init__(self, tk_root: tk.Tk, delay_ms: int, fn: Callable):
        self.root = tk_root
        self.delay_ms = delay_ms
        self.fn = fn
        self._after_id = None
        self._last_args = None
        self._last_kwargs = None

    def call(self, *args, **kwargs):
        self._last_args = args
        self._last_kwargs = kwargs
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self._after_id = self.root.after(self.delay_ms, self._fire)

    def _fire(self):
        self._after_id = None
        if self._last_args is not None:
            self.fn(*self._last_args, **(self._last_kwargs or {}))


class LabeledScale(ttk.Frame):
    """Reusable labeled scale widget with value display.

    The value label auto-updates on any variable change (including
    programmatic .set() calls from status push) via trace_add.
    """

    def __init__(self, parent, label, var, from_, to, resolution,
                 on_change, format_str="{:.1f}"):
        super().__init__(parent)
        self.var = var
        self.resolution = resolution
        self.on_change = on_change
        self.format_str = format_str

        self.columnconfigure(1, weight=1)

        ttk.Label(self, text=label, width=16, anchor="w").grid(row=0, column=0, sticky="w")

        self.scale = ttk.Scale(self, from_=from_, to=to, orient=tk.HORIZONTAL,
                               variable=var, command=self._on_scale)
        self.scale.grid(row=0, column=1, sticky="ew", padx=(8, 8))

        self.value_label = ttk.Label(self, width=10, anchor="e")
        self.value_label.grid(row=0, column=2, sticky="e")
        self._update_value_label()

        self.scale.bind("<ButtonRelease-1>", self._on_release)

        # Auto-update label when variable changes (including programmatic .set())
        self._trace_id = var.trace_add("write", self._on_var_write)

    def _on_scale(self, _val):
        self._update_value_label()

    def _on_var_write(self, *_args):
        """Called on ANY variable change -- fixes TX Power label not updating."""
        self._update_value_label()

    def _on_release(self, _event):
        v = float(self.var.get())
        v = round(v / self.resolution) * self.resolution
        self.var.set(v)
        self._update_value_label()
        self.on_change(v)

    def _update_value_label(self):
        v = float(self.var.get())
        if callable(self.format_str):
            text = self.format_str(v)
        else:
            text = self.format_str.format(v)
        self.value_label.config(text=text)


# ============================================================
# SCROLLABLE FRAME
# ============================================================

class ScrollableFrame(ttk.Frame):
    """A frame with a vertical scrollbar for content that may exceed window height."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)

        self.canvas = tk.Canvas(self, highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)

        self.inner.bind("<Configure>", self._on_inner_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        self.canvas_window = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        self.canvas.bind("<Enter>", self._bind_mousewheel)
        self.canvas.bind("<Leave>", self._unbind_mousewheel)

    def _on_inner_configure(self, _event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.canvas.itemconfig(self.canvas_window, width=event.width)

    def _bind_mousewheel(self, _event):
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _unbind_mousewheel(self, _event):
        self.canvas.unbind_all("<Button-4>")
        self.canvas.unbind_all("<Button-5>")
        self.canvas.unbind_all("<MouseWheel>")

    def _on_mousewheel(self, event):
        if event.num == 4:
            self.canvas.yview_scroll(-3, "units")
        elif event.num == 5:
            self.canvas.yview_scroll(3, "units")
        elif hasattr(event, 'delta'):
            self.canvas.yview_scroll(-1 * (event.delta // 120), "units")


# ============================================================
# MAIN APPLICATION
# ============================================================

class SX1280ControlApp(ttk.Frame):
    FREQ_MIN_HZ = 2_300_000_000
    FREQ_MAX_HZ = 2_450_000_000
    FREQ_STEP_HZ = 100

    # QO-100 narrowband transponder limits
    QO100_MIN_HZ = 2_400_000_000
    QO100_MAX_HZ = 2_400_500_000

    def __init__(self, master):
        super().__init__(master)
        self.master = master

        self.config = TxConfig()
        self.rx_queue = queue.Queue()
        self.worker = SerialWorker(self.rx_queue)
        self._status_updating = False
        self._heartbeat_id = None

        self.debounced_send = Debouncer(master, 150, self._send_cmd_safe)
        self.freq_debouncer = Debouncer(master, 200, self._send_freq)

        self._create_variables()
        self._build_ui()
        self._update_freq_display()

        self._poll_rx()
        self.pack(fill="both", expand=True)

    # ----------------------------------------------------------
    def _create_variables(self):
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="\u26ab Disconnected")
        self.freq_mhz_var = tk.DoubleVar(value=self.config.freq_hz / 1_000_000)
        self.freq_khz_var = tk.StringVar(value=f"{self.config.freq_hz / 1000:.1f}")
        self.ppm_var = tk.DoubleVar(value=0.0)
        self.txpwr_var = tk.IntVar(value=self.config.tx_power_dbm)
        self.tx_enabled_var = tk.BooleanVar(value=False)
        self.mode_var = tk.StringVar(value="usb")
        self.full_band_var = tk.BooleanVar(value=False)
        self.src_var = tk.StringVar(value="pc")
        self.tune_var = tk.BooleanVar(value=False)
        self.en_bp_var = tk.BooleanVar(value=self.config.enable_bp)
        self.en_eq_var = tk.BooleanVar(value=self.config.enable_eq)
        self.en_comp_var = tk.BooleanVar(value=self.config.enable_comp)
        self.bp_lo_var = tk.DoubleVar(value=self.config.bp_lo_hz)
        self.bp_hi_var = tk.DoubleVar(value=self.config.bp_hi_hz)
        self.bp_stages_var = tk.IntVar(value=self.config.bp_stages)
        self.eq_low_hz_var = tk.DoubleVar(value=self.config.eq_low_hz)
        self.eq_low_db_var = tk.DoubleVar(value=self.config.eq_low_db)
        self.eq_high_hz_var = tk.DoubleVar(value=self.config.eq_high_hz)
        self.eq_high_db_var = tk.DoubleVar(value=self.config.eq_high_db)
        self.comp_thr_var = tk.DoubleVar(value=self.config.comp_thr_db)
        self.comp_ratio_var = tk.DoubleVar(value=self.config.comp_ratio)
        self.comp_att_var = tk.DoubleVar(value=self.config.comp_attack_ms)
        self.comp_rel_var = tk.DoubleVar(value=self.config.comp_release_ms)
        self.comp_makeup_var = tk.DoubleVar(value=self.config.comp_makeup_db)
        self.comp_knee_var = tk.DoubleVar(value=self.config.comp_knee_db)
        self.comp_outlim_var = tk.DoubleVar(value=self.config.comp_out_limit)
        self.amp_gain_var = tk.DoubleVar(value=self.config.amp_gain)
        self.amp_min_a_var = tk.StringVar(value=f"{self.config.amp_min_a:.9f}")
        self.mic_agc_target_var = tk.DoubleVar(value=self.config.mic_agc_target)
        self.mic_agc_max_gain_var = tk.DoubleVar(value=self.config.mic_agc_max_gain)
        self.mic_agc_attack_var = tk.DoubleVar(value=self.config.mic_agc_attack)
        self.mic_agc_release_var = tk.DoubleVar(value=self.config.mic_agc_release)
        self.mic_gate_thresh_var = tk.DoubleVar(value=self.config.mic_gate_thresh)
        self.fm_dev_var = tk.DoubleVar(value=2500.0)
        self.ctcss_var = tk.StringVar(value="Off")
        self.resampling_var = tk.StringVar(value=self.config.resampling)

    # ----------------------------------------------------------
    def _build_ui(self):
        self.master.title("SX1280 QO-100 SSB TX Control")
        self.master.geometry("900x800")
        self.master.minsize(600, 400)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        self._build_connection_bar()

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self._build_dsp_tab()
        self._build_console_tab()

    # ----------------------------------------------------------
    def _build_connection_bar(self):
        conn_frame = ttk.Frame(self)
        conn_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=(0, 5))

        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_var.set(labels[0] if labels else "")

        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var,
                                        values=labels, state="readonly", width=45)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=5)

        btn_frame = ttk.Frame(conn_frame)
        btn_frame.grid(row=0, column=2)
        ttk.Button(btn_frame, text="\U0001f504", width=3, command=self._refresh_ports).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="Connect", command=self._connect).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="Disconnect", command=self._disconnect).pack(side="left", padx=2)

        ttk.Label(conn_frame, textvariable=self.status_var).grid(row=0, column=3, padx=(10, 0))

    # ----------------------------------------------------------
    def _build_dsp_tab(self):
        scroll_container = ScrollableFrame(self.notebook)
        self.notebook.add(scroll_container, text="RF & DSP")
        self._dsp_scroll = scroll_container
        tab = scroll_container.inner
        tab.columnconfigure(0, weight=1)

        # === Mode / Tune ===
        mt_frame = ttk.LabelFrame(tab, text="Mode / Tune", padding=10)
        mt_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        mt_inner = ttk.Frame(mt_frame)
        mt_inner.pack(fill="x")

        ttk.Label(mt_inner, text="Mode:").pack(side="left", padx=(0, 5))
        ttk.Radiobutton(mt_inner, text="USB (SSB)", variable=self.mode_var,
                         value="usb", command=self._on_mode_change).pack(side="left", padx=5)
        ttk.Radiobutton(mt_inner, text="CW", variable=self.mode_var,
                         value="cw", command=self._on_mode_change).pack(side="left", padx=5)
        ttk.Radiobutton(mt_inner, text="FM", variable=self.mode_var,
                         value="fm", command=self._on_mode_change).pack(side="left", padx=5)

        ttk.Separator(mt_inner, orient="vertical").pack(side="left", fill="y", padx=10, pady=2)

        ttk.Label(mt_inner, text="Src:").pack(side="left", padx=(0, 5))
        ttk.Radiobutton(mt_inner, text="PC", variable=self.src_var,
                         value="pc", command=self._on_src_change).pack(side="left", padx=5)
        ttk.Radiobutton(mt_inner, text="MIC", variable=self.src_var,
                         value="mic", command=self._on_src_change).pack(side="left", padx=5)

        ttk.Separator(mt_inner, orient="vertical").pack(side="left", fill="y", padx=10, pady=2)

        ttk.Label(mt_inner, text="Resampling:").pack(side="left", padx=(0, 5))
        ttk.Radiobutton(mt_inner, text="Adaptive", variable=self.resampling_var,
                         value="adaptive", command=self._on_resampling_change).pack(side="left", padx=5)
        ttk.Radiobutton(mt_inner, text="Fixed", variable=self.resampling_var,
                         value="fixed", command=self._on_resampling_change).pack(side="left", padx=5)

        ttk.Separator(mt_inner, orient="vertical").pack(side="left", fill="y", padx=10, pady=2)

        self.tune_button = tk.Button(mt_inner, text="TUNE OFF", width=7,
                                      font=("TkDefaultFont", 10, "bold"),
                                      command=self._toggle_tune, relief="raised", bd=3,
                                      bg="#cccccc", fg="black")
        self.tune_button.pack(side="left", padx=5)

        ttk.Separator(mt_inner, orient="vertical").pack(side="left", fill="y", padx=10, pady=2)

        self.tx_button = tk.Button(mt_inner, text="TX OFF", width=7,
                                    font=("TkDefaultFont", 10, "bold"),
                                    command=self._toggle_tx, relief="raised", bd=3)
        self.tx_button.pack(side="left", padx=5)
        self._update_tx_button()

        # === RF / Frequency ===
        rf_frame = ttk.LabelFrame(tab, text="RF / Frequency", padding=10)
        rf_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        rf_frame.columnconfigure(1, weight=1)

        ttk.Checkbutton(rf_frame, text="Full Band (2300–2450 MHz)",
                         variable=self.full_band_var,
                         command=self._on_full_band_toggle).grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Label(rf_frame, text="Frequency:").grid(row=1, column=0, sticky="w")
        freq_slider_frame = ttk.Frame(rf_frame)
        freq_slider_frame.grid(row=1, column=1, sticky="ew", padx=5)
        freq_slider_frame.columnconfigure(0, weight=1)

        min_freq, max_freq = self._get_freq_scale_min_max(self.full_band_var.get())
        self.freq_scale = ttk.Scale(freq_slider_frame,
                                     from_=min_freq / 1_000_000,
                                     to=max_freq / 1_000_000,
                                     orient=tk.HORIZONTAL,
                                     variable=self.freq_mhz_var,
                                     command=self._on_freq_slider)
        self.freq_scale.grid(row=0, column=0, sticky="ew")

        freq_entry_frame = ttk.Frame(rf_frame)
        freq_entry_frame.grid(row=1, column=2)
        self.freq_entry = ttk.Entry(freq_entry_frame, textvariable=self.freq_khz_var, width=14)
        self.freq_entry.pack(side="left")
        self.freq_entry.bind("<Return>", lambda e: self._send_freq_from_entry())
        ttk.Label(freq_entry_frame, text=" kHz").pack(side="left")

        freq_display_frame = ttk.Frame(rf_frame)
        freq_display_frame.grid(row=2, column=1, columnspan=2, sticky="w", padx=5)

        self.freq_mhz_label = ttk.Label(freq_display_frame, text="2400.1000 MHz \u2191",
                                         font=("TkDefaultFont", 12, "bold"))
        self.freq_mhz_label.pack(side="left")

        ttk.Label(freq_display_frame, text="   \u2192   ").pack(side="left")
        self.downlink_label = ttk.Label(freq_display_frame, text="10489.6000 MHz \u2193",
                                         font=("TkDefaultFont", 12, "bold"), foreground="blue")
        self.downlink_label.pack(side="left")

        # PPM
        ttk.Label(rf_frame, text="PPM:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ppm_frame = ttk.Frame(rf_frame)
        ppm_frame.grid(row=3, column=1, columnspan=2, sticky="ew", padx=5, pady=(10, 0))
        ppm_frame.columnconfigure(0, weight=1)

        self.ppm_scale = ttk.Scale(ppm_frame, from_=-2.0, to=2.0, orient=tk.HORIZONTAL,
                                    variable=self.ppm_var, command=self._on_ppm_slider)
        self.ppm_scale.grid(row=0, column=0, sticky="ew")
        self.ppm_label = ttk.Label(ppm_frame, text="0.000 ppm", width=12)
        self.ppm_label.grid(row=0, column=1, padx=5)

        # TX Power
        ttk.Label(rf_frame, text="TX Power:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        txpwr_frame = ttk.Frame(rf_frame)
        txpwr_frame.grid(row=4, column=1, sticky="ew", padx=5, pady=(10, 0))
        txpwr_frame.columnconfigure(0, weight=1)

        self.txpwr_scale = LabeledScale(txpwr_frame, "", self.txpwr_var, -18, 13, 1,
                     lambda v: self._send_cmd_safe(f"txpwr {int(v)}"),
                     lambda v: f"{int(v)} dBm")
        self.txpwr_scale.pack(fill="x")

        # === DSP Modules ===
        enable_frame = ttk.LabelFrame(tab, text="DSP Modules", padding=10)
        enable_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        ttk.Checkbutton(enable_frame, text="Bandpass Filter", variable=self.en_bp_var,
                        command=lambda: self._send_enable("bp", self.en_bp_var.get())).pack(side="left", padx=20)
        ttk.Checkbutton(enable_frame, text="Equalizer", variable=self.en_eq_var,
                        command=lambda: self._send_enable("eq", self.en_eq_var.get())).pack(side="left", padx=20)
        ttk.Checkbutton(enable_frame, text="Compressor", variable=self.en_comp_var,
                        command=lambda: self._send_enable("comp", self.en_comp_var.get())).pack(side="left", padx=20)

        # === Bandpass ===
        bp_frame = ttk.LabelFrame(tab, text="Bandpass Filter", padding=10)
        bp_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        bp_frame.columnconfigure(0, weight=1)
        LabeledScale(bp_frame, "Low cutoff (Hz)", self.bp_lo_var, 50, 1500, 10,
                     lambda v: self.debounced_send.call(f"set bp_lo {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(bp_frame, "High cutoff (Hz)", self.bp_hi_var, 500, 3600, 10,
                     lambda v: self.debounced_send.call(f"set bp_hi {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(bp_frame, "Steepness (stages)", self.bp_stages_var, 1, 10, 1,
                     lambda v: self.debounced_send.call(f"set bp_stages {int(v)}"),
                     lambda v: f"{int(v)} ({int(v)*12} dB/oct)").pack(fill="x")

        # === EQ ===
        eq_frame = ttk.LabelFrame(tab, text="Equalizer (Shelving)", padding=10)
        eq_frame.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        eq_frame.columnconfigure(0, weight=1)
        LabeledScale(eq_frame, "Low shelf freq (Hz)", self.eq_low_hz_var, 50, 1000, 10,
                     lambda v: self.debounced_send.call(f"set eq_low_hz {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(eq_frame, "Low shelf gain (dB)", self.eq_low_db_var, -24, 24, 0.5,
                     lambda v: self.debounced_send.call(f"set eq_low_db {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(eq_frame, "High shelf freq (Hz)", self.eq_high_hz_var, 500, 3500, 10,
                     lambda v: self.debounced_send.call(f"set eq_high_hz {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(eq_frame, "High shelf gain (dB)", self.eq_high_db_var, -24, 24, 0.5,
                     lambda v: self.debounced_send.call(f"set eq_high_db {v:.1f}"),
                     "{:.1f}").pack(fill="x")

        # === Compressor ===
        comp_frame = ttk.LabelFrame(tab, text="Compressor", padding=10)
        comp_frame.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        comp_frame.columnconfigure(0, weight=1)
        LabeledScale(comp_frame, "Threshold (dB)", self.comp_thr_var, -60, 0, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_thr {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Ratio", self.comp_ratio_var, 1, 20, 0.1,
                     lambda v: self.debounced_send.call(f"set comp_ratio {v:.1f}"),
                     "{:.1f}:1").pack(fill="x")
        LabeledScale(comp_frame, "Attack (ms)", self.comp_att_var, 0.1, 200, 0.1,
                     lambda v: self.debounced_send.call(f"set comp_att {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Release (ms)", self.comp_rel_var, 10, 2000, 1,
                     lambda v: self.debounced_send.call(f"set comp_rel {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(comp_frame, "Makeup gain (dB)", self.comp_makeup_var, 0, 40, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_makeup {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Knee (dB)", self.comp_knee_var, 0, 24, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_knee {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Output limit", self.comp_outlim_var, 0.01, 0.999, 0.001,
                     lambda v: self.debounced_send.call(f"set comp_outlim {v:.3f}"),
                     "{:.3f}").pack(fill="x")

        # === Power Shaping ===
        pwr_frame = ttk.LabelFrame(tab, text="Power Shaping", padding=10)
        pwr_frame.grid(row=6, column=0, sticky="ew", pady=(0, 10))
        pwr_frame.columnconfigure(0, weight=1)
        LabeledScale(pwr_frame, "Amp gain", self.amp_gain_var, 0.01, 5.0, 0.01,
                     lambda v: self.debounced_send.call(f"set amp_gain {v:.3f}"),
                     "{:.3f}").pack(fill="x")
        amp_min_frame = ttk.Frame(pwr_frame)
        amp_min_frame.pack(fill="x", pady=(5, 0))
        ttk.Label(amp_min_frame, text="Amp min A:", width=16).pack(side="left")
        ttk.Entry(amp_min_frame, textvariable=self.amp_min_a_var, width=16).pack(side="left", padx=5)
        ttk.Button(amp_min_frame, text="Set",
                   command=lambda: self._send_cmd_safe(f"set amp_min_a {self.amp_min_a_var.get()}")
                  ).pack(side="left")

        # === MIC AGC (ADC microphone input processing) ===
        mic_frame = ttk.LabelFrame(tab, text="MIC AGC (ADC input)", padding=10)
        mic_frame.grid(row=7, column=0, sticky="ew", pady=(0, 10))
        mic_frame.columnconfigure(0, weight=1)
        LabeledScale(mic_frame, "AGC target", self.mic_agc_target_var, 0.01, 1.0, 0.01,
                     lambda v: self.debounced_send.call(f"set mic_agc_target {v:.3f}"),
                     "{:.3f}").pack(fill="x")
        LabeledScale(mic_frame, "AGC max gain", self.mic_agc_max_gain_var, 1.0, 200.0, 1.0,
                     lambda v: self.debounced_send.call(f"set mic_agc_max_gain {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(mic_frame, "AGC attack", self.mic_agc_attack_var, 0.0001, 0.5, 0.0001,
                     lambda v: self.debounced_send.call(f"set mic_agc_attack {v:.4f}"),
                     "{:.4f}").pack(fill="x")
        LabeledScale(mic_frame, "AGC release", self.mic_agc_release_var, 0.00001, 0.1, 0.00001,
                     lambda v: self.debounced_send.call(f"set mic_agc_release {v:.5f}"),
                     "{:.5f}").pack(fill="x")
        LabeledScale(mic_frame, "Noise gate", self.mic_gate_thresh_var, 0.0, 0.5, 0.001,
                     lambda v: self.debounced_send.call(f"set mic_gate {v:.4f}"),
                     "{:.4f}").pack(fill="x")

        # === FM Settings ===
        fm_frame = ttk.LabelFrame(tab, text="FM Settings", padding=10)
        fm_frame.grid(row=8, column=0, sticky="ew", pady=(0, 10))
        fm_frame.columnconfigure(0, weight=1)

        CTCSS_TONES = [
            "Off", "67.0", "69.3", "71.9", "74.4", "77.0", "79.7", "82.5",
            "85.4", "88.5", "91.5", "94.8", "97.4", "100.0", "103.5", "107.2",
            "110.9", "114.8", "118.8", "123.0", "127.3", "131.8", "136.5",
            "141.3", "146.2", "151.4", "156.7", "162.2", "167.9", "173.8",
            "179.9", "186.2", "192.8", "203.5", "206.5", "210.7", "218.1",
            "225.7", "229.1", "233.6", "241.8", "250.3", "254.1",
        ]

        LabeledScale(fm_frame, "Deviation (Hz)", self.fm_dev_var, 200, 100000, 100,
                     lambda v: self.debounced_send.call(f"set fm_dev {v:.0f}"),
                     lambda v: f"{v:.0f} Hz").pack(fill="x")

        ctcss_row = ttk.Frame(fm_frame)
        ctcss_row.pack(fill="x", pady=(5, 0))
        ttk.Label(ctcss_row, text="CTCSS Tone:", width=16, anchor="w").pack(side="left")
        self.ctcss_combo = ttk.Combobox(ctcss_row, textvariable=self.ctcss_var,
                                         values=CTCSS_TONES, state="readonly", width=10)
        self.ctcss_combo.pack(side="left", padx=5)
        self.ctcss_combo.bind("<<ComboboxSelected>>", self._on_ctcss_change)
        ttk.Label(ctcss_row, text="Hz").pack(side="left")

        # === Spectrum Analyzer (placeholder) ===
        spec_frame = ttk.LabelFrame(tab, text="Spectrum Analyzer", padding=10)
        spec_frame.grid(row=9, column=0, sticky="ew", pady=(0, 10))
        spec_frame.columnconfigure(0, weight=1)

        # Simple placeholder canvas; real FFT plotting will be added later
        self.spec_canvas = tk.Canvas(spec_frame, height=120, bg="black")
        self.spec_canvas.pack(fill="x")
        self.spec_canvas.create_text(8, 60, anchor="w", fill="#cccccc",
                                     text="Spectrum analyzer display (placeholder)")
        ttk.Button(spec_frame, text="Refresh", command=lambda: self._send_cmd_safe("get")).pack(side="right", padx=5, pady=5)

    # ----------------------------------------------------------
    def _build_console_tab(self):
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Console")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)

        log_frame = ttk.Frame(tab)
        log_frame.grid(row=0, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, wrap="word", font=("Consolas", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=scrollbar.set)

        self.log_text.tag_configure("sent", foreground="#0066cc")
        self.log_text.tag_configure("recv", foreground="#006600")
        self.log_text.tag_configure("error", foreground="#cc0000")
        self.log_text.tag_configure("info", foreground="#666666")

        # Manual command (below the log)
        cmd_frame = ttk.LabelFrame(tab, text="Command", padding=5)
        cmd_frame.grid(row=1, column=0, sticky="ew", pady=(5, 5))
        cmd_frame.columnconfigure(0, weight=1)

        self.manual_cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.manual_cmd_var)
        cmd_entry.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        cmd_entry.bind("<Return>", lambda e: self._send_manual_cmd())
        ttk.Button(cmd_frame, text="Send", command=self._send_manual_cmd).grid(row=0, column=1)

        btn_frame = ttk.Frame(tab)
        btn_frame.grid(row=2, column=0, sticky="ew", pady=(5, 0))
        ttk.Button(btn_frame, text="Clear Log", command=self._clear_log).pack(side="left")
        ttk.Button(btn_frame, text="Send All Settings", command=self._send_all).pack(side="right")

    # === Connection Methods ===

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_combo["values"] = labels
        if labels:
            self.port_var.set(labels[0])
        self._log("Ports refreshed", "info")

    def _connect(self):
        if not HAS_SERIAL:
            messagebox.showerror("Missing dependency",
                                "pyserial is required.\nInstall with: pip install pyserial")
            return
        label = self.port_var.get()
        port = self.port_map.get(label)
        if not port or "no ports" in label.lower():
            messagebox.showerror("No port", "No serial port selected.")
            return
        try:
            self.worker.connect(port)
            self.status_var.set(f"\U0001f7e2 Connected: {port}")
            self._log(f"Connected to {port}", "info")
            self.master.after(500, lambda: self._send_cmd_safe("get"))
            self.master.after(800, lambda: self._send_cmd_safe("status"))
            self._start_heartbeat()
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            self.status_var.set("\U0001f534 Connection failed")

    def _disconnect(self):
        if self._heartbeat_id is not None:
            self.master.after_cancel(self._heartbeat_id)
            self._heartbeat_id = None
        self.worker.disconnect()
        self.status_var.set("\u26ab Disconnected")
        self._log("Disconnected", "info")

    def _start_heartbeat(self):
        """Periodically request status from firmware to keep GUI in sync."""
        if not self.worker.is_connected():
            self._heartbeat_id = None
            return
        try:
            self.worker.send_line("status")
        except Exception:
            pass
        self._heartbeat_id = self.master.after(2000, self._start_heartbeat)

    # === Command Methods ===

    def _send_cmd_safe(self, cmd):
        try:
            if not self.worker.is_connected():
                self._log(f"[NOT CONNECTED] {cmd}", "error")
                return
            self.worker.send_line(cmd)
            self._log(f"> {cmd}", "sent")
        except Exception as e:
            self._log(f"[SEND ERROR] {e}", "error")

    def _send_enable(self, which, enabled):
        v = "1" if enabled else "0"
        self._send_cmd_safe(f"enable {which} {v}")

    def _toggle_tx(self):
        current = self.tx_enabled_var.get()
        new_state = not current
        self.tx_enabled_var.set(new_state)
        self._update_tx_button()
        self._send_cmd_safe(f"tx {'1' if new_state else '0'}")

    def _on_mode_change(self):
        if self._status_updating:
            return
        mode = self.mode_var.get()
        self._send_cmd_safe(f"mode {mode}")

    def _on_ctcss_change(self, _event=None):
        if self._status_updating:
            return
        val = self.ctcss_var.get()
        freq = "0" if val == "Off" else val
        self._send_cmd_safe(f"set ctcss {freq}")

    def _get_freq_scale_min_max(self, full):
        if full:
            return self.FREQ_MIN_HZ, self.FREQ_MAX_HZ
        else:
            return self.QO100_MIN_HZ, self.QO100_MAX_HZ

    def _on_full_band_toggle(self):
        full = self.full_band_var.get()
        lo, hi = self._get_freq_scale_min_max(full)

        self.freq_scale.configure(from_=lo / 1_000_000, to=hi / 1_000_000)
        # Clamp current frequency into new range
        try:
            hz = float(self.freq_khz_var.get()) * 1000
        except ValueError:
            hz = self.config.freq_hz
        hz = max(lo, min(hi, hz))
        self.freq_khz_var.set(f"{hz / 1000:.1f}")
        self.freq_mhz_var.set(hz / 1_000_000)
        self._update_freq_display()

    def _on_src_change(self):
        if self._status_updating:
            return
        src = self.src_var.get()
        self._send_cmd_safe(f"src {src}")

    def _on_resampling_change(self):
        if self._status_updating:
            return
        resampling = self.resampling_var.get()
        self._send_cmd_safe(f"resampling {resampling}")

    def _toggle_tune(self):
        new_state = not self.tune_var.get()
        self.tune_var.set(new_state)
        self._update_tune_button()
        self._send_cmd_safe(f"tune {'1' if new_state else '0'}")

    def _update_tune_button(self):
        if self.tune_var.get():
            self.tune_button.config(text="TUNE ON", bg="#ff8800", fg="white",
                                     activebackground="#ffaa00", activeforeground="white")
        else:
            self.tune_button.config(text="TUNE OFF", bg="#cccccc", fg="black",
                                     activebackground="#dddddd", activeforeground="black")

    def _update_tx_button(self):
        if self.tx_enabled_var.get():
            self.tx_button.config(text="TX ON", bg="#00cc00", fg="white",
                                   activebackground="#00ff00", activeforeground="white")
        else:
            self.tx_button.config(text="TX OFF", bg="#cccccc", fg="black",
                                   activebackground="#dddddd", activeforeground="black")

    def _on_ppm_slider(self, _val):
        if self._status_updating:
            return
        ppm = self.ppm_var.get()
        self.ppm_label.config(text=f"{ppm:.3f} ppm")
        self._update_freq_display()
        self._send_cmd_safe(f"ppm {ppm:.4f}")

    def _update_freq_display(self):
        try:
            khz = float(self.freq_khz_var.get())
            hz = khz * 1000
        except ValueError:
            hz = self.config.freq_hz
        self.freq_mhz_label.config(text=f"{hz/1_000_000:.4f} MHz \u2191")
        if self.full_band_var.get():
            self.downlink_label.config(text="")
        else:
            downlink_hz = hz + 8089_500_000
            self.downlink_label.config(text=f"{downlink_hz/1_000_000:.4f} MHz \u2193")

    def _on_freq_slider(self, _val):
        if self._status_updating:
            return
        mhz = self.freq_mhz_var.get()
        hz = round(mhz * 1_000_000 / 100) * 100  # snap to 0.1 kHz
        hz = self._clamp_freq(hz)
        self.freq_khz_var.set(f"{hz / 1000:.1f}")
        self._update_freq_display()
        self._send_cmd_safe(f"freq {hz}")

    def _clamp_freq(self, hz):
        if self.full_band_var.get():
            lo, hi = self.FREQ_MIN_HZ, self.FREQ_MAX_HZ
        else:
            lo, hi = self.QO100_MIN_HZ, self.QO100_MAX_HZ
        return max(lo, min(hi, hz))

    def _send_freq(self, hz):
        self._send_cmd_safe(f"freq {hz:.1f}")

    def _send_freq_from_entry(self):
        try:
            khz = float(self.freq_khz_var.get().replace(",", "."))
            hz = round(khz * 1000 / 100) * 100  # snap to 0.1 kHz
            hz = self._clamp_freq(hz)
            self.freq_khz_var.set(f"{hz / 1000:.1f}")
            self.freq_mhz_var.set(hz / 1_000_000)
            self._update_freq_display()
            self._send_cmd_safe(f"freq {hz:.1f}")
        except ValueError:
            messagebox.showerror("Invalid frequency", "Frequency must be a number in kHz")

    def _send_ppm(self):
        ppm = self.ppm_var.get()
        self._send_cmd_safe(f"ppm {ppm:.4f}")

    def _send_manual_cmd(self):
        cmd = self.manual_cmd_var.get().strip()
        if cmd:
            self._send_cmd_safe(cmd)
            self.manual_cmd_var.set("")

    def _send_all(self):
        try:
            khz = float(self.freq_khz_var.get())
            hz = self._clamp_freq(int(round(khz * 1000)))
        except ValueError:
            hz = int(self.config.freq_hz)
        self._send_cmd_safe(f"freq {hz}")
        try:
            ppm = float(self.ppm_var.get())
            if -100 <= ppm <= 100:
                self._send_cmd_safe(f"ppm {ppm}")
        except Exception:
            pass
        self._send_cmd_safe(f"txpwr {int(self.txpwr_var.get())}")
        self._send_cmd_safe(f"enable bp {'1' if self.en_bp_var.get() else '0'}")
        self._send_cmd_safe(f"enable eq {'1' if self.en_eq_var.get() else '0'}")
        self._send_cmd_safe(f"enable comp {'1' if self.en_comp_var.get() else '0'}")
        self._send_cmd_safe(f"set bp_lo {self.bp_lo_var.get():.0f}")
        self._send_cmd_safe(f"set bp_hi {self.bp_hi_var.get():.0f}")
        self._send_cmd_safe(f"set bp_stages {int(self.bp_stages_var.get())}")
        self._send_cmd_safe(f"set eq_low_hz {self.eq_low_hz_var.get():.0f}")
        self._send_cmd_safe(f"set eq_low_db {self.eq_low_db_var.get():.1f}")
        self._send_cmd_safe(f"set eq_high_hz {self.eq_high_hz_var.get():.0f}")
        self._send_cmd_safe(f"set eq_high_db {self.eq_high_db_var.get():.1f}")
        self._send_cmd_safe(f"set comp_thr {self.comp_thr_var.get():.1f}")
        self._send_cmd_safe(f"set comp_ratio {self.comp_ratio_var.get():.1f}")
        self._send_cmd_safe(f"set comp_att {self.comp_att_var.get():.1f}")
        self._send_cmd_safe(f"set comp_rel {self.comp_rel_var.get():.0f}")
        self._send_cmd_safe(f"set comp_makeup {self.comp_makeup_var.get():.1f}")
        self._send_cmd_safe(f"set comp_knee {self.comp_knee_var.get():.1f}")
        self._send_cmd_safe(f"set comp_outlim {self.comp_outlim_var.get():.3f}")
        self._send_cmd_safe(f"set amp_gain {self.amp_gain_var.get():.3f}")
        self._send_cmd_safe(f"set amp_min_a {self.amp_min_a_var.get()}")
        self._send_cmd_safe(f"resampling {self.resampling_var.get()}")
        self._log("All settings sent", "info")

    # === Logging ===

    def _log(self, msg, tag="recv"):
        self.log_text.insert("end", msg + "\n", tag)
        self.log_text.see("end")

    def _clear_log(self):
        self.log_text.delete("1.0", "end")

    def _poll_rx(self):
        processed = 0
        max_per_cycle = 20
        latest_status = None
        try:
            while processed < max_per_cycle:
                line = self.rx_queue.get_nowait()
                processed += 1
                if line.startswith("!S "):
                    latest_status = line  # keep only the newest status
                else:
                    self._log(line, "recv")
        except queue.Empty:
            pass
        if latest_status is not None:
            self._handle_status_push(latest_status)
        # Poll faster when queue had items, slower when idle
        delay = 10 if processed >= max_per_cycle else 50
        self.master.after(delay, self._poll_rx)

    def _handle_status_push(self, line):
        """Parse firmware status push and update GUI widgets.

        Format: '!S mode=1 tune=0 tx=1 pwr=13 ppm=0.12 freq=2400100000.0 resampling=adaptive'

        Fixes:
        - TX Power label: LabeledScale trace_add auto-updates on .set()
        - Frequency: formatted as integer string, slider guarded with _status_updating
        """
        try:
            parts = line.strip().split()
            kv = {}
            for p in parts[1:]:
                if "=" in p:
                    k, v = p.split("=", 1)
                    kv[k] = v

            self._status_updating = True

            if "mode" in kv:
                mode_map = {"0": "usb", "1": "cw", "2": "fm"}
                new_mode = mode_map.get(kv["mode"], "usb")
                if self.mode_var.get() != new_mode:
                    self.mode_var.set(new_mode)

            if "src" in kv:
                new_src = "mic" if kv["src"] == "1" else "pc"
                if self.src_var.get() != new_src:
                    self.src_var.set(new_src)

            if "tune" in kv:
                new_tune = kv["tune"] == "1"
                if self.tune_var.get() != new_tune:
                    self.tune_var.set(new_tune)
                    self._update_tune_button()

            if "tx" in kv:
                new_tx = kv["tx"] == "1"
                if self.tx_enabled_var.get() != new_tx:
                    self.tx_enabled_var.set(new_tx)
                    self._update_tx_button()

            if "pwr" in kv:
                new_pwr = int(kv["pwr"])
                if self.txpwr_var.get() != new_pwr:
                    self.txpwr_var.set(new_pwr)
                    # LabeledScale trace auto-updates the value label

            if "ppm" in kv:
                new_ppm = float(kv["ppm"])
                if abs(self.ppm_var.get() - new_ppm) > 0.0001:
                    self.ppm_var.set(new_ppm)
                    self.ppm_label.config(text=f"{new_ppm:.3f} ppm")

            if "resampling" in kv:
                new_resampling = str(kv["resampling"])

                if self.resampling_var.get() != new_resampling:
                    self.resampling_var.set(new_resampling)

            if "bp" in kv:
                new_bp = int(kv["bp"])
                if self.en_bp_var.get() != new_bp:
                    self.en_bp_var.set(new_bp)
            if "eq" in kv:
                new_eq = int(kv["eq"])
                if self.en_eq_var.get() != new_eq:
                    self.en_eq_var.set(new_eq)
            if "comp" in kv:
                new_comp = int(kv["comp"])
                if self.en_comp_var.get() != new_comp:
                    self.en_comp_var.set(new_comp)                                        

            if "freq" in kv:
                new_freq = float(kv["freq"])
                if abs(self.config.freq_hz - new_freq) > 0.5:
                    self.config.freq_hz = new_freq
                    self.freq_mhz_var.set(new_freq / 1_000_000)
                    self.freq_khz_var.set(f"{new_freq / 1000:.1f}")
                    self._update_freq_display()

            if "fm_dev" in kv:
                new_dev = float(kv["fm_dev"])
                if abs(self.fm_dev_var.get() - new_dev) > 1.0:
                    self.fm_dev_var.set(new_dev)

            if "ctcss" in kv:
                new_ctcss = float(kv["ctcss"])
                if new_ctcss < 0.5:
                    if self.ctcss_var.get() != "Off":
                        self.ctcss_var.set("Off")
                else:
                    tone_str = f"{new_ctcss:.1f}"
                    if self.ctcss_var.get() != tone_str:
                        self.ctcss_var.set(tone_str)

            self._status_updating = False
        except Exception as e:
            self._status_updating = False
            self._log(f"[STATUS PARSE ERROR] {e}: {line!r}", "error")


# ============================================================
# MAIN
# ============================================================

def main():
    root = tk.Tk()

    style = ttk.Style(root)
    available_themes = style.theme_names()
    for theme in ["clam", "alt", "default"]:
        if theme in available_themes:
            style.theme_use(theme)
            break

    style.configure("TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))

    app = SX1280ControlApp(root)

    def on_close():
        app.worker.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
