# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Plots that show what the settings actually do.

A slider labelled "Q factor" means nothing to most people, and neither does a
limiter threshold in isolation. Drawing the response the settings produce
turns three abstract numbers into one picture, and it updates as the sliders
move, so the relationship is visible rather than described.

The maths is the ordinary textbook kind - an RBJ peaking biquad for the
parametric band, a second-order Butterworth for the high-pass - evaluated at
the drawing resolution rather than filtered through anything. These are
pictures of the intent, not measurements of the DSP.
"""

import cmath
import math

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

SAMPLE_RATE = 48000.0
F_MIN, F_MAX = 20.0, 20000.0
DB_SPAN = 15.0          # vertical half-range of the EQ plot

ACCENT = (0.16, 0.68, 0.38)
GRIDC = (1, 1, 1, 0.10)
TEXTC = (1, 1, 1, 0.45)


def _peaking_db(f, f0, gain_db, q):
    """Magnitude of an RBJ peaking filter at frequency f, in dB."""
    if f0 <= 0 or gain_db == 0 or q <= 0:
        return 0.0
    A = 10.0 ** (gain_db / 40.0)
    w0 = 2.0 * math.pi * f0 / SAMPLE_RATE
    alpha = math.sin(w0) / (2.0 * q)

    b = (1 + alpha * A, -2 * math.cos(w0), 1 - alpha * A)
    a = (1 + alpha / A, -2 * math.cos(w0), 1 - alpha / A)

    z = cmath.exp(-1j * 2.0 * math.pi * f / SAMPLE_RATE)
    num = b[0] + b[1] * z + b[2] * z * z
    den = a[0] + a[1] * z + a[2] * z * z
    if den == 0:
        return 0.0
    return 20.0 * math.log10(abs(num / den))


def _highpass_db(f, fc):
    """Second-order Butterworth high-pass magnitude at f, in dB."""
    if fc <= 0:
        return 0.0
    r = f / fc
    if r <= 0:
        return -80.0
    mag = (r * r) / math.sqrt(1.0 + r ** 4)
    return 20.0 * math.log10(max(mag, 1e-6))


class Plot(Gtk.DrawingArea):
    """Shared chrome: background, border, grid helpers."""

    def __init__(self, height=150):
        super().__init__()
        self.set_size_request(-1, height)
        self.set_hexpand(True)
        self.connect("draw", self._draw)

    def _bg(self, cr, w, h):
        cr.set_source_rgba(0, 0, 0, 0.18)
        cr.rectangle(0, 0, w, h)
        cr.fill()

    def _hline(self, cr, w, y, label=None):
        cr.set_source_rgba(*GRIDC)
        cr.set_line_width(1)
        cr.move_to(0, y)
        cr.line_to(w, y)
        cr.stroke()
        if label:
            cr.set_source_rgba(*TEXTC)
            cr.set_font_size(9)
            cr.move_to(4, y - 3)
            cr.show_text(label)

    def _draw(self, _w, cr):
        raise NotImplementedError


class EqCurve(Plot):
    """Frequency response of the parametric band plus the high-pass."""

    def __init__(self):
        super().__init__(height=160)
        self.f0 = 0.0
        self.gain = 0.0
        self.q = 0.71
        self.hpf = 0.0

    def update(self, f0, gain_db, q_hundredths, hpf):
        self.f0 = float(f0)
        self.gain = float(gain_db)
        self.q = max(0.05, q_hundredths / 100.0)
        self.hpf = float(hpf)
        self.queue_draw()

    def _x(self, f, w):
        lo, hi = math.log10(F_MIN), math.log10(F_MAX)
        return w * (math.log10(max(F_MIN, min(F_MAX, f))) - lo) / (hi - lo)

    def _y(self, db, h):
        return h / 2.0 - (db / DB_SPAN) * (h / 2.0 - 6)

    def _draw(self, _w, cr):
        a = self.get_allocation()
        w, h = a.width, a.height
        self._bg(cr, w, h)

        for db in (-12, -6, 0, 6, 12):
            self._hline(cr, w, self._y(db, h),
                        f"{db:+d} dB" if db else "0 dB")

        cr.set_source_rgba(*GRIDC)
        cr.set_font_size(9)
        for f in (100, 1000, 10000):
            x = self._x(f, w)
            cr.move_to(x, 0)
            cr.line_to(x, h)
            cr.stroke()
            cr.set_source_rgba(*TEXTC)
            cr.move_to(x + 3, h - 4)
            cr.show_text("100 Hz" if f == 100 else
                         ("1 kHz" if f == 1000 else "10 kHz"))
            cr.set_source_rgba(*GRIDC)

        pts = []
        steps = max(64, int(w))
        for i in range(steps + 1):
            t = i / steps
            f = F_MIN * (F_MAX / F_MIN) ** t
            db = _peaking_db(f, self.f0, self.gain, self.q) + \
                _highpass_db(f, self.hpf)
            pts.append((self._x(f, w), self._y(max(-DB_SPAN, min(DB_SPAN, db)), h)))

        cr.move_to(*pts[0])
        for x, y in pts[1:]:
            cr.line_to(x, y)
        cr.set_source_rgb(*ACCENT)
        cr.set_line_width(2)
        cr.stroke()

        # Fill between the curve and the 0 dB line rather than down to the
        # bottom of the plot: what matters is the departure from flat, and a
        # fill to the floor turns a flat response into a solid block.
        zero = self._y(0.0, h)
        cr.move_to(pts[0][0], zero)
        for x, y in pts:
            cr.line_to(x, y)
        cr.line_to(pts[-1][0], zero)
        cr.close_path()
        cr.set_source_rgba(*ACCENT, 0.20)
        cr.fill()


class LimiterCurve(Plot):
    """Input against output level, showing where the limiter takes over."""

    def __init__(self):
        super().__init__(height=140)
        self.threshold = 0.0

    def update(self, threshold_db):
        self.threshold = float(threshold_db)
        self.queue_draw()

    def _draw(self, _w, cr):
        a = self.get_allocation()
        w, h = a.width, a.height
        self._bg(cr, w, h)

        # unity reference: what the output would be with no limiter
        cr.set_source_rgba(1, 1, 1, 0.18)
        cr.set_line_width(1)
        cr.set_dash([4, 4])
        cr.move_to(0, h)
        cr.line_to(w, 0)
        cr.stroke()
        cr.set_dash([])

        # A limiter is an infinite-ratio compressor: below the threshold it
        # passes the signal untouched, above it the output stops rising.
        thr = max(0.0, min(20.0, self.threshold))
        knee = 1.0 - (thr / 20.0)
        cr.move_to(0, h)
        cr.line_to(w * knee, h - h * knee)
        cr.line_to(w, h - h * knee)
        cr.set_source_rgb(*ACCENT)
        cr.set_line_width(2)
        cr.stroke()

        x = w * knee
        cr.set_source_rgba(1, 1, 1, 0.35)
        cr.move_to(x, 0)
        cr.line_to(x, h)
        cr.set_line_width(1)
        cr.stroke()
        cr.set_source_rgba(*TEXTC)
        cr.set_font_size(9)
        cr.move_to(min(w - 60, x + 4), 12)
        cr.show_text(f"-{thr:.0f} dB")


class RampCurve(Plot):
    """Volume against time for the selected ramp shape."""

    SHAPES = ("linear", "exponential", "logarithmic")

    def __init__(self):
        super().__init__(height=120)
        self.shape = 0
        self.ms = 0

    def update(self, shape_index, ms):
        self.shape = int(shape_index)
        self.ms = int(ms)
        self.queue_draw()

    def _value(self, t):
        if self.shape == 1:
            return t * t
        if self.shape == 2:
            return math.sqrt(t)
        return t

    def _draw(self, _w, cr):
        a = self.get_allocation()
        w, h = a.width, a.height
        self._bg(cr, w, h)
        self._hline(cr, w, h * 0.5)

        if self.ms <= 0:
            cr.set_source_rgba(*TEXTC)
            cr.set_font_size(11)
            cr.move_to(10, h / 2 + 4)
            cr.show_text("instant")
            return

        steps = max(32, int(w))
        cr.move_to(0, h)
        for i in range(steps + 1):
            t = i / steps
            cr.line_to(w * t, h - self._value(t) * (h - 6))
        cr.set_source_rgb(*ACCENT)
        cr.set_line_width(2)
        cr.stroke()

        cr.set_source_rgba(*TEXTC)
        cr.set_font_size(9)
        cr.move_to(w - 52, h - 5)
        cr.show_text(f"{self.ms} ms")
