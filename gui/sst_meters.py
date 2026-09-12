# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Peak meters drawn with Cairo.

GtkLevelBar can show a value, but it cannot show the two things that make a
meter worth looking at: where the signal sits relative to clipping, and where
it peaked a moment ago. Both matter when setting a limiter threshold, which is
what these are for.

The scale is decibels, so the gradient is placed at dB positions rather than
spread evenly: the amber band starts at -12 dB and red at -3 dB, which is
where the limiter starts earning its keep.
"""

import math

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk  # noqa: E402

DB_FLOOR = -96.0
DB_AMBER = -12.0
DB_RED = -3.0

# Peak hold, in refresh ticks. Long enough to read, short enough to follow.
HOLD_TICKS = 8


def _frac(db):
    """
    Position of a dB value along the meter, 0 at the floor and 1 at full.

    Linear in dB rather than in amplitude: a meter linear in amplitude spends
    almost its whole length on the top 6 dB and is useless for judging
    anything quiet.
    """
    if db <= DB_FLOOR:
        return 0.0
    if db >= 0.0:
        return 1.0
    return (db - DB_FLOOR) / (0.0 - DB_FLOOR)


class PeakMeter(Gtk.DrawingArea):
    """A horizontal peak meter with peak-hold."""

    def __init__(self, height=18):
        super().__init__()
        self.set_size_request(-1, height)
        self.set_hexpand(True)
        self._db = DB_FLOOR
        self._hold_db = DB_FLOOR
        self._hold_left = 0
        self.connect("draw", self._draw)

    def set_db(self, db):
        self._db = max(DB_FLOOR, min(0.0, db))
        if self._db >= self._hold_db:
            self._hold_db = self._db
            self._hold_left = HOLD_TICKS
        elif self._hold_left > 0:
            self._hold_left -= 1
        else:
            # decay the hold rather than dropping it, so the eye can follow
            self._hold_db = max(self._db, self._hold_db - 1.5)
        self.queue_draw()

    def _draw(self, _w, cr):
        a = self.get_allocation()
        w, h = a.width, a.height
        r = h / 2.0

        # track
        cr.set_source_rgba(0, 0, 0, 0.22)
        self._rounded(cr, 0, 0, w, h, r)
        cr.fill()

        level = _frac(self._db)
        if level > 0.001:
            grad = self._gradient(w)
            cr.save()
            self._rounded(cr, 0, 0, w, h, r)
            cr.clip()
            cr.set_source(grad)
            cr.rectangle(0, 0, w * level, h)
            cr.fill()
            cr.restore()

        # peak hold marker
        if self._hold_db > DB_FLOOR:
            x = w * _frac(self._hold_db)
            cr.set_source_rgba(1, 1, 1, 0.85)
            cr.rectangle(max(0, min(w - 2, x - 1)), 0, 2, h)
            cr.fill()

        # tick at the amber and red thresholds, so the scale is readable
        for db in (DB_AMBER, DB_RED):
            x = w * _frac(db)
            cr.set_source_rgba(1, 1, 1, 0.25)
            cr.rectangle(x, h * 0.25, 1, h * 0.5)
            cr.fill()

    def _gradient(self, w):
        from cairo import LinearGradient
        g = LinearGradient(0, 0, w, 0)
        g.add_color_stop_rgb(0.0, 0.16, 0.68, 0.38)
        g.add_color_stop_rgb(_frac(DB_AMBER), 0.35, 0.75, 0.30)
        g.add_color_stop_rgb(_frac(DB_AMBER) + 0.001, 0.95, 0.72, 0.13)
        g.add_color_stop_rgb(_frac(DB_RED), 0.95, 0.61, 0.13)
        g.add_color_stop_rgb(_frac(DB_RED) + 0.001, 0.86, 0.24, 0.20)
        g.add_color_stop_rgb(1.0, 0.78, 0.15, 0.13)
        return g

    @staticmethod
    def _rounded(cr, x, y, w, h, r):
        r = min(r, w / 2.0, h / 2.0)
        cr.new_sub_path()
        cr.arc(x + w - r, y + r, r, -math.pi / 2, 0)
        cr.arc(x + w - r, y + h - r, r, 0, math.pi / 2)
        cr.arc(x + r, y + h - r, r, math.pi / 2, math.pi)
        cr.arc(x + r, y + r, r, math.pi, 3 * math.pi / 2)
        cr.close_path()


class Sparkline(Gtk.DrawingArea):
    """
    A short rolling history of the signal level.

    A meter shows now; this shows the last few seconds, which is what tells
    you whether the limiter is catching occasional peaks or riding the signal
    continuously.
    """

    def __init__(self, samples=120, height=44):
        super().__init__()
        self.set_size_request(-1, height)
        self.set_hexpand(True)
        self._n = samples
        self._data = [0.0] * samples
        self.connect("draw", self._draw)

    def push(self, db):
        self._data.append(_frac(max(DB_FLOOR, min(0.0, db))))
        if len(self._data) > self._n:
            self._data.pop(0)
        self.queue_draw()

    def _draw(self, _w, cr):
        a = self.get_allocation()
        w, h = a.width, a.height

        cr.set_source_rgba(0, 0, 0, 0.18)
        cr.rectangle(0, 0, w, h)
        cr.fill()

        if not self._data:
            return
        step = w / float(max(1, len(self._data) - 1))

        cr.move_to(0, h)
        for i, v in enumerate(self._data):
            cr.line_to(i * step, h - v * h)
        cr.line_to(w, h)
        cr.close_path()
        cr.set_source_rgba(0.16, 0.68, 0.38, 0.30)
        cr.fill_preserve()
        cr.set_source_rgba(0.16, 0.68, 0.38, 0.9)
        cr.set_line_width(1.5)
        cr.stroke()
