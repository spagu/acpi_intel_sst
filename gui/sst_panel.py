#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Control panel for the acpi_intel_sst driver.

The driver exposes 26 sysctls covering a parametric equaliser, a limiter,
volume ramps, jack detection and live telemetry. They are perfectly usable
from the command line, but tuning an equaliser by typing sysctl names is
miserable and the peak meters are only meaningful if you can watch them move.

Writes need privilege. Rather than demanding to be run as root, the panel
checks whether it can write and says so plainly, leaving the controls visible
but disabled - an unprivileged user can still watch the telemetry.
"""

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk  # noqa: E402

import sst_sysctl as sysctl  # noqa: E402

REFRESH_MS = 250

# Meters are drawn 0..DB_FLOOR because GtkLevelBar will not take a negative
# minimum; -DB_FLOOR dB maps to 0 and 0 dB to DB_FLOOR.
DB_FLOOR = 96.0


def _db_to_bar(db):
    return max(0.0, min(DB_FLOOR, db + DB_FLOOR))


EQ_PRESETS = [
    "płaski", "wzmocniony bas", "wzmocniona mowa",
    "wzmocniona góra", "własny",
]
RAMP_CURVES = ["liniowa", "wykładnicza", "logarytmiczna"]


class Row:
    """A labelled control bound to one sysctl."""

    def __init__(self, grid, row, label, widget, suffix=None, tip=None):
        lab = Gtk.Label(label=label, xalign=0)
        if tip:
            lab.set_tooltip_text(tip)
            widget.set_tooltip_text(tip)
        grid.attach(lab, 0, row, 1, 1)
        grid.attach(widget, 1, row, 1, 1)
        if suffix:
            grid.attach(Gtk.Label(label=suffix, xalign=0), 2, row, 1, 1)
        self.widget = widget


def scale(lower, upper, step=1):
    s = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, lower, upper, step)
    s.set_value_pos(Gtk.PositionType.RIGHT)
    s.set_digits(0)
    s.set_hexpand(True)
    s.set_size_request(280, -1)
    return s


def grid():
    g = Gtk.Grid(column_spacing=12, row_spacing=8)
    g.set_margin_top(12)
    g.set_margin_bottom(12)
    g.set_margin_start(12)
    g.set_margin_end(12)
    return g


class Panel(Gtk.Window):
    def __init__(self):
        super().__init__(title="Intel SST — panel sterowania")
        self.set_default_size(560, 460)
        self.set_border_width(0)

        self.can_write = sysctl.writable()
        self._loading = True

        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(outer)

        if not self.can_write:
            outer.pack_start(self._readonly_banner(), False, False, 0)

        nb = Gtk.Notebook()
        outer.pack_start(nb, True, True, 0)
        nb.append_page(self._page_eq(), Gtk.Label(label="Korektor"))
        nb.append_page(self._page_limiter(), Gtk.Label(label="Limiter"))
        nb.append_page(self._page_ramps(), Gtk.Label(label="Rampy"))
        nb.append_page(self._page_jack(), Gtk.Label(label="Gniazdo"))
        nb.append_page(self._page_diag(), Gtk.Label(label="Diagnostyka"))

        self.status = Gtk.Label(xalign=0)
        self.status.set_margin_start(12)
        self.status.set_margin_end(12)
        self.status.set_margin_bottom(8)
        outer.pack_start(self.status, False, False, 0)

        self._load()
        self._loading = False
        GLib.timeout_add(REFRESH_MS, self._tick)

    def _readonly_banner(self):
        bar = Gtk.InfoBar(message_type=Gtk.MessageType.INFO)
        bar.get_content_area().add(Gtk.Label(
            label="Tylko podgląd — zmiana ustawień wymaga uprawnień roota.",
            xalign=0))
        return bar

    # ---------------------------------------------------------------- pages

    def _page_eq(self):
        g = grid()
        self.eq_preset = Gtk.ComboBoxText()
        for p in EQ_PRESETS:
            self.eq_preset.append_text(p)
        Row(g, 0, "Zestaw", self.eq_preset,
            tip="Gotowy zestaw ustawień korektora")

        self.peq_freq = scale(0, 20000, 10)
        Row(g, 1, "Częstotliwość", self.peq_freq, "Hz",
            tip="Środek pasma korygowanego przez filtr parametryczny")
        self.peq_gain = scale(-12, 12)
        Row(g, 2, "Wzmocnienie", self.peq_gain, "dB",
            tip="Wzmocnienie lub tłumienie w tym paśmie")
        self.peq_q = scale(1, 200)
        Row(g, 3, "Dobroć Q", self.peq_q, "×0,01",
            tip="Szerokość pasma: wyższe Q to węższy zakres")
        self.hpf = scale(0, 500, 10)
        Row(g, 4, "Filtr górnoprzepustowy", self.hpf, "Hz",
            tip="Odcina najniższe częstotliwości, których i tak nie odtworzą "
                "małe głośniki laptopa — odciąża je przy dużej głośności")

        self._bind(self.eq_preset, "eq_preset", combo=True)
        self._bind(self.peq_freq, "peq_freq")
        self._bind(self.peq_gain, "peq_gain")
        self._bind(self.peq_q, "peq_q")
        self._bind(self.hpf, "hpf_cutoff")
        return g

    def _page_limiter(self):
        g = grid()
        self.lim_thr = scale(0, 20)
        Row(g, 0, "Próg", self.lim_thr, "dB",
            tip="Poziom, powyżej którego limiter zaczyna tłumić sygnał")
        self.lim_rel = scale(0, 1000, 10)
        Row(g, 1, "Czas zwolnienia", self.lim_rel, "ms",
            tip="Jak szybko limiter puszcza sygnał po ustąpieniu szczytu")

        self.lim_active = Gtk.Label(xalign=0)
        Row(g, 2, "Stan", self.lim_active,
            tip="Czy limiter tłumi sygnał w tej chwili")

        g.attach(Gtk.Separator(), 0, 3, 3, 1)

        # LevelBar refuses a negative minimum, so the meters run 0..96 and
        # the dB value is mapped onto that: -96 dB is silence at the left,
        # 0 dB is full scale at the right.
        self.peak_l = Gtk.LevelBar.new_for_interval(0, DB_FLOOR)
        self.peak_l.set_hexpand(True)
        Row(g, 4, "Szczyt L", self.peak_l, "dB")
        self.peak_r = Gtk.LevelBar.new_for_interval(0, DB_FLOOR)
        self.peak_r.set_hexpand(True)
        Row(g, 5, "Szczyt P", self.peak_r, "dB")
        self.peak_txt = Gtk.Label(xalign=0)
        Row(g, 6, "Poziom", self.peak_txt)

        self.clip = Gtk.Label(xalign=0)
        Row(g, 7, "Przesterowania", self.clip,
            tip="Ile razy sygnał przekroczył zakres od ostatniego zerowania")
        btn = Gtk.Button(label="Wyzeruj licznik")
        btn.connect("clicked", self._reset_clip)
        btn.set_sensitive(self.can_write)
        g.attach(btn, 1, 8, 1, 1)

        self._bind(self.lim_thr, "limiter_threshold")
        self._bind(self.lim_rel, "limiter_release")
        return g

    def _page_ramps(self):
        g = grid()
        self.ramp_ms = scale(0, 500, 10)
        Row(g, 0, "Czas rampy", self.ramp_ms, "ms",
            tip="Płynne dojście do zadanej głośności zamiast skoku")
        self.resume_ms = scale(0, 2000, 10)
        Row(g, 1, "Rampa po wybudzeniu", self.resume_ms, "ms",
            tip="To samo po wyjściu z uśpienia, gdzie zwykle warto dłużej")
        self.ramp_curve = Gtk.ComboBoxText()
        for c in RAMP_CURVES:
            self.ramp_curve.append_text(c)
        Row(g, 2, "Kształt", self.ramp_curve,
            tip="Jak głośność narasta w czasie rampy")

        self._bind(self.ramp_ms, "ramp_ms")
        self._bind(self.resume_ms, "resume_ramp_ms")
        self._bind(self.ramp_curve, "ramp_curve", combo=True)
        return g

    def _page_jack(self):
        g = grid()
        self.jack_on = Gtk.Switch(halign=Gtk.Align.START)
        Row(g, 0, "Wykrywanie gniazda", self.jack_on,
            tip="Przełączanie na słuchawki po ich podłączeniu")
        self.jack_hp = Gtk.Label(xalign=0)
        Row(g, 1, "Słuchawki", self.jack_hp)
        self.jack_mic = Gtk.Label(xalign=0)
        Row(g, 2, "Mikrofon", self.jack_mic)
        self.jack_hp_n = Gtk.Label(xalign=0)
        Row(g, 3, "Podłączeń słuchawek", self.jack_hp_n)
        self.jack_polls = Gtk.Label(xalign=0)
        Row(g, 4, "Odpytań", self.jack_polls,
            tip="Licznik sondowania gniazda — rośnie, dopóki wykrywanie działa")

        self.jack_on.set_sensitive(self.can_write)
        self.jack_on.connect("state-set", self._jack_toggled)
        return g

    def _page_diag(self):
        g = grid()
        self.debug = scale(0, 3)
        Row(g, 0, "Poziom diagnostyki", self.debug,
            tip="0 tylko błędy, 3 pełne śledzenie. Wyższe poziomy potrafią "
                "zalać dziennik systemowy")
        self.i2c_err = Gtk.Label(xalign=0)
        Row(g, 1, "Błędy I2C kodeka", self.i2c_err,
            tip="Nieudane rozmowy z kodekiem RT286 po magistrali I2C")
        self._bind(self.debug, "debug")
        return g

    # -------------------------------------------------------------- binding

    def _bind(self, widget, name, combo=False):
        """Write the sysctl when the control moves, unless we are loading."""
        def changed(w):
            if self._loading or not self.can_write:
                return
            value = w.get_active() if combo else int(w.get_value())
            try:
                sysctl.set_int(name, value)
                self._say(f"{name} = {value}")
            except sysctl.ReadOnly as e:
                self._say(f"nie zapisano {name}: {e}", error=True)
        widget.connect("changed" if combo else "value-changed", changed)

    def _jack_toggled(self, _sw, state):
        try:
            sysctl.set_int("jack.enabled", 1 if state else 0)
        except sysctl.ReadOnly as e:
            self._say(str(e), error=True)
        return False

    def _reset_clip(self, _btn):
        try:
            sysctl.set_int("telemetry.clip_reset", 1)
            self._say("licznik przesterowań wyzerowany")
        except sysctl.ReadOnly as e:
            self._say(str(e), error=True)

    def _say(self, text, error=False):
        self.status.set_markup(
            f"<span foreground='#c0392b'>{GLib.markup_escape_text(text)}</span>"
            if error else GLib.markup_escape_text(text))

    # --------------------------------------------------------------- values

    def _load(self):
        """Pull current values into the controls without writing them back."""
        self.eq_preset.set_active(sysctl.get_int("eq_preset"))
        self.peq_freq.set_value(sysctl.get_int("peq_freq"))
        self.peq_gain.set_value(sysctl.get_int("peq_gain"))
        self.peq_q.set_value(sysctl.get_int("peq_q"))
        self.hpf.set_value(sysctl.get_int("hpf_cutoff"))
        self.lim_thr.set_value(sysctl.get_int("limiter_threshold"))
        self.lim_rel.set_value(sysctl.get_int("limiter_release"))
        self.ramp_ms.set_value(sysctl.get_int("ramp_ms"))
        self.resume_ms.set_value(sysctl.get_int("resume_ramp_ms"))
        self.ramp_curve.set_active(sysctl.get_int("ramp_curve"))
        self.debug.set_value(sysctl.get_int("debug"))
        self.jack_on.set_active(sysctl.get_int("jack.enabled") == 1)

    def _tick(self):
        """Refresh everything the driver reports rather than accepts."""
        dl = sysctl.get_float("telemetry.peak_db_left", -DB_FLOOR)
        dr = sysctl.get_float("telemetry.peak_db_right", -DB_FLOOR)
        self.peak_l.set_value(_db_to_bar(dl))
        self.peak_r.set_value(_db_to_bar(dr))
        self.peak_txt.set_text(f"L {dl:.1f} dB   P {dr:.1f} dB")

        self.lim_active.set_text(
            "tłumi" if sysctl.get_int("telemetry.limiter_active") else "bezczynny")
        self.clip.set_text(str(sysctl.get_int("telemetry.clip_count")))

        self.jack_hp.set_text(
            "podłączone" if sysctl.get_int("jack.headphone") else "odłączone")
        self.jack_mic.set_text(
            "podłączony" if sysctl.get_int("jack.microphone") else "odłączony")
        self.jack_hp_n.set_text(str(sysctl.get_int("jack.hp_insertions")))
        self.jack_polls.set_text(str(sysctl.get_int("jack.poll_count")))
        self.i2c_err.set_text(str(sysctl.get_int("codec.i2c_errors")))
        return True


def main():
    if not sysctl.available():
        d = Gtk.MessageDialog(
            message_type=Gtk.MessageType.ERROR, buttons=Gtk.ButtonsType.CLOSE,
            text="Sterownik acpi_intel_sst nie jest załadowany.")
        d.format_secondary_text(
            "Załaduj go poleceniem  kldload acpi_intel_sst  i uruchom panel "
            "ponownie.")
        d.run()
        return 1

    w = Panel()
    w.connect("destroy", Gtk.main_quit)
    w.show_all()
    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
