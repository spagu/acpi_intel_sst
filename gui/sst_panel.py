#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Control panel for the acpi_intel_sst driver.

The driver exposes 26 sysctls: a parametric equaliser, a limiter, volume
ramps, jack detection and live telemetry. They work from a shell, but tuning
an equaliser by typing sysctl names is miserable and the peak meters only mean
anything if you can watch them move.

Source strings are English; translations live in po/ and follow the user's
locale. Writes need privilege, so the panel probes whether it can write and
degrades to a live read-only view rather than demanding to be run as root.
"""

import gi

# Both namespaces need pinning before the first import: without it Gdk
# resolves to 4.0 while Gtk is 3.0, and the import fails outright.
gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, GLib, Gtk  # noqa: E402

import sst_curves as curves  # noqa: E402
import sst_i18n as i18n  # noqa: E402
import sst_meters as meters  # noqa: E402
import sst_sysctl as sysctl  # noqa: E402

_ = i18n._

REFRESH_MS = 250

CSS = b"""
.sst-header-sub { font-size: 90%; opacity: 0.65; }
.sst-card {
    background: alpha(@theme_fg_color, 0.04);
    border: 1px solid alpha(@theme_fg_color, 0.10);
    border-radius: 10px;
    padding: 14px;
}
.sst-section { font-weight: bold; }
.sst-hint { font-size: 90%; opacity: 0.65; }
.sst-reading { font-family: monospace; font-size: 115%; }
.sst-badge {
    border-radius: 999px;
    padding: 2px 10px;
    font-size: 88%;
}
.sst-badge-on  { background: alpha(#e67e22, 0.22); }
.sst-badge-off { background: alpha(@theme_fg_color, 0.10); }
.sst-status { font-size: 90%; opacity: 0.75; }
"""


def _preset_names():
    return [_("Flat"), _("Bass boost"), _("Voice boost"),
            _("Treble boost"), _("Custom")]


def _curve_names():
    return [_("Linear"), _("Exponential"), _("Logarithmic")]


def card(title=None):
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    box.get_style_context().add_class("sst-card")
    if title:
        lab = Gtk.Label(label=title, xalign=0)
        lab.get_style_context().add_class("sst-section")
        box.pack_start(lab, False, False, 0)
    return box


def grid():
    g = Gtk.Grid(column_spacing=14, row_spacing=10)
    g.set_column_homogeneous(False)
    return g


def scale(lower, upper, step=1):
    s = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, lower, upper, step)
    s.set_value_pos(Gtk.PositionType.RIGHT)
    s.set_digits(0)
    s.set_hexpand(True)
    s.set_size_request(260, -1)
    return s


def row(g, r, label, widget, suffix=None, hint=None):
    lab = Gtk.Label(label=label, xalign=0)
    lab.set_valign(Gtk.Align.CENTER)
    g.attach(lab, 0, r, 1, 1)
    g.attach(widget, 1, r, 1, 1)
    if suffix:
        s = Gtk.Label(label=suffix, xalign=0)
        s.get_style_context().add_class("sst-hint")
        g.attach(s, 2, r, 1, 1)
    if hint:
        lab.set_tooltip_text(hint)
        widget.set_tooltip_text(hint)
    return widget


def page():
    b = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
    b.set_margin_top(16)
    b.set_margin_bottom(16)
    b.set_margin_start(16)
    b.set_margin_end(16)
    return b


class Panel(Gtk.Window):
    def __init__(self):
        super().__init__(title=_("Intel SST Audio"))
        self.set_default_size(620, 580)

        self.can_write = sysctl.writable()
        self._loading = True

        self._css()
        self.set_titlebar(self._header())

        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(outer)

        if not self.can_write:
            outer.pack_start(self._readonly_bar(), False, False, 0)

        nb = Gtk.Notebook()
        nb.set_scrollable(True)
        outer.pack_start(nb, True, True, 0)
        nb.append_page(self._page_eq(), Gtk.Label(label=_("Equaliser")))
        nb.append_page(self._page_limiter(), Gtk.Label(label=_("Limiter")))
        nb.append_page(self._page_ramps(), Gtk.Label(label=_("Ramps")))
        nb.append_page(self._page_jack(), Gtk.Label(label=_("Jack")))
        nb.append_page(self._page_diag(), Gtk.Label(label=_("Diagnostics")))

        self.status = Gtk.Label(xalign=0)
        self.status.get_style_context().add_class("sst-status")
        self.status.set_margin_start(16)
        self.status.set_margin_end(16)
        self.status.set_margin_bottom(10)
        outer.pack_start(self.status, False, False, 0)

        self._load()
        self._refresh_curves()
        self._loading = False
        GLib.timeout_add(REFRESH_MS, self._tick)

    # ---------------------------------------------------------------- chrome

    def _css(self):
        p = Gtk.CssProvider()
        p.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), p,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    def _header(self):
        hb = Gtk.HeaderBar()
        hb.set_show_close_button(True)
        hb.set_title(_("Intel SST Audio"))
        hb.set_subtitle(_("Broadwell-U DSP") if self.can_write
                        else _("Broadwell-U DSP — read only"))

        combo = Gtk.ComboBoxText()
        for code, name in i18n.LANGUAGES.items():
            combo.append(code, name)
        combo.set_active_id(self._current_lang)
        combo.set_tooltip_text(_("Interface language"))
        combo.connect("changed", self._lang_changed)
        hb.pack_end(combo)
        return hb

    def _readonly_bar(self):
        bar = Gtk.InfoBar(message_type=Gtk.MessageType.INFO)
        bar.get_content_area().add(Gtk.Label(
            label=_("Viewing only — changing settings needs root privileges."),
            xalign=0))
        return bar

    def _lang_changed(self, combo):
        code = combo.get_active_id()
        if not code or code == self._current_lang:
            return
        i18n.activate(code)
        d = Gtk.MessageDialog(
            transient_for=self, modal=True,
            message_type=Gtk.MessageType.INFO, buttons=Gtk.ButtonsType.OK,
            text=_("Restart the panel to apply the new language."))
        d.run()
        d.destroy()

    # ----------------------------------------------------------------- pages

    def _page_eq(self):
        p = page()

        c = card(_("Response"))
        self.eq_curve = curves.EqCurve()
        c.pack_start(self.eq_curve, False, False, 0)
        hint = Gtk.Label(xalign=0, label=_(
            "Combined response of the parametric band and the high-pass "
            "filter. Follows the sliders below."))
        hint.get_style_context().add_class("sst-hint")
        hint.set_line_wrap(True)
        c.pack_start(hint, False, False, 0)
        p.pack_start(c, False, False, 0)

        c = card(_("Parametric band"))
        g = grid()
        self.eq_preset = Gtk.ComboBoxText()
        for n in _preset_names():
            self.eq_preset.append_text(n)
        row(g, 0, _("Preset"), self.eq_preset,
            hint=_("A ready-made set of equaliser settings"))
        self.peq_freq = scale(0, 20000, 10)
        row(g, 1, _("Centre frequency"), self.peq_freq, _("Hz"),
            _("Middle of the band the parametric filter acts on"))
        self.peq_gain = scale(-12, 12)
        row(g, 2, _("Gain"), self.peq_gain, _("dB"),
            _("Boost or cut applied within that band"))
        self.peq_q = scale(1, 200)
        row(g, 3, _("Q factor"), self.peq_q, _("×0.01"),
            _("Bandwidth: a higher Q affects a narrower range"))
        c.pack_start(g, False, False, 0)
        p.pack_start(c, False, False, 0)

        c2 = card(_("High-pass filter"))
        g2 = grid()
        self.hpf = scale(0, 500, 10)
        row(g2, 0, _("Cutoff"), self.hpf, _("Hz"),
            _("Removes the lowest frequencies that small laptop speakers "
              "cannot reproduce anyway, which relieves them at high volume"))
        c2.pack_start(g2, False, False, 0)
        h = Gtk.Label(xalign=0, label=_("0 disables the filter."))
        h.get_style_context().add_class("sst-hint")
        c2.pack_start(h, False, False, 0)
        p.pack_start(c2, False, False, 0)

        self._bind(self.eq_preset, "eq_preset", combo=True)
        self._bind(self.peq_freq, "peq_freq")
        self._bind(self.peq_gain, "peq_gain")
        self._bind(self.peq_q, "peq_q")
        self._bind(self.hpf, "hpf_cutoff")
        return p

    def _page_limiter(self):
        p = page()

        c = card(_("Output level"))
        self.meter_l = meters.PeakMeter()
        self.meter_r = meters.PeakMeter()
        for lbl, m in ((_("Left"), self.meter_l), (_("Right"), self.meter_r)):
            b = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
            la = Gtk.Label(label=lbl, xalign=0)
            la.set_size_request(52, -1)
            b.pack_start(la, False, False, 0)
            b.pack_start(m, True, True, 0)
            c.pack_start(b, False, False, 0)

        self.peak_txt = Gtk.Label(xalign=0)
        self.peak_txt.get_style_context().add_class("sst-reading")
        c.pack_start(self.peak_txt, False, False, 0)

        self.spark = meters.Sparkline()
        c.pack_start(self.spark, False, False, 0)
        sh = Gtk.Label(xalign=0, label=_("Last few seconds"))
        sh.get_style_context().add_class("sst-hint")
        c.pack_start(sh, False, False, 0)
        p.pack_start(c, False, False, 0)

        c2 = card(_("Limiter"))
        self.lim_curve = curves.LimiterCurve()
        c2.pack_start(self.lim_curve, False, False, 0)
        lh = Gtk.Label(xalign=0, label=_(
            "Input against output. The dashed line is what you would get "
            "with no limiter at all."))
        lh.get_style_context().add_class("sst-hint")
        lh.set_line_wrap(True)
        c2.pack_start(lh, False, False, 0)
        g = grid()
        self.lim_thr = scale(0, 20)
        row(g, 0, _("Threshold"), self.lim_thr, _("dB"),
            _("Level above which the limiter starts attenuating"))
        self.lim_rel = scale(0, 1000, 10)
        row(g, 1, _("Release"), self.lim_rel, _("ms"),
            _("How quickly it lets go once the peak has passed"))
        c2.pack_start(g, False, False, 0)

        b = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.lim_badge = Gtk.Label(label=_("idle"))
        self.lim_badge.get_style_context().add_class("sst-badge")
        self.lim_badge.get_style_context().add_class("sst-badge-off")
        b.pack_start(self.lim_badge, False, False, 0)
        self.clip = Gtk.Label(xalign=0)
        b.pack_start(self.clip, False, False, 0)
        btn = Gtk.Button(label=_("Reset counter"))
        btn.connect("clicked", self._reset_clip)
        btn.set_sensitive(self.can_write)
        b.pack_end(btn, False, False, 0)
        c2.pack_start(b, False, False, 0)
        p.pack_start(c2, False, False, 0)

        self._bind(self.lim_thr, "limiter_threshold")
        self._bind(self.lim_rel, "limiter_release")
        return p

    def _page_ramps(self):
        p = page()
        c = card(_("Volume ramps"))
        self.ramp_curve_plot = curves.RampCurve()
        c.pack_start(self.ramp_curve_plot, False, False, 0)
        rh = Gtk.Label(xalign=0, label=_("Volume over the length of the ramp."))
        rh.get_style_context().add_class("sst-hint")
        c.pack_start(rh, False, False, 0)
        g = grid()
        self.ramp_ms = scale(0, 500, 10)
        row(g, 0, _("Ramp time"), self.ramp_ms, _("ms"),
            _("Ease into a new volume instead of jumping to it"))
        self.resume_ms = scale(0, 2000, 10)
        row(g, 1, _("After resume"), self.resume_ms, _("ms"),
            _("The same after waking from sleep, where longer usually suits"))
        self.ramp_curve = Gtk.ComboBoxText()
        for n in _curve_names():
            self.ramp_curve.append_text(n)
        row(g, 2, _("Shape"), self.ramp_curve,
            hint=_("How the volume travels over the ramp"))
        c.pack_start(g, False, False, 0)
        p.pack_start(c, False, False, 0)

        self._bind(self.ramp_ms, "ramp_ms")
        self._bind(self.resume_ms, "resume_ramp_ms")
        self._bind(self.ramp_curve, "ramp_curve", combo=True)
        return p

    def _page_jack(self):
        p = page()
        c = card(_("Jack detection"))
        g = grid()
        self.jack_on = Gtk.Switch(halign=Gtk.Align.START)
        row(g, 0, _("Enabled"), self.jack_on,
            hint=_("Switch to headphones when they are plugged in"))
        self.jack_hp = Gtk.Label(xalign=0)
        row(g, 1, _("Headphones"), self.jack_hp)
        self.jack_mic = Gtk.Label(xalign=0)
        row(g, 2, _("Microphone"), self.jack_mic)
        c.pack_start(g, False, False, 0)
        p.pack_start(c, False, False, 0)

        c2 = card(_("Counters"))
        g2 = grid()
        self.jack_hp_n = Gtk.Label(xalign=0)
        row(g2, 0, _("Headphone insertions"), self.jack_hp_n)
        self.jack_polls = Gtk.Label(xalign=0)
        row(g2, 1, _("Polls"), self.jack_polls,
            hint=_("Rises for as long as detection is running"))
        c2.pack_start(g2, False, False, 0)
        p.pack_start(c2, False, False, 0)

        self.jack_on.set_sensitive(self.can_write)
        self.jack_on.connect("state-set", self._jack_toggled)
        return p

    def _page_diag(self):
        p = page()
        c = card(_("Driver"))
        g = grid()
        self.debug = scale(0, 3)
        row(g, 0, _("Debug level"), self.debug, None,
            _("0 errors only, 3 full tracing. The higher levels can flood "
              "the system log"))
        self.i2c_err = Gtk.Label(xalign=0)
        row(g, 1, _("Codec I2C errors"), self.i2c_err,
            hint=_("Failed exchanges with the RT286 codec over I2C"))
        c.pack_start(g, False, False, 0)
        p.pack_start(c, False, False, 0)
        self._bind(self.debug, "debug")
        return p

    # --------------------------------------------------------------- plumbing

    def _refresh_curves(self):
        """Redraw the plots from the current control positions.

        Driven by the widgets rather than by the sysctls so the picture
        follows the slider even when the write is refused - an unprivileged
        user can still see what a setting would do.
        """
        if hasattr(self, "eq_curve"):
            self.eq_curve.update(self.peq_freq.get_value(),
                                 self.peq_gain.get_value(),
                                 self.peq_q.get_value(),
                                 self.hpf.get_value())
        if hasattr(self, "lim_curve"):
            self.lim_curve.update(self.lim_thr.get_value())
        if hasattr(self, "ramp_curve_plot"):
            self.ramp_curve_plot.update(self.ramp_curve.get_active(),
                                        self.ramp_ms.get_value())

    def _bind(self, widget, name, combo=False):
        def changed(w):
            self._refresh_curves()
            if self._loading or not self.can_write:
                return
            value = w.get_active() if combo else int(w.get_value())
            try:
                sysctl.set_int(name, value)
                self._say(f"{name} = {value}")
            except sysctl.ReadOnly as e:
                self._say(_("could not write {0}: {1}").format(name, e), True)
        widget.connect("changed" if combo else "value-changed", changed)

    def _jack_toggled(self, _sw, state):
        try:
            sysctl.set_int("jack.enabled", 1 if state else 0)
        except sysctl.ReadOnly as e:
            self._say(str(e), True)
        return False

    def _reset_clip(self, _btn):
        try:
            sysctl.set_int("telemetry.clip_reset", 1)
            self._say(_("Clip counter reset."))
        except sysctl.ReadOnly as e:
            self._say(str(e), True)

    def _say(self, text, error=False):
        esc = GLib.markup_escape_text(text)
        self.status.set_markup(
            f"<span foreground='#c0392b'>{esc}</span>" if error else esc)

    def _load(self):
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
        dl = sysctl.get_float("telemetry.peak_db_left", meters.DB_FLOOR)
        dr = sysctl.get_float("telemetry.peak_db_right", meters.DB_FLOOR)
        self.meter_l.set_db(dl)
        self.meter_r.set_db(dr)
        self.spark.push(max(dl, dr))
        self.peak_txt.set_text(f"L {dl:6.1f} dB     R {dr:6.1f} dB")

        active = sysctl.get_int("telemetry.limiter_active")
        self.lim_badge.set_text(_("limiting") if active else _("idle"))
        ctx = self.lim_badge.get_style_context()
        ctx.remove_class("sst-badge-on" if not active else "sst-badge-off")
        ctx.add_class("sst-badge-on" if active else "sst-badge-off")
        self.clip.set_text(
            _("clipped {0}×").format(sysctl.get_int("telemetry.clip_count")))

        self.jack_hp.set_text(
            _("connected") if sysctl.get_int("jack.headphone")
            else _("not connected"))
        self.jack_mic.set_text(
            _("connected") if sysctl.get_int("jack.microphone")
            else _("not connected"))
        self.jack_hp_n.set_text(str(sysctl.get_int("jack.hp_insertions")))
        self.jack_polls.set_text(str(sysctl.get_int("jack.poll_count")))
        self.i2c_err.set_text(str(sysctl.get_int("codec.i2c_errors")))
        return True


def main():
    Panel._current_lang = i18n.activate()

    if not sysctl.available():
        d = Gtk.MessageDialog(
            message_type=Gtk.MessageType.ERROR, buttons=Gtk.ButtonsType.CLOSE,
            text=_("The acpi_intel_sst driver is not loaded."))
        d.format_secondary_text(
            _("Load it with  kldload acpi_intel_sst  and start the panel "
              "again."))
        d.run()
        return 1

    w = Panel()
    w.connect("destroy", Gtk.main_quit)
    w.show_all()
    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
