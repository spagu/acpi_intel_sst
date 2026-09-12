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
import sst_help as help_  # noqa: E402
import sst_i18n as i18n  # noqa: E402
import sst_meters as meters  # noqa: E402
import sst_sysctl as sysctl  # noqa: E402
import sst_testtone as tone  # noqa: E402

_ = i18n._

REFRESH_MS = 250

CSS = b"""
.sst-title { font-size: 125%; font-weight: bold; }
.sst-header-sub { font-size: 90%; opacity: 0.65; }
.sst-titlebar { border-bottom: 1px solid alpha(@theme_fg_color, 0.10); }
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
.sst-help {
    padding: 0; min-height: 22px; min-width: 22px;
    opacity: 0.5; font-weight: bold;
    border-radius: 999px;
    background: alpha(@theme_fg_color, 0.10);
}
.sst-help:hover { opacity: 1.0; }
"""


def _preset_names():
    """
    The presets the driver actually has, in its own order.

    These are `enum sst_eq_preset_id` in src/sst_topology.h: a bypass and two
    high-pass filters that protect the speaker. The list used to read "Flat,
    Bass boost, Voice boost, Treble boost, Custom", which described a tone
    control this driver has never had - and the last two indexes do not exist,
    so selecting either, or running the preset comparison past the third,
    failed with "Invalid argument" straight from the kernel.
    """
    return [_("Flat (bypass)"),
            _("Stock speaker (150 Hz high-pass)"),
            _("Modified speaker (100 Hz high-pass)")]


def _curve_names():
    return [_("Linear"), _("Exponential"), _("Logarithmic")]


def card(title=None, topic=None):
    """
    A titled group of controls.

    Passing a topic puts a "?" beside the title; the explanation behind it is
    the place for anything that needs more than a tooltip.
    """
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    box.get_style_context().add_class("sst-card")
    if title and topic:
        box.pack_start(help_.section(title, topic), False, False, 0)
    elif title:
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


def _scrolled(page):
    """
    Give a notebook page its own scroller.

    Without one the notebook's minimum height is the tallest page's natural
    height, and GTK never shrinks a window below its minimum - set_default_size
    cannot override it. The tallest page asks for 1047 points, measured with
    get_allocation() on the target machine, whose 3200x1800 display at scale 2
    leaves 973 points of height. The window therefore opened taller than the
    screen it was on.
    """
    sw = Gtk.ScrolledWindow()
    sw.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
    sw.add(page)
    return sw


class Panel(Gtk.Window):
    def __init__(self):
        super().__init__(title=_("Intel SST Audio"))
        self.set_default_size(620, 580)

        self.can_write = sysctl.writable()
        self._elevation = False   # not probed yet; None = no route
        self._loading = True
        self._player = None
        self._comparing = False
        self._saved_preset = None

        self._css()

        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(outer)
        outer.pack_start(self._header(), False, False, 0)

        if not self.can_write:
            outer.pack_start(self._readonly_bar(), False, False, 0)

        nb = self.notebook = Gtk.Notebook()
        nb.set_scrollable(True)
        outer.pack_start(nb, True, True, 0)
        nb.append_page(_scrolled(self._page_eq()),
                       Gtk.Label(label=_("Equaliser")))
        nb.append_page(_scrolled(self._page_limiter()),
                       Gtk.Label(label=_("Limiter")))
        nb.append_page(_scrolled(self._page_ramps()),
                       Gtk.Label(label=_("Ramps")))
        nb.append_page(_scrolled(self._page_jack()),
                       Gtk.Label(label=_("Jack")))
        nb.append_page(_scrolled(self._page_diag()),
                       Gtk.Label(label=_("Diagnostics")))
        nb.append_page(_scrolled(self._page_info()),
                       Gtk.Label(label=_("Info")))

        self.status = Gtk.Label(xalign=0)
        self.status.get_style_context().add_class("sst-status")
        self.status.set_margin_start(16)
        self.status.set_margin_end(16)
        self.status.set_margin_bottom(10)
        outer.pack_start(self.status, False, False, 0)

        self.connect("destroy", lambda *_a: self._stop_player())

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
        """
        The title strip.

        Deliberately an ordinary widget inside the window rather than a
        GtkHeaderBar set as the titlebar. Client-side decorations gave no
        close, minimise or maximise buttons at all under this window manager,
        whatever gtk-decoration-layout said - and a window you cannot close
        with the mouse is a worse outcome than a slightly plainer frame.
        Leaving the frame to the window manager gets the usual buttons back.
        """
        bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        bar.get_style_context().add_class("sst-titlebar")
        bar.set_margin_top(10)
        bar.set_margin_bottom(10)
        bar.set_margin_start(16)
        bar.set_margin_end(16)

        titles = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        t = Gtk.Label(label=_("Intel SST Audio"), xalign=0)
        t.get_style_context().add_class("sst-title")
        titles.pack_start(t, False, False, 0)
        sub = Gtk.Label(xalign=0, label=_("Broadwell-U DSP") if self.can_write
                        else _("Broadwell-U DSP — read only"))
        sub.get_style_context().add_class("sst-header-sub")
        titles.pack_start(sub, False, False, 0)
        bar.pack_start(titles, False, False, 0)

        combo = Gtk.ComboBoxText()
        for code, name in i18n.LANGUAGES.items():
            combo.append(code, name)
        combo.set_active_id(self._current_lang)
        combo.set_tooltip_text(_("Interface language"))
        combo.connect("changed", self._lang_changed)
        combo.set_valign(Gtk.Align.CENTER)
        bar.pack_end(combo, False, False, 0)
        return bar

    def _readonly_bar(self):
        bar = Gtk.InfoBar(message_type=Gtk.MessageType.INFO)
        can_elevate = self._elevation_command() is not None
        bar.get_content_area().add(Gtk.Label(
            label=(_("Viewing only — changing settings needs root "
                     "privileges.") if can_elevate else
                   _("Viewing only — this account cannot gain the root "
                     "privileges that changing settings needs.")),
            xalign=0))
        if can_elevate:
            btn = bar.add_button(_("Run as administrator"),
                                 Gtk.ResponseType.OK)
            btn.set_tooltip_text(
                _("Start a second copy with the privileges needed to change "
                  "settings. This one stays open until it appears."))
            bar.connect("response", self._elevate)
        return bar

    # The elevated panel keeps running, so "still alive" is the success
    # signal; a refused elevation exits almost at once.
    ELEVATE_SETTLE_S = 2.0

    def _elevation_command(self):
        """
        The first command that can actually raise privilege, or None.

        Order matters:

        sudo -n   asks for nothing when the account has NOPASSWD, which is
                  the usual arrangement on an administrator's own laptop.
        sudo -A   pops a password dialog through an askpass helper.
        pkexec    comes last. When it finds no registered graphical polkit
                  agent for the caller's session it falls back to a *textual*
                  agent, and a panel started from a desktop menu has no
                  controlling terminal to put that on: it dies with "Error
                  opening current controlling terminal", having asked for
                  nothing and started nothing.

        Returns (argv, extra environment, whether it prompts), or None when
        no route exists - an account outside sudoers with no polkit rights
        is better off without a button that cannot do anything.
        """
        import os
        import shutil
        import subprocess

        if self._elevation is not False:
            return self._elevation

        target = shutil.which("sst-panel") or os.path.abspath(__file__)
        launcher = ([target] if target.endswith("sst-panel")
                    else ["python3", target])

        found = None
        sudo = shutil.which("sudo")
        if sudo:
            try:
                free = subprocess.run([sudo, "-n", "true"],
                                      stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL,
                                      timeout=5).returncode == 0
            except (OSError, subprocess.SubprocessError):
                free = False
            if free:
                found = ([sudo, "-n", "-E"] + launcher, {}, False)
            else:
                askpass = (shutil.which("ksshaskpass")
                           or shutil.which("ssh-askpass"))
                if askpass:
                    found = ([sudo, "-A", "-E"] + launcher,
                             {"SUDO_ASKPASS": askpass}, True)
        if found is None and shutil.which("pkexec"):
            disp = os.environ.get("DISPLAY", ":0")
            xauth = os.environ.get("XAUTHORITY",
                                   os.path.expanduser("~/.Xauthority"))
            found = (["pkexec", "env", f"DISPLAY={disp}",
                      f"XAUTHORITY={xauth}"] + launcher, {}, True)

        self._elevation = found
        return found

    def _elevate(self, _bar, _response):
        """
        Relaunch with privilege, then check that it actually happened.

        The first version of this treated Popen() returning as success. That
        only means the binary could be executed, so a pkexec that gave up at
        once still left "Starting with administrator privileges..." on screen
        while nothing started, its explanation thrown away with stderr. The
        child now gets a moment, and if it is already gone its own last line
        of stderr is what the user is shown.
        """
        import os
        import subprocess
        import tempfile

        chosen = self._elevation_command()
        if chosen is None:
            self._say(_("No way to elevate privileges was found. Run "
                        "\"sudo sst-panel\" from a terminal instead."), True)
            return
        cmd, extra, prompts = chosen

        env = dict(os.environ)
        env["DISPLAY"] = os.environ.get("DISPLAY", ":0")
        env["XAUTHORITY"] = os.environ.get(
            "XAUTHORITY", os.path.expanduser("~/.Xauthority"))
        env.update(extra)

        log = tempfile.NamedTemporaryFile(prefix="sst-panel-elevate-",
                                          suffix=".log", delete=False)
        try:
            child = subprocess.Popen(cmd, env=env,
                                     stdout=subprocess.DEVNULL, stderr=log)
        except OSError as exc:
            log.close()
            os.unlink(log.name)
            self._say(_("Could not start the elevated panel: {}")
                      .format(exc), True)
            return

        self._say(_("Asking for your password…") if prompts
                  else _("Starting with administrator privileges…"))
        GLib.timeout_add(int(self.ELEVATE_SETTLE_S * 1000),
                         self._elevation_settled, child, log.name)

    def _elevation_settled(self, child, logname):
        import os

        if child.poll() is None:
            return False        # running: it took, or a dialog is still open
        try:
            with open(logname, encoding="utf-8", errors="replace") as fh:
                lines = [ln.strip() for ln in fh if ln.strip()]
        except OSError:
            lines = []
        finally:
            try:
                os.unlink(logname)
            except OSError:
                pass
        self._say(lines[-1] if lines
                  else _("The elevated panel stopped immediately."), True)
        return False

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

        c = card(_("Response"), "eq")
        self.eq_curve = curves.EqCurve()
        c.pack_start(self.eq_curve, False, False, 0)
        hint = Gtk.Label(xalign=0, label=_(
            "Combined response of the parametric band and the high-pass "
            "filter. Follows the sliders below."))
        hint.get_style_context().add_class("sst-hint")
        hint.set_line_wrap(True)
        c.pack_start(hint, False, False, 0)
        p.pack_start(c, False, False, 0)

        c = card(_("Parametric band"), "eq")
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

        c2 = card(_("High-pass filter"), "hpf")
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

        c3 = card(_("Listen"))
        lh = Gtk.Label(xalign=0, label=_(
            "A sweep from 40 Hz to 16 kHz. It visits every band in turn, so a "
            "boost is audible as the sweep passes through it rather than "
            "hidden behind whatever a piece of music happens to be doing."))
        lh.get_style_context().add_class("sst-hint")
        lh.set_line_wrap(True)
        c3.pack_start(lh, False, False, 0)

        tb = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.btn_play = Gtk.Button(label=_("Play sweep"))
        self.btn_play.connect("clicked", self._play_once)
        tb.pack_start(self.btn_play, False, False, 0)

        self.btn_compare = Gtk.Button(label=_("Compare all presets"))
        self.btn_compare.connect("clicked", self._compare_presets)
        self.btn_compare.set_sensitive(self.can_write)
        if not self.can_write:
            self.btn_compare.set_tooltip_text(
                _("Switching presets needs root privileges"))
        tb.pack_start(self.btn_compare, False, False, 0)

        self.play_state = Gtk.Label(xalign=0)
        self.play_state.get_style_context().add_class("sst-reading")
        tb.pack_end(self.play_state, False, False, 0)
        c3.pack_start(tb, False, False, 0)
        p.pack_start(c3, False, False, 0)

        self._bind(self.eq_preset, "eq_preset", combo=True)
        self._bind(self.peq_freq, "peq_freq")
        self._bind(self.peq_gain, "peq_gain")
        self._bind(self.peq_q, "peq_q")
        self._bind(self.hpf, "hpf_cutoff")
        return p

    def _page_limiter(self):
        p = page()

        c = card(_("Output level"), "meters")
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

        c2 = card(_("Limiter"), "limiter")
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
        c = card(_("Volume ramps"), "ramps")
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
        c = card(_("Jack detection"), "jack")
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

        c2 = card(_("Counters"), "jack")
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
        c = card(_("Driver"), "debug")
        g = grid()
        self.debug = scale(0, 3)
        row(g, 0, _("Debug level"), self.debug, None,
            _("0 errors only, 3 full tracing. The higher levels can flood "
              "the system log"))
        self.recoveries = Gtk.Label(xalign=0)
        row(g, 2, _("DSP recoveries"), self.recoveries,
            hint=_("Times the driver reset a DSP that had stopped accepting "
                   "streams. Occasional entries are the self-recovery doing "
                   "its job; a number that climbs steadily is worth reporting"))

        self.i2c_err = Gtk.Label(xalign=0)
        i2c_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        i2c_box.pack_start(self.i2c_err, False, False, 0)
        i2c_box.pack_start(help_.HelpButton("i2c"), False, False, 0)
        row(g, 1, _("Codec I2C errors"), i2c_box,
            hint=_("Failed exchanges with the RT286 codec over I2C"))
        c.pack_start(g, False, False, 0)
        p.pack_start(c, False, False, 0)

        c2 = card(_("Report"))
        hint = Gtk.Label(xalign=0, label=_(
            "Everything worth pasting into a bug report, gathered in one "
            "place so you do not have to guess which sysctl matters."))
        hint.get_style_context().add_class("sst-hint")
        hint.set_line_wrap(True)
        c2.pack_start(hint, False, False, 0)

        sw = Gtk.ScrolledWindow()
        sw.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        sw.set_size_request(-1, 190)
        self.dump_view = Gtk.TextView()
        self.dump_view.set_editable(False)
        self.dump_view.set_monospace(True)
        self.dump_view.set_wrap_mode(Gtk.WrapMode.NONE)
        sw.add(self.dump_view)
        c2.pack_start(sw, True, True, 0)

        bb = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        gen = Gtk.Button(label=_("Collect"))
        gen.connect("clicked", self._collect_dump)
        bb.pack_start(gen, False, False, 0)
        cp = Gtk.Button(label=_("Copy to clipboard"))
        cp.connect("clicked", self._copy_dump)
        bb.pack_start(cp, False, False, 0)
        issues = Gtk.LinkButton.new_with_label(
            help_.REPO + "/issues/new", _("Open an issue"))
        bb.pack_end(issues, False, False, 0)
        c2.pack_start(bb, False, False, 0)
        p.pack_start(c2, True, True, 0)

        self._bind(self.debug, "debug")
        return p

    def _collect_dump(self, _btn=None):
        self.dump_view.get_buffer().set_text(sysctl.dump())
        self._say(_("Collected. Copy it into the issue."))

    def _copy_dump(self, _btn):
        buf = self.dump_view.get_buffer()
        text = buf.get_text(buf.get_start_iter(), buf.get_end_iter(), False)
        if not text.strip():
            self._collect_dump()
            buf = self.dump_view.get_buffer()
            text = buf.get_text(buf.get_start_iter(), buf.get_end_iter(), False)
        Gtk.Clipboard.get(Gdk.SELECTION_CLIPBOARD).set_text(text, -1)
        self._say(_("Copied to clipboard."))

    def _page_info(self):
        import os

        p = page()
        sc = Gtk.ScrolledWindow()
        sc.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        inner = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        sc.add(inner)
        p.pack_start(sc, True, True, 0)

        # Project image, if it was installed alongside the panel. Absence is
        # not worth an error - the tab is still useful without it.
        for cand in (os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "FreeBSD_SST_Audio.png"),
                     "/usr/local/lib/sst-panel/FreeBSD_SST_Audio.png"):
            if os.path.exists(cand):
                try:
                    from gi.repository import GdkPixbuf
                    # Deliberately small. At full width the artwork pushes
                    # everything else in the tab below the fold, which makes
                    # an information tab that shows no information.
                    pb = GdkPixbuf.Pixbuf.new_from_file_at_scale(
                        cand, 240, -1, True)
                    img = Gtk.Image.new_from_pixbuf(pb)
                    img.set_halign(Gtk.Align.CENTER)
                    frame = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
                    frame.get_style_context().add_class("sst-card")
                    frame.pack_start(img, False, False, 0)
                    inner.pack_start(frame, False, False, 0)
                except GLib.Error:
                    pass
                break

        c = card(_("About"))
        about = Gtk.Label(xalign=0, label=_(
            "Analog audio on Intel Broadwell-U laptops, where sound is routed "
            "through the SST DSP rather than standard HDA. This panel is the "
            "graphical front end to the driver's settings."))
        about.set_line_wrap(True)
        about.set_max_width_chars(60)
        c.pack_start(about, False, False, 0)

        vg = grid()
        dv = sysctl.driver_version()
        v1 = Gtk.Label(xalign=0, label=dv or _("unknown"))
        v1.get_style_context().add_class("sst-reading")
        row(vg, 0, _("Driver version"), v1,
            hint=_("Taken from the message the driver prints when it attaches"))
        v2 = Gtk.Label(xalign=0, label=_("GTK 3 / Python"))
        row(vg, 1, _("Panel"), v2)
        c.pack_start(vg, False, False, 0)
        inner.pack_start(c, False, False, 0)

        c2 = card(_("Project"))
        for label, url in (
            (_("Source code and documentation"), help_.REPO),
            (_("Report a problem"), help_.REPO + "/issues"),
            (_("Configuration reference"),
             help_.REPO + "#sysctl-configuration-reference"),
        ):
            b = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
            la = Gtk.Label(label=label, xalign=0)
            la.set_size_request(220, -1)
            b.pack_start(la, False, False, 0)
            link = Gtk.LinkButton.new_with_label(url, _("Open"))
            link.set_halign(Gtk.Align.START)
            b.pack_start(link, False, False, 0)
            c2.pack_start(b, False, False, 0)
        inner.pack_start(c2, False, False, 0)

        c3 = card(_("Licence"))
        lic = Gtk.Label(xalign=0, label=_(
            "Driver and panel: BSD 3-Clause.\n"
            "DSP firmware (IntcSST2.bin) is distributed by Intel under its "
            "own binary licence and is not covered by the above."))
        lic.set_line_wrap(True)
        lic.set_max_width_chars(60)
        c3.pack_start(lic, False, False, 0)
        liclink = Gtk.LinkButton.new_with_label(
            help_.REPO + "/blob/main/LICENSE", _("Full text"))
        liclink.set_halign(Gtk.Align.START)
        c3.pack_start(liclink, False, False, 0)
        inner.pack_start(c3, False, False, 0)

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

    def _stop_player(self):
        """
        End the sample player and reap it.

        Dropping the handle is not enough. The player is meant to exit when
        the sample runs out, but with --ao=oss it blocks on the device rather
        than exiting as soon as the DSP stops draining - exactly the state
        this panel exists to diagnose. Every play then left one more process
        holding /dev/dsp0 for good; seven were found alive after an afternoon,
        two of them over an hour and a half old.
        """
        import subprocess

        proc, self._player = self._player, None
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass

    def _play_once(self, _btn=None):
        """Play the sweep with whatever is set right now."""
        if self._player and self._player.poll() is None:
            self._stop_player()
            self.play_state.set_text("")
            self.btn_play.set_label(_("Play sweep"))
            return
        try:
            self._player = tone.play_async()
        except OSError as e:
            self._say(str(e), True)
            return
        self.btn_play.set_label(_("Stop"))
        self.play_state.set_text(_("playing…"))
        GLib.timeout_add(tone.duration_ms() + 200, self._play_finished)

    def _play_finished(self):
        self._stop_player()
        self.btn_play.set_label(_("Play sweep"))
        self.play_state.set_text("")
        return False

    def _compare_presets(self, _btn):
        """
        Play the sweep once per preset, announcing each as it starts.

        Stepped through with timeouts rather than a loop so the interface
        stays responsive and the preset name on screen matches what is coming
        out of the speakers.
        """
        if self._comparing:
            self._comparing = False
            if self._player and self._player.poll() is None:
                self._player.terminate()
            self._restore_preset()
            return

        self._comparing = True
        self._saved_preset = sysctl.get_int("eq_preset")
        self.btn_compare.set_label(_("Stop"))
        self.btn_play.set_sensitive(False)
        GLib.idle_add(self._compare_step, 0)

    def _compare_step(self, index):
        names = _preset_names()
        if not self._comparing or index >= len(names):
            self._restore_preset()
            return False

        try:
            sysctl.set_int("eq_preset", index)
        except sysctl.ReadOnly as e:
            self._say(str(e), True)
            self._restore_preset()
            return False

        self._loading = True
        self.eq_preset.set_active(index)
        self._loading = False
        self.play_state.set_text(f"{index + 1}/{len(names)}  {names[index]}")

        self._stop_player()     # the previous preset's player, if it hung
        try:
            self._player = tone.play_async()
        except OSError as e:
            self._say(str(e), True)
            self._restore_preset()
            return False

        # a breath between presets, so the ear can tell them apart
        GLib.timeout_add(tone.duration_ms() + 600, self._compare_step, index + 1)
        return False

    def _restore_preset(self):
        """Put the preset back where it was before the comparison started."""
        self._comparing = False
        self._stop_player()
        self.btn_compare.set_label(_("Compare all presets"))
        self.btn_play.set_sensitive(True)
        self.play_state.set_text("")
        if self._saved_preset is not None and self.can_write:
            try:
                sysctl.set_int("eq_preset", self._saved_preset)
                self._loading = True
                self.eq_preset.set_active(self._saved_preset)
                self._loading = False
            except sysctl.ReadOnly:
                pass
        self._saved_preset = None

    def _jack_toggled(self, _sw, state):
        # set_active() during _load() emits state-set just as a click does.
        # Without this guard the panel tries to write on startup and, when it
        # cannot, greets an unprivileged user with a permission error they did
        # nothing to cause.
        if self._loading or not self.can_write:
            return False
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
        self.recoveries.set_text(str(sysctl.get_int("dsp_recoveries")))
        return True


# Tab order, for --screenshot. Kept next to the code that builds them so the
# two cannot drift apart unnoticed.
TAB_NAMES = ("equaliser", "limiter", "ramps", "jack", "diagnostics",
             "info")


def _shoot(window, directory, lang):
    """
    Capture each tab to a PNG.

    GTK photographs its own window, which avoids depending on a screenshot
    tool being installed - there is none on the target machine. Each tab is
    given a turn through the main loop before capture, otherwise the
    screenshot catches the previous page still drawn.
    """
    import os
    from gi.repository import GdkPixbuf  # noqa: F401  (registers the type)

    os.makedirs(directory, exist_ok=True)
    shots = []

    def step(index):
        if index >= len(TAB_NAMES):
            Gtk.main_quit()
            return False
        window.notebook.set_current_page(index)
        # let the switch settle, then let one refresh tick populate the meters
        GLib.timeout_add(700, capture, index)
        return False

    def capture(index):
        gw = window.get_window()
        w, h = gw.get_width(), gw.get_height()
        pb = Gdk.pixbuf_get_from_window(gw, 0, 0, w, h)
        path = os.path.join(directory, f"{lang}-{TAB_NAMES[index]}.png")
        pb.savev(path, "png", [], [])
        shots.append(path)
        print(path)
        GLib.timeout_add(120, step, index + 1)
        return False

    GLib.timeout_add(900, step, 0)
    return shots


def main():
    import argparse

    ap = argparse.ArgumentParser(
        description="Control panel for the acpi_intel_sst driver")
    ap.add_argument("--lang", choices=sorted(i18n.LANGUAGES),
                    help="override the interface language")
    ap.add_argument("--tab", type=int, metavar="N",
                    help="open on this tab, counting from 0")
    ap.add_argument("--screenshot", metavar="DIR",
                    help="write a PNG of every tab to DIR and exit")
    args = ap.parse_args()

    Panel._current_lang = i18n.activate(args.lang)

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

    if args.tab is not None:
        w.notebook.set_current_page(args.tab)
    if args.screenshot:
        _shoot(w, args.screenshot, Panel._current_lang)

    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
