# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Inline help.

Tooltips are good for a one-line reminder and bad for anything that needs a
paragraph. These are the paragraphs: a small "?" next to a section opens a
popover explaining what the setting is, what it is for, and what a sensible
value looks like, with a link into the project documentation for the rest.

Help text is translatable like the rest of the interface. Links point at the
published documentation rather than at a copy bundled here, so they cannot go
stale relative to the driver.
"""

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

import sst_i18n as i18n  # noqa: E402

_ = i18n._

REPO = "https://github.com/spagu/acpi_intel_sst"
DOCS = f"{REPO}/blob/main/docs"


def topics():
    """
    The help entries.

    Built lazily rather than at import so the strings pick up whichever
    language was activated, instead of whatever was current when the module
    first loaded.
    """
    return {
        "eq": (
            _("Parametric equaliser"),
            _("A parametric band boosts or cuts one region of the spectrum. "
              "Three numbers describe it: the centre frequency, how much to "
              "boost or cut there, and Q — how wide a region is affected. "
              "A low Q touches a broad range and sounds gentle; a high Q is "
              "surgical and is what you want for removing a single resonance.\n\n"
              "Gain of 0 dB disables the band whatever the other settings say."),
            f"{REPO}#dsp-audio-parameters",
        ),
        "hpf": (
            _("High-pass filter"),
            _("Removes everything below the cutoff. Laptop speakers cannot "
              "reproduce deep bass, but the amplifier still tries, which "
              "wastes headroom and makes the speaker distort earlier than it "
              "needs to.\n\n"
              "Cutting what cannot be heard anyway leaves more room for what "
              "can. Somewhere between 100 and 200 Hz suits most small "
              "speakers. Set 0 to disable."),
            f"{DOCS}/GAIN_STAGING.md",
        ),
        "limiter": (
            _("Limiter"),
            _("A limiter stops the signal exceeding a level, however loud the "
              "input gets. Below the threshold nothing happens; above it, the "
              "output stops rising.\n\n"
              "Release is how quickly it lets go once the peak has passed. "
              "Too short and loud passages pump audibly; too long and one "
              "transient ducks everything after it.\n\n"
              "Watch the meters: a limiter that engages on occasional peaks "
              "is doing its job, one that never lets go is set too low."),
            f"{DOCS}/GAIN_STAGING.md",
        ),
        "meters": (
            _("Peak meters"),
            _("Output level in decibels below full scale, so 0 dB is the "
              "loudest the hardware can represent and everything else is "
              "negative.\n\n"
              "The bright line is peak-hold: where the signal reached a "
              "moment ago, which is easier to read than a bar that moves too "
              "fast to follow. Amber starts at -12 dB and red at -3 dB.\n\n"
              "The strip below plots the last few seconds, which distinguishes "
              "occasional peaks from a signal riding at the limit."),
            f"{REPO}#dsp-telemetry",
        ),
        "ramps": (
            _("Volume ramps"),
            _("Applying a volume change instantly produces a click, because "
              "the waveform jumps. A ramp moves to the new level over a few "
              "milliseconds instead, which is inaudible.\n\n"
              "The resume ramp is separate and usually longer: coming out of "
              "sleep the codec needs a moment to settle, and a slow fade-in "
              "hides the noise it makes while doing so.\n\n"
              "Shape decides how the travel is distributed over that time."),
            f"{DOCS}/VOLUME_RAMPING.md",
        ),
        "jack": (
            _("Jack detection"),
            _("The codec reports whether something is plugged into the "
              "headphone socket, and the driver switches output accordingly.\n\n"
              "Detection here is by polling rather than interrupt, so the "
              "poll counter rises steadily whenever it is enabled — that is "
              "normal, and a counter that has stopped means detection is "
              "not running."),
            f"{REPO}#jack-detection",
        ),
        "debug": (
            _("Debug level"),
            _("How much the driver writes to the system log. 0 is errors and "
              "attach messages only, which is the right setting for normal "
              "use. 3 traces everything and will flood the log.\n\n"
              "Raise it when reproducing a problem, and put it back "
              "afterwards."),
            f"{REPO}#debug",
        ),
        "i2c": (
            _("Codec I2C errors"),
            _("Failed exchanges with the RT286 codec over the I2C bus. The "
              "codec is not on the audio path itself — it is the chip the "
              "driver configures to route and amplify — so errors here show "
              "up as settings that do not take effect rather than as silence.\n\n"
              "A handful during startup is common. A counter that keeps "
              "climbing is worth investigating."),
            f"{DOCS}/TROUBLESHOOTING.md",
        ),
    }


class HelpButton(Gtk.MenuButton):
    """A small "?" that opens an explanation."""

    def __init__(self, topic_key):
        super().__init__()
        title, body, url = topics()[topic_key]

        self.set_relief(Gtk.ReliefStyle.NONE)
        self.set_tooltip_text(_("What is this?"))
        # A literal "?" rather than a themed icon: help-about-symbolic is
        # absent from several icon themes, and GTK then substitutes whatever
        # placeholder the theme provides - a star, in one case - which reads
        # as decoration rather than as help.
        self.add(Gtk.Label(label="?"))
        self.get_style_context().add_class("sst-help")

        pop = Gtk.Popover()
        pop.set_position(Gtk.PositionType.BOTTOM)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.set_margin_top(14)
        box.set_margin_bottom(14)
        box.set_margin_start(14)
        box.set_margin_end(14)
        box.set_size_request(380, -1)

        head = Gtk.Label(label=title, xalign=0)
        head.get_style_context().add_class("sst-section")
        box.pack_start(head, False, False, 0)

        text = Gtk.Label(label=body, xalign=0)
        text.set_line_wrap(True)
        text.set_max_width_chars(52)
        text.set_justify(Gtk.Justification.LEFT)
        box.pack_start(text, False, False, 0)

        link = Gtk.LinkButton.new_with_label(url, _("Documentation on GitHub"))
        link.set_halign(Gtk.Align.START)
        box.pack_start(link, False, False, 0)

        box.show_all()
        pop.add(box)
        self.set_popover(pop)


def section(title, topic_key):
    """A section heading with its help button beside it."""
    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
    lab = Gtk.Label(label=title, xalign=0)
    lab.get_style_context().add_class("sst-section")
    box.pack_start(lab, False, False, 0)
    box.pack_start(HelpButton(topic_key), False, False, 0)
    return box
