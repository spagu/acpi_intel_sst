# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Translations for the SST panel.

The interface follows the user's own locale and falls back to English, which
is also the language the source strings are written in - so an untranslated
string is still readable rather than a msgid like "eq.preset.label".

Compiled catalogues are looked for next to the code first, then in the usual
system location, so the panel works both from a checkout and installed.
"""

import gettext
import locale
import os

DOMAIN = "sst-panel"

# Languages with a catalogue. The value is what appears in the selector, in
# that language - nobody looks for "Chinese" written in English.
LANGUAGES = {
    "en": "English",
    "pl": "Polski",
    "de": "Deutsch",
    "fr": "Français",
    "zh": "中文",
}

_translation = None


def _localedirs():
    here = os.path.dirname(os.path.abspath(__file__))
    return [
        os.path.join(here, "locale"),
        "/usr/local/lib/sst-panel/locale",
        "/usr/local/share/locale",
    ]


def system_language():
    """
    Which language did the user ask for?

    getlocale() is preferred over the deprecated getdefaultlocale(), with the
    environment consulted directly as a fallback: on a bare X session LANG is
    often the only thing set.
    """
    try:
        code = locale.getlocale()[0]
    except (TypeError, ValueError):
        code = None
    if not code:
        code = os.environ.get("LC_ALL") or os.environ.get("LANG") or "en"
    short = code.split("_")[0].split(".")[0].lower()
    return short if short in LANGUAGES else "en"


def activate(lang=None):
    """
    Install a translation and return the language actually used.

    Falls back silently: a missing catalogue leaves the English source strings
    in place, which is a perfectly good outcome and not worth an error dialog.
    """
    global _translation
    lang = lang or system_language()

    for d in _localedirs():
        if not os.path.isdir(d):
            continue
        try:
            _translation = gettext.translation(
                DOMAIN, localedir=d, languages=[lang], fallback=False)
            return lang
        except FileNotFoundError:
            continue

    _translation = gettext.NullTranslations()
    return "en"


def _(text):
    """Translate one string."""
    if _translation is None:
        activate()
    return _translation.gettext(text)
