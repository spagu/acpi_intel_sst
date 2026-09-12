# Build and install the SST control panel.
#
#   make mo       compile the translation catalogues
#   make check    byte-compile the sources and verify the catalogues
#   make install  install into /usr/local
#   make run      run from the checkout, without installing

PREFIX?=	/usr/local
LIBDIR=		$(PREFIX)/lib/sst-panel
LANGS=		pl de fr zh
PY=		sst_panel.py sst_sysctl.py sst_i18n.py sst_meters.py sst_curves.py \
		sst_help.py

mo:
	@for l in $(LANGS); do \
		mkdir -p locale/$$l/LC_MESSAGES; \
		msgfmt -o locale/$$l/LC_MESSAGES/sst-panel.mo po/$$l.po && \
		echo "  $$l ok"; \
	done

check: mo
	@python3 -m py_compile $(PY) && echo "  python ok"
	@for l in $(LANGS); do \
		msgfmt --check --statistics -o /dev/null po/$$l.po; \
	done

install: mo
	install -d $(DESTDIR)$(LIBDIR)
	install -m 644 $(PY) $(DESTDIR)$(LIBDIR)/
	-install -m 644 FreeBSD_SST_Audio.png $(DESTDIR)$(LIBDIR)/
	install -d $(DESTDIR)$(PREFIX)/share/applications
	install -m 644 sst-panel.desktop $(DESTDIR)$(PREFIX)/share/applications/
	@for l in $(LANGS); do \
		install -d $(DESTDIR)$(LIBDIR)/locale/$$l/LC_MESSAGES; \
		install -m 644 locale/$$l/LC_MESSAGES/sst-panel.mo \
			$(DESTDIR)$(LIBDIR)/locale/$$l/LC_MESSAGES/; \
	done
	install -d $(DESTDIR)$(PREFIX)/bin
	printf '#!/bin/sh\nexec python3 -c "import sys; sys.path.insert(0,\\"$(LIBDIR)\\"); import sst_panel; raise SystemExit(sst_panel.main())" "$$@"\n' \
		> $(DESTDIR)$(PREFIX)/bin/sst-panel
	chmod 755 $(DESTDIR)$(PREFIX)/bin/sst-panel

run: mo
	@python3 sst_panel.py

clean:
	rm -rf locale __pycache__

.PHONY: mo check install run clean
