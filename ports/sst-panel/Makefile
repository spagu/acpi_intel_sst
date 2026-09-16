PORTNAME=	sst-panel
DISTVERSIONPREFIX=	v
DISTVERSION=	0.69.3
CATEGORIES=	audio

MAINTAINER=	please@forget.me.uk
COMMENT=	Graphical control panel for the Intel SST audio driver
WWW=		https://github.com/spagu/acpi_intel_sst

LICENSE=	BSD3CLAUSE
LICENSE_FILE=	${WRKSRC}/../LICENSE

# The panel talks to the driver over its sysctl tree and is useless without
# it. It still starts and says so plainly, but there is no reason to install
# one without the other.
RUN_DEPENDS=	${KMODDIR}/acpi_intel_sst.ko:audio/acpi_intel_sst-kmod

USES=		desktop-file-utils gettext-tools gnome python:run shebangfix
USE_GITHUB=	yes
GH_ACCOUNT=	spagu
GH_PROJECT=	acpi_intel_sst
# v0.68.0 is the first tag whose tarball contains gui/.
USE_GNOME=	gtk30 pygobject3
SHEBANG_FILES=	sst_panel.py

NO_ARCH=	yes

# Everything lives in gui/; the top-level Makefile builds the kernel module,
# which this port has no business doing.
WRKSRC_SUBDIR=	gui

# Byte-compiles the sources and checks every catalogue. Needs no hardware and
# no kernel, so it is safe wherever the port is built.
TEST_TARGET=	check

.include <bsd.port.mk>
