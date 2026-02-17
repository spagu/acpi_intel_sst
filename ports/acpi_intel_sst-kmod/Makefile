PORTNAME=	acpi_intel_sst-kmod
DISTVERSION=	0.64.2
CATEGORIES=	audio
MASTER_SITES=	GH

MAINTAINER=	spagu@example.com
COMMENT=	Intel Smart Sound Technology Audio Driver for FreeBSD
WWW=		https://github.com/spagu/acpi_intel_sst

LICENSE=	BSD3CLAUSE
LICENSE_FILE=	${WRKSRC}/LICENSE

USES=		kmod uidfix

USE_GITHUB=	yes
GH_ACCOUNT=	spagu
GH_PROJECT=	acpi_intel_sst

WRKSRC_SUBDIR=	src

MAN4=		acpi_intel_sst.4

SST_FWDIR=	/boot/firmware/intel

post-install:
	${INSTALL_MAN} ${WRKSRC}/../man/acpi_intel_sst.4 \
		${STAGEDIR}${PREFIX}/share/man/man4/
	${MKDIR} ${STAGEDIR}${SST_FWDIR}
	${INSTALL_DATA} ${WRKSRC}/../firmware/ubuntu/IntcSST2.bin \
		${STAGEDIR}${SST_FWDIR}/IntcSST2.bin

.include <bsd.port.mk>
