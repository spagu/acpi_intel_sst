PORTNAME=	acpi_intel_sst-kmod
DISTVERSIONPREFIX=	v
DISTVERSION=	0.64.3
CATEGORIES=	audio
MASTER_SITES=	GH

MAINTAINER=	please@forget.me.uk
COMMENT=	Intel Smart Sound Technology Audio Driver for FreeBSD
WWW=		https://github.com/spagu/acpi_intel_sst

LICENSE=	BSD3CLAUSE INTEL_FW
LICENSE_COMB=	multi
LICENSE_NAME_INTEL_FW=	Intel Binary Firmware Release License
LICENSE_FILE_BSD3CLAUSE=	${WRKSRC}/../LICENSE
LICENSE_FILE_INTEL_FW=	${WRKSRC}/../firmware/ubuntu/LICENSE.IntcSST2
LICENSE_PERMS_INTEL_FW=	dist-mirror no-dist-sell pkg-mirror no-pkg-sell auto-accept

ONLY_FOR_ARCHS=	amd64
ONLY_FOR_ARCHS_REASON=	Intel SST DSP is only available on x86_64 platforms

USES=		kmod uidfix

USE_GITHUB=	yes
GH_ACCOUNT=	spagu
GH_PROJECT=	acpi_intel_sst

WRKSRC_SUBDIR=	src

SST_FWDIR=	/boot/firmware/intel

post-install:
	${INSTALL_MAN} ${WRKSRC}/../man/acpi_intel_sst.4 \
		${STAGEDIR}${PREFIX}/share/man/man4/
	${MKDIR} ${STAGEDIR}${SST_FWDIR}
	${INSTALL_DATA} ${WRKSRC}/../firmware/ubuntu/IntcSST2.bin \
		${STAGEDIR}${SST_FWDIR}/IntcSST2.bin
	${INSTALL_DATA} ${WRKSRC}/../firmware/ubuntu/LICENSE.IntcSST2 \
		${STAGEDIR}${SST_FWDIR}/LICENSE.IntcSST2

.include <bsd.port.mk>
