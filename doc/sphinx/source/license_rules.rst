*******************
no-OS license rules
*******************

no-OS is, by default, permissively licensed under **BSD-3-Clause**.
Individual files may be under different terms, always stated in the file's
own header. This page explains how licenses are declared and organized so
that the license of any file is clear and, where possible, machine
readable.

For a human-readable map of which directory trees use which license, see
the top-level ``LICENSE`` file. For a short pointer to the default and to
this directory, see the top-level ``COPYING`` file.

Declaring a license in a source file
====================================

Every source file declares its license with an SPDX identifier in its
header comment, in a form appropriate to the file type. For C sources and
headers::

    /* SPDX-License-Identifier: BSD-3-Clause */

For scripts and other ``#``-comment files::

    # SPDX-License-Identifier: BSD-3-Clause

The SPDX tag is authoritative for that file, and is required for new
no-OS code because it is precise and machine parsable. Analog Devices'
own no-OS sources carry this tag (usually alongside a copyright line)
rather than the full BSD boilerplate.

Full-text license headers remain only in bundled third-party code under
``libraries/``, in the vendor-supplied device-API subtrees that cannot be
relicensed, and in a small number of third-party-derived drivers under a
different license (for example the BSD-2-Clause ``drivers/adc/ltc2312/``);
there the per-file header is the authoritative statement of the file's
license.

Analog Devices' own glue and porting files that live inside a
``libraries/`` directory (for example ``libraries/mqtt/mqtt_client.c`` or
``libraries/fatfs/adi_diskio.c``) are not third-party code: they carry
the SPDX ``BSD-3-Clause`` tag like the rest of no-OS. Only the upstream
library sources they sit next to keep their own full-text headers.

The ``LICENSES/`` directory
===========================

The full text of every license used by Analog Devices' own no-OS code is
collected under ``LICENSES/``, one file per license, grouped by role:

``LICENSES/preferred/``
  ``BSD-3-Clause`` (the default and the only license for new no-OS code).

``LICENSES/permissive/``
  Permissive licenses that are allowed but are not the default:
  ``Apache-2.0``, ``BSD-2-Clause`` and ``MIT`` (the last used by some
  bundled libraries). Note that ``Apache-2.0`` is not compatible with
  ``GPL-2.0-only``.

``LICENSES/copyleft/``
  Copyleft licenses present in the tree: ``GPL-2.0`` (and its
  ``GPL-2.0+`` "or later" variant). Not for new general-purpose code. It
  governs the ``adrv9025*``/``adrv904x*`` no-OS wrappers in the madura and
  koror trees, and the AD9081 vendor device API under
  ``drivers/adc/ad9081/api`` (the same code the Linux kernel ships under
  GPL-2.0). The JESD204 framework (``jesd204/`` and ``include/jesd204.h``)
  is ``GPL-2.0+``, matching the Linux kernel framework it is ported from;
  unlike the optional vendor code above it is a core subsystem, so any
  project that uses JESD204 links GPL-2.0+ code.

``LICENSES/proprietary/``
  Vendor device API licenses: ``LicenseRef-ADI-API-License`` (the ADI
  "Source Code Software License Agreement" / CTSLA) that governs the
  RF-transceiver device API code, and the closely related ADI "Software
  License Agreement" carried by a few converter device-API files (for
  example ``drivers/adc/ad9083/ad9083_api/``,
  ``drivers/adc/ad9208/ad9208_api/`` and
  ``drivers/dac/ad917x/ad917x_api/``). A further variant,
  ``LicenseRef-ADI-ADRV9025-SLA`` (the ADI "Software License Agreement"
  ``20180813-ADI-N3PT-CTSLA``), governs the ADRV9025 vendor API files
  under ``drivers/rf-transceiver/madura``.
  The Talise device API source (``drivers/rf-transceiver/talise/api``)
  is the ``20130524-CISC-CTSLA`` revision, tagged
  ``LicenseRef-ADI-API-License``, whose full text ships as
  ``talise/api/LICENSE.txt``; the accompanying ARM and stream firmware
  binaries (``drivers/rf-transceiver/talise/firmware/*.h``) are the
  separate ``20180301-AD937X-CTSLA`` firmware revision, tagged
  ``LicenseRef-ADI-AD937X-Firmware-CTSLA``, whose full text is embedded
  at the top of those generated headers (the same file the Linux kernel
  ships as ``firmware/TALISE_FIRMWARE_SOFTWARE_LICENSE_AGREEMENT.txt``);
  the ``adrv9009*`` no-OS wrappers in that tree are BSD-3-Clause.
  The Navassa device API source (``drivers/rf-transceiver/navassa``) is
  the ADRV9001 SDK ``20190814-APIGUIHDLFWNAV-CTSLA`` revision, tagged
  ``LicenseRef-ADI-ADRV9001-SLA``, whose full text ships as the SDK's
  ``pkg/LICENSE.md`` (the ``adrv9002*`` no-OS wrappers are BSD-3-Clause
  and the bundled ``third_party/jsmn`` parser is MIT).
  All the vendor API licenses are non-permissive and confined to their
  vendor driver subtrees. (The koror tree, including its
  ``platforms`` integration files, is Apache-2.0 as of the ADRV904X
  2.15.0 API release; earlier snapshots carried a proprietary ADI
  "Software License Agreement" instead.)

Each license file begins with a ``Valid-License-Identifier:`` line (the
SPDX or repository-local ``LicenseRef-`` identifier), an optional
``SPDX-URL:`` for standard licenses, a ``Usage-Guide:`` describing when
the license may be used, and the full ``License-Text:``.

Repository-local ``LicenseRef-`` identifiers
============================================

Some licenses used by no-OS are not standard SPDX licenses, so they use
REUSE-style repository-local identifiers:

- ``LicenseRef-ADI-API-License`` — the ADI CTSLA vendor API license.
- ``LicenseRef-ADI-ADRV9025-SLA`` — the ADI "Software License Agreement"
  ``20180813-ADI-N3PT-CTSLA`` carried by the ADRV9025 (madura) vendor
  API files; the same non-permissive ADI "API license" family. It
  expressly forbids subjecting the software to GPL/LGPL terms.
- ``LicenseRef-ADI-AD937X-Firmware-CTSLA`` — the ADI "Software License
  Agreement" ``20180301-AD937X-CTSLA`` carried by the Talise (AD937x)
  ARM and stream firmware binaries under
  ``drivers/rf-transceiver/talise/firmware``; the same non-permissive
  ADI "API license" family. It covers firmware object code and
  expressly forbids subjecting the software to any "Excluded License"
  (source-disclosure / copyleft terms).
- ``LicenseRef-ADI-ADRV9001-SLA`` — the ADI "Software License
  Agreement" ``20190814-APIGUIHDLFWNAV-CTSLA`` carried by the ADRV9001
  SDK (navassa) vendor API files under
  ``drivers/rf-transceiver/navassa``; the same non-permissive ADI
  "API license" family. Its full text ships as the SDK's
  ``pkg/LICENSE.md``, and it is incompatible with ``GPL-2.0``.

They are **non-permissive** and must not be used for new files.

Acceptable licenses for new contributions
=========================================

New contributions must use the permissive ``BSD-3-Clause`` license. See
the "Licensing of contributions" section of :doc:`contributing` for the
full rule, including the single exception for vendor-supplied device APIs
that cannot be relicensed.

Bundled third-party libraries
=============================

Libraries bundled under ``libraries/`` are third-party code. Each carries
its own ``LICENSE`` / ``COPYING`` file in its directory, which governs
that library; they are not re-described under ``LICENSES/``.
