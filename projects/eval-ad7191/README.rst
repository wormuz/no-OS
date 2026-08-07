EVAL-AD7191 no-OS Example Project
=================================

.. no-os-doxygen::

.. contents:: Table of Contents
    :depth: 3

Supported Evaluation Boards
---------------------------

* `EVAL-AD7191EBZ <https://www.analog.com/EVAL-AD7191>`_

Overview
--------

The AD7191 is a pin-programmable, ultra-low noise 24-bit sigma-delta ADC for
bridge sensor measurements. It contains a programmable gain amplifier with gains
of 1, 8, 64 and 128, two differential analog input pairs, an internal
temperature sensor and a bridge power-down switch.

The part has no register map. Gain, output data rate, input pair, temperature
sensor selection and clock source are all strapped on dedicated pins, and the
serial interface only shifts conversion results out. That interface is two-wire:
SCLK plus a single DOUT/RDY line that is both the data output and the data-ready
indication. Output data rates of 10 Hz, 50 Hz, 60 Hz and 120 Hz are available;
the 50 Hz and 60 Hz settings place filter notches on the mains frequency.

The EVAL-AD7191EBZ carries the part along with an ADR421 2.5 V reference, a
crystal for the optional external clock, and a Cypress FX2 USB controller for
use with the vendor evaluation software. It is not a Raspberry Pi HAT and has no
40-pin header, so driving it from a Pi means hand-wiring to the external
controller header.

Applications
------------

* Weigh scales
* Strain gauges and pressure measurement
* Temperature measurement
* Bridge transducer interfaces

Hardware Specifications
-----------------------

Power Supply Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~

The board is normally powered from the USB connection to the FX2 controller.
When it is driven from an external controller instead, supply AVDD and DVDD
externally and tie the board ground to the controller ground; a floating ground
between the two boards is the most common cause of nonsense readings.

On-board Connectors
~~~~~~~~~~~~~~~~~~~

============= ===================================================
Connector     Function
============= ===================================================
J1            USB connection to the on-board FX2 controller
J2            External controller header: serial interface and
              configuration pins
J3            Reference select: ADR421 2.5 V output or AVDD
S6            DIP switch strapping the configuration pins, with
              100 kOhm pull-ups
============= ===================================================

Two details of this board differ from the data sheet and will cost you time if
missed. First, the schematic uses its own net names: ``FILTSEL2``/``FILTSEL1``
are ODR2/ODR1, ``GAINSEL2``/``GAINSEL1`` are PGA2/PGA1, ``CHSEL`` is CHAN,
``TEMPSEL`` is TEMP, and - confusingly - the net labelled ``CS`` on pin 4 is
PDOWN, not a chip select. Second, the configuration pins are pulled up through
S6, so before an external controller can drive them the corresponding switches
must be opened.

The reference defaults to the ADR421 at 2.5 V. If J3 is moved to take the
reference from AVDD instead, change ``AD7191_VREF_MV`` in
``src/common/common_data.h`` to match.

No-OS Supported Examples
------------------------

This project is organized around the no-OS variant based build flow. Selecting a
variant at build time (``--variant <name>``) chooses which application is
compiled. The platform ``main()`` is a thin dispatcher that calls
``example_main()``, provided by the selected example. Shared initialization data
is defined in
`src/common <https://github.com/analogdevicesinc/no-OS/tree/main/projects/eval-ad7191/src/common>`__,
and platform-specific macros and extra init parameters are in
`src/platform <https://github.com/analogdevicesinc/no-OS/tree/main/projects/eval-ad7191/src/platform>`__.

Basic Example
~~~~~~~~~~~~~

Reads both differential input pairs and the internal temperature sensor in a
loop, printing the raw code alongside the converted microvolts or millidegrees
Celsius. Because switching the input is a configuration change, each iteration
pays the filter settling time of four conversion periods per switch.

The gain, output data rate, input pair and clock source the device starts in are
set in ``ad7191_ip`` in ``src/common/common_data.c``.

IIO Example
~~~~~~~~~~~

Runs an IIOD server exposing the AD7191 as an IIO device named ``ad7191`` with
three channels: two differential voltage channels and one temperature channel.
On the linux platform iio_app serves IIOD over TCP on port 30431 rather than
over the UART, so no serial connection is involved.

Only one input can be converted at a time, so a buffered read must have exactly
one channel enabled. Enabling more is rejected rather than returning samples
skewed by the settling time each switch would cost.

If you are not familiar with ADI IIO Application, please take a look at:
:dokuwiki:`IIO No-OS </resources/tools-software/no-os-software/iio>`

If you are not familiar with ADI IIO-Oscilloscope Client, please take a look at:
:dokuwiki:`IIO Oscilloscope </resources/tools-software/linux-software/iio_oscilloscope>`

No-OS Supported Platforms
-------------------------

Linux
~~~~~

Used Hardware
^^^^^^^^^^^^^

* `Raspberry Pi 5 <https://www.raspberrypi.com/products/raspberry-pi-5/>`_
* `EVAL-AD7191EBZ <https://www.analog.com/EVAL-AD7191>`_

The linux platform is native: the project builds into an ordinary user-space
executable that runs on the Pi itself, talking to the part through ``spidev``
and the GPIO character device. There is nothing to flash and no debug probe.

Connections
^^^^^^^^^^^

Wire the eval board's J2 header to the Pi's 40-pin header as below. GPIO numbers
are BCM numbers; the header pin numbers are given for convenience.

================ ============ ============ ==========================================
AD7191 signal    Pi BCM GPIO  Pi header pin Notes
================ ============ ============ ==========================================
SCLK             GPIO11       23           SPI0 SCLK
DOUT/RDY         GPIO9        21           SPI0 MISO
DOUT/RDY         GPIO25       22           Same wire, second tap for ready sensing
PGA1             GPIO5        29
PGA2             GPIO6        31
ODR1             GPIO13       33
ODR2             GPIO19       35
CHAN             GPIO26       37
TEMP             GPIO16       36           Overrides CHAN while asserted
CLKSEL           GPIO20       38           Driven high for the internal oscillator
PDOWN            GPIO21       40           Labelled CS on the eval board
GND              GND          e.g. 39      Required: common ground with the eval board
================ ============ ============ ==========================================

DOUT/RDY is deliberately connected to two Pi pins. It is both the SPI data
output and the data-ready flag, so while spidev owns GPIO9 for the transfer the
second tap on GPIO25 lets the GPIO character device watch the ready indication.
Without it the driver falls back to timing out a conversion period, which works
but wastes the tail of each period; to run that way, set ``.gpio_rdy = NULL`` in
``ad7191_ip``.

There is no chip select. SPI0's CE0 is left unconnected, and PDOWN must not be
used in its place: it resets the modulator and digital filter, so driving it per
transfer would restart settling before every sample.

Before building, enable the SPI controller by adding ``dtparam=spi=on`` to
``/boot/firmware/config.txt`` and rebooting, then confirm that
``/dev/spidev0.0`` exists.

Then find the gpiochip that owns the 40-pin header:

.. code-block:: bash

   gpiodetect

On a Raspberry Pi 5 the header is on the RP1 southbridge, not on gpiochip0. Pick
the chip described as ``pinctrl-rp1`` - commonly ``gpiochip4``, though the
numbering varies with the kernel version - and set ``GPIO_CHIP_NUM`` in
``src/platform/linux/parameters.h`` to its index.

The basic example prints to the UART configured in ``parameters.h``, which
defaults to ``/dev/ttyAMA0``. Using the Pi's primary UART requires
``enable_uart=1`` in ``config.txt`` and the serial console released from it. To
print to the terminal the program was started from instead, point the console at
the controlling terminal:

.. code-block:: bash

   # in src/platform/linux/parameters.c
   .device_id = "tty",

Build Command
^^^^^^^^^^^^^

The linux platform uses the CMake/Ninja build system via the ``no_os_build.py``
helper script. Available variants: ``basic``, ``iio``. Available boards:
``rpi5``.

The build is native, so it must run on the Pi itself; no cross-toolchain
environment variable is needed. Building on a workstation is possible by passing
a cross compiler explicitly, for example
``-DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc``.

.. code-block:: bash

   cd no-OS

   # build the basic example
   python tools/scripts/no_os_build.py build \
      --project eval-ad7191 --variant basic --board rpi5

   # build the IIO example
   python tools/scripts/no_os_build.py build \
      --project eval-ad7191 --variant iio --board rpi5

The linux platform is native, so the build produces a plain executable rather
than an image to flash. Accessing ``/dev/spidev*`` and ``/dev/gpiochip*``
requires membership of the ``spi`` and ``gpio`` groups, which the default ``pi``
user already has. Run it directly:

.. code-block:: bash

   ./build/eval-ad7191-basic-rpi5/build/eval-ad7191

With the IIO variant, connect from another machine using the Pi's address:

.. code-block:: bash

   iio_attr -u ip:<pi-address>:30431 -d
   iio_readdev -u ip:<pi-address>:30431 ad7191 -b 256
