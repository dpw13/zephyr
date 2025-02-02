.. _seeed_xiao_sx1262:

Seeed XIAO Wio-SX1262 LoRa Shield
#################################

Overview
********

The Seeed XIAO Wio-SX1262 LoRa Shield is a XIAO
compatible shield based on the SX1262 LoRa transceiver
from Semtech.

More information about the shield can be found
at the `mbed SX1262MB2xAS website`_.

Pins Assignment of the Seeed XIAO Wio-SX1262 LoRa Shield
========================================================

+-----------------------+-----------------+
| Shield Connector Pin  | Function        |
+=======================+=================+
| D2                    | SX1262 RESET    |
+-----------------------+-----------------+
| D1                    | SX1262 BUSY     |
+-----------------------+-----------------+
| D0                    | SX1262 DIO1     |
+-----------------------+-----------------+
| D3                    | SX1262 SPI NSS  |
+-----------------------+-----------------+
| D4                    | SX1262 ANT SW   |
+-----------------------+-----------------+
| D10                   | SX1262 SPI MOSI |
+-----------------------+-----------------+
| D9                    | SX1262 SPI MISO |
+-----------------------+-----------------+
| D8                    | SX1262 SPI SCK  |
+-----------------------+-----------------+

The SX1262 signals DIO2 and DIO3 are not available at the shield connector.

Requirements
************

This shield can only be used with a board which provides a configuration
for XIAO connectors (see :ref:`shields` for more details).

Programming
***********

Set ``--shield seeed_xiao_sx1262`` when you invoke ``west build``. For
example:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/lorawan/class_a
   :board: xiao_esp32s3/esp32s3/procpu
   :shield: seeed_xiao_sx1262
   :goals: build

References
**********

.. target-notes::

.. _mbed SX1262MB2xAS website:
   https://os.mbed.com/components/SX126xMB2xAS/
