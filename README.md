# SunMon

A Home Assistant integration that reads the power and voltage of a SolarCount
solar power meter over RS485.

A SolarCount is a "vintage" multi-channel electronic power meter built to
monitor the generated power of solar panels. It was sold in Germany by
SolarConsult around 2009. SunMon talks to it with its own proprietary protocol,
not SunSpec, through a USB to RS485 adapter (for example one with a CH343G
chip).

Integration domain: `sunmon`. Needs Home Assistant 2026.9 or newer.

## What you get

For each channel (up to 6):

- power in W and voltage in V
- the yield of the previous hour, day, month and year, as the raw numbers the
  SolarCount reports

For all channels together:

- the sum of the power and of the yields
- **Total power generated** in kWh, integrated from the power at every read
  cycle; usable in the Energy dashboard

For troubleshooting: RS485 counters, throughput and bus load, and a
"Values held" indicator.

## Install with HACS

1. In HACS, open the menu at the top right and choose **Custom repositories**.
2. Add `https://github.com/birdygryphon/SunMon` with the type **Integration**.
3. Search for **SunMon** in HACS and download it.
4. Restart Home Assistant.
5. Go to **Settings > Devices & services > Add integration** and search for
   **SunMon**.

## Install manually

1. Copy the folder `custom_components/sunmon` into the `custom_components`
   folder of your Home Assistant configuration directory (next to
   `configuration.yaml`).
2. Restart Home Assistant and add **SunMon** under
   **Settings > Devices & services > Add integration**.

## Wiring

Connect the adapter's A and B terminals to the SolarCount's RS485 A and B. If
the setup dialog reports that the SolarCount does not answer, the usual cause
is A and B being swapped.

## Protocol

The UART protocol, with frame layouts, examples and flow diagrams, is
documented in [docs/PROTOCOL.md](docs/PROTOCOL.md).
