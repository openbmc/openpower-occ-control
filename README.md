# OpenPOWER OCC Control Service

This service will handle communications to the On-Chip Controller (OCC) on Power
processors. The OCC provides processor and memory temperatures, power readings,
power cap support, system power mode support, and idle power saver support. OCC
Control will be interfacing with the OCC to collect the temperatures and power
readings, updating the system power mode, setting power caps, and idle power
save parameters.

The service is started automatically when the BMC is started.

## Build Project

This project can be built with meson. The typical meson workflow is: meson
builddir && ninja -C builddir.

## Server

The server will start automatically after BMC is powered on.

Server status: `systemctl status org.open_power.OCC.Control.service`

To restart the service: `systemctl restart org.open_power.OCC.Control.service`

## Configuration

Service files are located in service_files subdirectory.

## References

### Power10

IBM EnergyScale for Power10 Processor-Based Systems whitepaper:
<https://www.ibm.com/downloads/cas/E7RL9N4E>

OCC Firmware Interface Spec for Power10:
<https://github.com/open-power/docs/blob/P10/occ/OCC_P10_FW_Interfaces_v1_17.pdf>

OCC Firmware: <https://github.com/open-power/occ/tree/master-p10>

### Power9

IBM EnergyScale for POWER9 Processor-Based Systems:
<https://www-01.ibm.com/common/ssi/cgi-bin/ssialias?htmlfid=49019149USEN&>

OCC Firmware Interface Spec for POWER9:
<https://github.com/open-power/docs/blob/P9/occ/OCC_P9_FW_Interfaces.pdf>
