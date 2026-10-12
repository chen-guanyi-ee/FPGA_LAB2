## ------------------------------------------------------------------
## vga_extra.xdc  --  ADDITIONAL constraints for the VGA visualiser.
## Nexys-A7-100T-Master.xdc is NOT modified; add this as a 2nd
## constraint file.  Pin locations from the Digilent Nexys A7 master XDC.
## ------------------------------------------------------------------

## VGA connector
set_property -dict { PACKAGE_PIN A3  IOSTANDARD LVCMOS33 } [get_ports {VGA_R[0]}]
set_property -dict { PACKAGE_PIN B4  IOSTANDARD LVCMOS33 } [get_ports {VGA_R[1]}]
set_property -dict { PACKAGE_PIN C5  IOSTANDARD LVCMOS33 } [get_ports {VGA_R[2]}]
set_property -dict { PACKAGE_PIN A4  IOSTANDARD LVCMOS33 } [get_ports {VGA_R[3]}]
set_property -dict { PACKAGE_PIN C6  IOSTANDARD LVCMOS33 } [get_ports {VGA_G[0]}]
set_property -dict { PACKAGE_PIN A5  IOSTANDARD LVCMOS33 } [get_ports {VGA_G[1]}]
set_property -dict { PACKAGE_PIN B6  IOSTANDARD LVCMOS33 } [get_ports {VGA_G[2]}]
set_property -dict { PACKAGE_PIN A6  IOSTANDARD LVCMOS33 } [get_ports {VGA_G[3]}]
set_property -dict { PACKAGE_PIN B7  IOSTANDARD LVCMOS33 } [get_ports {VGA_B[0]}]
set_property -dict { PACKAGE_PIN C7  IOSTANDARD LVCMOS33 } [get_ports {VGA_B[1]}]
set_property -dict { PACKAGE_PIN D7  IOSTANDARD LVCMOS33 } [get_ports {VGA_B[2]}]
set_property -dict { PACKAGE_PIN D8  IOSTANDARD LVCMOS33 } [get_ports {VGA_B[3]}]
set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports {VGA_HS}]
set_property -dict { PACKAGE_PIN B12 IOSTANDARD LVCMOS33 } [get_ports {VGA_VS}]

## Slow-motion switches (SW0 = core /256, SW1 = core /65536)
set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports {SW[0]}]
set_property -dict { PACKAGE_PIN L16 IOSTANDARD LVCMOS33 } [get_ports {SW[1]}]
set_false_path -from [get_ports {SW[*]}]
