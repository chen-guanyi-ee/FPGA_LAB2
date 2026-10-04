# FPGA_LAB2

## 2026/10/03 陳冠亦

目前完成 AXI UART Lite wrapper 初版：

- UART 接收 N、d、y，共 96 bytes
- 使用 LFSR 計算 RX/TX byte 數量
- 計算完成後傳回 31-byte plaintext
- 提供 UART/AXI Lite wrapper testbench

RSA256 core 計算部分仍待整合。

## 2026/10/04 陳冠亦

- RSA256 計算已整合至 `Rsa256Wrapper.sv`
- 使用 MSB-first square-and-multiply 與單一共用 8-bit adder
- 直接重用 N、d、y 接收 buffer，輸出 31-byte plaintext
- UART 到 RSA 再到 UART 的完整模擬已通過
- Nexys A7-100T routed utilization：544 slices、1604 LUT、1700 FF
- 100 MHz timing passed，WNS 為 +1.137 ns

## 2026/10/04 陳冠亦

- 降低stage數量
- 不使用axi b response
- 改為m*m first
- slice = 389

- ## 2026/10/05 陳冠亦

- 使用sram
- slice=94

