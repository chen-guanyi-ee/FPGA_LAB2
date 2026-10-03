# FPGA_LAB2

2026/10/03 陳冠亦

目前完成 AXI UART Lite wrapper 初版：

- UART 接收 N、d、y，共 96 bytes
- 使用 LFSR 計算 RX/TX byte 數量
- 計算完成後傳回 31-byte plaintext
- 提供 UART/AXI Lite wrapper testbench

RSA256 core 計算部分仍待整合。
