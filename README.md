# FPGA_LAB2
<p>
AXI 陳冠亦 負責
Core algorithm -> 已經有open source 下載"soft macro " 不能是hard macro 。 譬如說下載了一個y*2^256 mod N mod1. 
top.sv  -> module mod1(clk, output); endmodule.
不要連axi 一起implement 找slice最小且能用的 注意測資testbence.sv要跑過

uart -> axi 格式 ->陳冠亦->soft macro指定的格式->你們優化

兩邊都好之後再一起impplement。


</p>
