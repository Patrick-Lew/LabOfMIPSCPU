`ifndef MYCPU_H
    `define MYCPU_H

    `define BR_BUS_WD       34 // fixed bug5: 32->33->34
    `define FS_TO_DS_BUS_WD 64
    `define DS_TO_ES_BUS_WD 157 //136->137->141->157 add src2 is zero-extended imm
    `define ES_TO_MS_BUS_WD 119 //71->87->119 add extend_bus, add es_rt_value
    `define MS_TO_WS_BUS_WD 74 //70->74 add byte_we
    `define WS_TO_RF_BUS_WD 38
    `define WIDTH_OF_CP0_ADDR 5
    `define CR_COMPARE 5'b01011 //CP0的11号寄存器是compare寄存器
    `define CR_STATUS 5'b01100 //CP0的12号寄存器是status寄存器
    `define CR_CAUSE  5'b01101 //CP0的13号寄存器是cause寄存器
    `define CR_EPC   5'b01110 //CP0的14号寄存器是epc寄存器
`endif
