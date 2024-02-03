`ifndef MYCPU_H
    `define MYCPU_H

    `define BR_BUS_WD       35 // fixed bug5: 32->33->34->36
    `define FS_TO_DS_BUS_WD 71 // 64->71 add exception
    `define DS_TO_ES_BUS_WD 188 //136->137->141->157->165->172->188 add src2 is zero-extended imm
    `define ES_TO_MS_BUS_WD 138 //71->87->119->130 add extend_bus, add es_rt_value
    `define MS_TO_WS_BUS_WD 93 //70->74->85->93 add byte_we
    `define WS_TO_RF_BUS_WD 38
    `define WIDTH_OF_CP0_ADDR 5
    `define CR_BADVADDR 5'b01000 //CP0的8号寄存器是badvaddr寄存器
    `define CR_COUNT  5'b01001 //CP0的9号寄存器是count寄存器
    `define CR_COMPARE 5'b01011 //CP0的11号寄存器是compare寄存器
    `define CR_STATUS 5'b01100 //CP0的12号寄存器是status寄存器
    `define CR_CAUSE  5'b01101 //CP0的13号寄存器是cause寄存器
    `define CR_EPC   5'b01110 //CP0的14号寄存器是epc寄存器
    `define CP0_OUT_BUS_WIDTH 36 //status_exl,cause_ti,eret_flush,epc

    `define EXCODE_INT  5'h00 //中断
    `define EXCODE_ADEL 5'h04 //地址异常
    `define EXCODE_ADES 5'h05 //写地址异常
    `define EXCODE_SYS  5'h08 //系统调用异常
    `define EXCODE_BP   5'h09 //断点异常
    `define EXCODE_RI   5'h0a //保留指令异常
    `define EXCODE_OVF  5'h0c //整形溢出异常

`endif
