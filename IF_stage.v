`include "mycpu.h"

module if_stage(
    input                          clk            ,
    input                          reset          ,
    //allwoin
    input                          ds_allowin     ,
    //brbus
    input  [`BR_BUS_WD       -1:0] br_bus         ,
    //to ds
    output                         fs_to_ds_valid ,
    output [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus   ,
    // inst sram interface
    output        inst_sram_en   ,
    output [ 3:0] inst_sram_wen  ,
    output [31:0] inst_sram_addr ,
    output [31:0] inst_sram_wdata,
    input  [31:0] inst_sram_rdata,
    input [`CP0_OUT_BUS_WIDTH - 1:0] cp0_to_fs_bus
);

reg         fs_valid; //IF stage的有效信号
wire        fs_ready_go;
wire        fs_allowin;
wire        to_fs_valid;
wire        ready_go;//pre-IF stage的ready信号, ADD：新增ready_go 

wire [31:0] seq_pc; //顺序执行的PC
wire [31:0] nextpc; //下一个PC

wire         br_stall; //是否暂停, ADD：新增br_stall
wire         br_taken; //是否跳转
wire [ 31:0] br_target; //跳转目标
wire         is_bd_inst; //是否是分支延迟槽中的指令`

assign {is_bd_inst,br_stall, br_taken,br_target} = eret_flush? 35'b0: br_bus; //ADD：新增br_stall

wire [31:0] fs_inst; //IF stage的指令
reg  [31:0] fs_pc; //IF stage的PC

reg start;

wire fs_ex;
wire fs_bd;
wire [4:0] fs_excode;

//CP0_in_bus
wire CP0_status_exl;
wire CP0_cause_ti;
wire eret_flush;
wire [31:0] CP0_epc;
wire [31:0] fs_inst_mux_32_zero;
wire ws_ex;

assign {ws_ex,CP0_status_exl, CP0_cause_ti, eret_flush, CP0_epc} = cp0_to_fs_bus;

assign fs_inst_mux_32_zero =fs_ex ? 32'h00000021 : fs_inst;

assign fs_ex = fs_pc[1:0] == 2'b00 ? 1'b0 : 1'b1;//地址错例外
assign fs_excode = fs_ex ? `EXCODE_ADEL : 5'h00;//0x04地址错例外
assign fs_bd = br_taken|is_bd_inst;//例外指令为分支延迟槽中的指令

assign fs_to_ds_bus = eret_flush ? 0 : {fs_ex, fs_bd, fs_excode,fs_inst_mux_32_zero ,
                       fs_pc   }; //IF stage到ID stage的总线，64位+7位 = 71位//inst置为0，在mem阶段用final来送badaddr和判断

// pre-IF stage
assign ready_go     = ~br_stall; //pre-IF stage的ready信号
assign to_fs_valid  = ~reset && ready_go; //进入IF stage的有效信号, ADD：新增ready_go
assign seq_pc       = fs_pc + 3'h4; //顺序执行的PC
assign nextpc       = eret_flush? CP0_epc :
                      br_taken ? br_target : seq_pc; //下一个PC

// IF stage
assign fs_ready_go    = 1'b1; //IF stage的ready信号
assign fs_allowin     = !fs_valid || fs_ready_go && ds_allowin;//reset结束后可以直接进入
assign fs_to_ds_valid =  fs_valid && fs_ready_go;
always @(posedge clk) begin
    if (reset) begin
        fs_valid <= 1'b0;
    end
    else if (fs_allowin) begin//允许进入，reset结束后可以直接进入
        fs_valid <= to_fs_valid; //
    end

    if (reset) begin
        fs_pc <= 32'hbfbffffc;  //trick: to make nextpc be 0xbfc00000 during reset 
    end
    else if (to_fs_valid && fs_allowin) begin //允许进入，reset结束后可以直接进入
    //to_fs_valid 因为br_stall的存在，这里不会被更新，分支延迟槽的指令锁在IF stage的fs_pc寄存器中
        fs_pc <= nextpc;
    end else if (eret_flush) begin
        fs_pc <= CP0_epc-4'h4; //
    end
end

assign inst_sram_en    = to_fs_valid && fs_allowin;//pre-IF阶段就送入了，这里也会被br_stall置于0
assign inst_sram_wen   = 4'h0;
assign inst_sram_addr  = nextpc;//pre-IF阶段送入的PC
assign inst_sram_wdata = 32'b0;

assign fs_inst         = inst_sram_rdata;//IF阶段再把指令和fs_pc送出去

endmodule
