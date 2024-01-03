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
    input  [31:0] inst_sram_rdata
);

reg         fs_valid; //IF stage的有效信号
wire        fs_ready_go;
wire        fs_allowin;
wire        to_fs_valid;

wire [31:0] seq_pc; //顺序执行的PC
wire [31:0] nextpc; //下一个PC

wire         br_taken; //是否跳转
wire [ 31:0] br_target; //跳转目标
assign {br_taken,br_target} = br_bus;

wire [31:0] fs_inst; //IF stage的指令
reg  [31:0] fs_pc; //IF stage的PC
assign fs_to_ds_bus = {fs_inst ,
                       fs_pc   }; //IF stage到ID stage的总线，64位

// pre-IF stage
assign to_fs_valid  = ~reset; //进入IF stage的有效信号
assign seq_pc       = fs_pc + 3'h4; //顺序执行的PC
assign nextpc       = br_taken ? br_target : seq_pc; //下一个PC

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
        fs_pc <= nextpc;
    end
end

assign inst_sram_en    = to_fs_valid && fs_allowin;//pre-IF阶段就送入了
assign inst_sram_wen   = 4'h0;
assign inst_sram_addr  = nextpc;//pre-IF阶段送入的PC
assign inst_sram_wdata = 32'b0;

assign fs_inst         = inst_sram_rdata;//IF阶段再把指令和fs_pc送出去

endmodule
