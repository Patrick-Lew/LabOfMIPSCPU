`include "mycpu.h"

module wb_stage(
    input                           clk           ,
    input                           reset         ,
    input  [4:0]                    ext_int       ,
    //allowin
    output                          ws_allowin    ,
    //from ms
    input                           ms_to_ws_valid,
    input  [`MS_TO_WS_BUS_WD -1:0]  ms_to_ws_bus  ,
    //to rf: for write back
    output [`WS_TO_RF_BUS_WD -1:0]  ws_to_rf_bus  ,
    //trace debug interface
    output [31:0] debug_wb_pc     ,
    output [ 3:0] debug_wb_rf_wen ,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata
);

reg         ws_valid;
wire        ws_ready_go;

reg [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus_r;
wire        ws_gr_we;
wire [ 4:0] ws_dest;
wire [31:0] ws_final_result;
wire [31:0] ws_pc;
wire [3:0] ws_byte_we;
assign {ws_byte_we    ,  //73:70
        ws_gr_we       ,  //69:69
        ws_dest        ,  //68:64
        ws_final_result,  //63:32
        ws_pc             //31:0
       } = ms_to_ws_bus_r;

wire        rf_we;
wire [4 :0] rf_waddr;
wire [31:0] rf_wdata;
assign ws_to_rf_bus = ws_valid ? {rf_we   ,  //37:37
                       rf_waddr,  //36:32
                       rf_wdata   //31:0
                      }:0;

assign ws_ready_go = 1'b1;
assign ws_allowin  = !ws_valid || ws_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ws_valid <= 1'b0;
    end
    else if (ws_allowin) begin
        ws_valid <= ms_to_ws_valid;
    end

    if (ms_to_ws_valid && ws_allowin) begin
        ms_to_ws_bus_r <= ms_to_ws_bus;
    end
end

assign rf_we    = ws_gr_we&&ws_valid;
assign rf_waddr = ws_dest;
assign rf_wdata = ws_final_result;

// debug info generate
assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_wen   = {4{rf_we}};
assign debug_wb_rf_wnum  = ws_dest;
assign debug_wb_rf_wdata = ws_final_result;

//------------------系统控制寄存器部分------------------
//在wb阶段实现更新与读取
reg [31:0] CP0_status;
reg [31:0] CP0_cause;
reg [31:0] CP0_epc;

wire wb_ex;//是否异常,顺着流水线传递 流水信号
wire wb_bd;//是否延迟槽指令 顺着流水线传递 流水信号
wire wb_excode;//异常码 顺着流水线传递 流水信号
wire eret_flush;

wire mtc0_we;
wire CP0_status_exl; //status.exl 表示是否在异常处理中 , output signal

wire CP0_cause_ti; //cause.ti,output signal
wire wb_inst_is_mtc0;
wire count_eq_compare;
wire [`WIDTH_OF_CP0_ADDR -1: 0] CP0_addr;

assign mtc0_we = ws_valid && wb_inst_is_mtc0 && !wb_ex;
assign CP0_status_exl = CP0_status[1];
assign CP0_cause_ti   = CP0_cause[30];

always @(posedge clk) begin
    //status寄存器
    if (reset) begin
        CP0_status[31:23] <= 9'b0;
        CP0_status[22]    <= 1'b1; //status.bev=1
        CP0_status[21:16] <= 6'b0;
        CP0_status[7:2]   <= 6'b0;
        CP0_status[1:0]   <= 2'b0;//EXL=0[1:1],IE=0[0:0]
    end else if(wb_ex) begin
        CP0_status[1]     <= 1'b1;//EXL=1
    end else if(eret_flush) begin
        CP0_status[1]     <= 1'b0;//EXL=0
    end else if (mtc0_we && CP0_addr == `CR_STATUS) begin
        CP0_status[1]  <= ws_final_result[1];      //status.exl
        CP0_status[15:8] <= ws_final_result[7:0];   //status.im[7:0]
        CP0_status[0] <= ws_final_result[0];       //status.ie
    end
    //————————————————————————————————————————————————————————————————————————————
    //cause寄存器
    if (reset) begin
                CP0_cause[31:0]   <= 32'b0;
        //epc复位值无
    end else if(mtc0_we && !CP0_status_exl) begin
        CP0_cause[31] <=wb_bd; //cause.bd
    end

    if (mtc0_we && CP0_addr==`CR_COMPARE) begin
        CP0_cause[30] <= 1'b0; //cause.ti 写compare寄存器时置0
    end else if (count_eq_compare) begin
        CP0_cause[30] <= 1'b1; //cause.ti 计数器等于compare寄存器时置1,优先是写compare时置0
    end

    if (!reset) begin //不复位
        CP0_cause[15] <= ext_int[5] | CP0_cause_ti; //cause.ip[7] = ext_int[5] | cause.ti
        CP0_cause[14:10] <= ext_int[4:0]; //cause.ip[6:2] = ext_int[4:0]
    end

    if (!reset&&mtc0_we&&CP0_addr==`CR_CAUSE) begin
        CP0_cause[9:8] <= ws_final_result[9:8]; //cause.ip[1:0],R/W
    end

    if (wb_ex) begin
        CP0_cause[6:2] <= wb_excode; //cause.exc_code,exception code
    end

    //————————————————————————————————————————————————————————————————————————————
    //epc寄存器
    if (wb_ex && !CP0_status_exl) begin
        CP0_epc <= wb_bd ? ws_pc - 3'h4 : ws_pc; //branch delay slot?
    end else if(mtc0_we && CP0_addr==`CR_EPC) begin
        CP0_epc <= ws_final_result;
    end

    
        

end

endmodule
