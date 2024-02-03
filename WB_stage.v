`include "mycpu.h"

module wb_stage(
    input                           clk           ,
    input                           reset         ,
    input  [5:0]                    ext_int       ,
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
    output [31:0] debug_wb_rf_wdata,
    output [`CP0_OUT_BUS_WIDTH - 1:0] CP0_out_bus
);

reg         ws_valid;
wire        ws_ready_go;

wire wb_inst_is_mtf0;//是否是mtc0指令,CP0

reg [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus_r;
wire        ws_gr_we;
wire [ 4:0] ws_dest;
wire [31:0] ws_final_result;
wire [31:0] ws_pc;
wire [3:0] ws_byte_we;
wire [18:0] ws_CP0_bus;
assign {ws_CP0_bus,
        ws_byte_we    ,  //73:70
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

assign rf_we    = ws_gr_we&&ws_valid&& ~wb_final_ex;
assign rf_waddr = ws_dest;
assign rf_wdata = wb_inst_is_mtf0 ? mfc0_data :  ws_final_result;

// debug info generate
assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_wen   = {4{rf_we}};
assign debug_wb_rf_wnum  = ws_dest;
assign debug_wb_rf_wdata = wb_inst_is_mtf0? mfc0_data : ws_final_result;

//------------------系统控制寄存器部分------------------
//在wb阶段实现更新与读取
reg [31:0] CP0_status;
reg [31:0] CP0_cause;
reg [31:0] CP0_epc;
reg [31:0] CP0_count;
reg [31:0] CP0_compare;
reg [31:0] CP0_badvaddr;
reg tick;
// wire wb_inst_is_mtf0;
wire wb_inst_is_syscall;

wire wb_final_ex;//最终是否异常，用作真正的异常判断
wire wb_ex;//是否异常,顺着流水线传递 流水信号
wire wb_bd;//是否延迟槽指令 顺着流水线传递 流水信号
wire [4:0]wb_final_excode;//最终是否异常，用作真正的异常判断
wire [4:0]wb_excode;//异常码 顺着流水线传递 流水信号
wire wb_inst_is_eret;
wire eret_flush;

wire mtc0_we;
wire CP0_status_exl; //status.exl 表示是否在异常处理中 , output signal
wire CP0_status_ie; //status.ie 表示是否开中断, output signal
wire CP0_cause_ti; //cause.ti,output signal
wire wb_inst_is_mtc0;
wire count_eq_compare;
wire [`WIDTH_OF_CP0_ADDR -1: 0] CP0_addr;
wire [4:0] wb_rd;
wire [2:0] wb_sel;
wire flush_and_jump;
wire [31:0] eret_or_jump_target;
wire int_ex_response;
wire [7:0] int_filter;

assign int_filter = CP0_cause[15:8] & CP0_status[15:8] ;
assign int_ex_response = CP0_status_ie && int_filter && ~CP0_status_exl;
assign wb_final_ex = wb_ex ? wb_ex : int_ex_response ;//优先级：写compare >计数器等于compare寄存器
assign wb_final_excode = wb_ex ? wb_excode : 
                         int_ex_response ? `EXCODE_INT : wb_excode;
                        
assign eret_flush = ws_valid ? wb_inst_is_eret && !wb_final_ex :1'b0 ;
assign {wb_inst_is_mtc0,wb_inst_is_mtf0,wb_inst_is_eret,wb_inst_is_syscall,wb_rd,wb_sel,
            wb_ex,wb_bd,wb_excode} = ws_CP0_bus; //少了addr和sel，总线里补充一下
assign mtc0_we = ws_valid && wb_inst_is_mtc0 && !wb_final_ex;
assign CP0_status_exl = CP0_status[1];
assign CP0_cause_ti   = CP0_cause[30];
assign CP0_addr = wb_rd;
assign CP0_status_ie = CP0_status[0];
assign count_eq_compare = CP0_count == CP0_compare;

assign flush_and_jump =  ws_valid ? wb_inst_is_syscall || eret_flush || wb_final_ex&(~CP0_status_exl):1'b0;
assign eret_or_jump_target = wb_inst_is_syscall || wb_final_ex&(~CP0_status_exl) ? 32'hbfc00380 : CP0_epc;



wire [31:0] mfc0_data;
assign mfc0_data = CP0_addr == `CR_STATUS ? CP0_status :
                    CP0_addr == `CR_CAUSE  ? CP0_cause  :
                    CP0_addr == `CR_EPC    ? CP0_epc    :
                    CP0_addr == `CR_BADVADDR? CP0_badvaddr:
                    CP0_addr == `CR_COUNT  ? CP0_count  :
                    CP0_addr == `CR_COMPARE? CP0_compare:
                    32'h0;

assign CP0_out_bus = {wb_final_ex,CP0_status_exl , CP0_cause_ti , flush_and_jump , eret_or_jump_target};//1+1+1+32=35

always @(posedge clk) begin
    //status寄存器
    if (reset) begin
        CP0_status[31:23] <= 9'b0;
        CP0_status[22]    <= 1'b1; //status.bev=1
        CP0_status[21:16] <= 6'b0;
        CP0_status[7:2]   <= 6'b0;
        CP0_status[1:0]   <= 2'b0;//EXL=0[1:1],IE=0[0:0]
    end else if(wb_final_ex) begin
        
        CP0_status[1]     <= 1'b1;//EXL=1
    end else if(eret_flush) begin
        CP0_status[1]     <= 1'b0;//EXL=0
        //这里需要清空流水线,以及重置PC
    end else if (mtc0_we && CP0_addr == `CR_STATUS) begin
        CP0_status[1]  <= ws_final_result[1];      //status.exl
        CP0_status[15:8] <= ws_final_result[15:8];   //status.im[7:0]
        CP0_status[0] <= ws_final_result[0];       //status.ie
    end
    //————————————————————————————————————————————————————————————————————————————
    //cause寄存器
    if (reset) begin
        CP0_cause[31:0]   <= 32'b0;
        //epc复位值无
    end else if(wb_final_ex && !CP0_status_exl) begin
        CP0_cause[31] <=wb_bd; //cause.bd
    end

    if (mtc0_we && CP0_addr==`CR_COMPARE) begin
        CP0_cause[30] <= 1'b0; //cause.ti 写compare寄存器时置0
        CP0_compare <= ws_final_result;
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

    if (wb_final_ex) begin
        CP0_cause[6:2] <= wb_final_excode; //cause.exc_code,exception code
    end

    //————————————————————————————————————————————————————————————————————————————
    //epc寄存器
    if (wb_final_ex && !CP0_status_exl) begin
             CP0_epc <= wb_bd ? ws_pc - 3'h4 : ws_pc; //branch delay slot?
    end else if(mtc0_we && CP0_addr==`CR_EPC ) begin
        CP0_epc <= ws_final_result; 
    end

    //————————————————————————————————————————————————————————————————————————————
    //badVaddr寄存器
    if (wb_final_ex && !CP0_status_exl && wb_final_excode==`EXCODE_ADEL) begin
        if (ws_gr_we && ws_pc!=32'b0) begin //读数据错误，在ms阶段把错误地址写入final_result
            CP0_badvaddr <= ws_final_result;
        end
    end else if (wb_final_ex && !CP0_status_exl && wb_final_excode == `EXCODE_ADES) begin
            CP0_badvaddr <= ws_final_result;//在ms阶段把错误地址写入final_result,复用数据通路
    end
    //————————————————————————————————————————————————————————————————————————————
    //count寄存器
    if (reset) begin
        tick <= 1'b0;
    end else begin
        tick <= ~tick;
    end

    if (mtc0_we && CP0_addr==`CR_COUNT) begin
        CP0_count <= ws_final_result;
    end else if (tick) begin
        CP0_count <= CP0_count + 1'b1;
    end

end

endmodule
