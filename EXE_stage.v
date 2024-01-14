`include "mycpu.h"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    // data sram interface
    output        data_sram_en   ,
    output [ 3:0] data_sram_wen  ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata
);




reg         es_valid      ;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
wire [31:0] es_alu_op     ;
wire        es_load_op    ;
wire        es_src1_is_sa ;  
wire        es_src1_is_pc ;
wire        es_src2_is_imm; 
wire        es_src2_is_zero_extended_imm; //Add
wire        es_src2_is_8  ;
wire        es_gr_we      ;
wire        es_mem_we     ;
wire [ 4:0] es_dest       ;
wire [15:0] es_imm        ;
wire [31:0] es_rs_value   ;
wire [31:0] es_rt_value   ;
wire [31:0] es_pc         ;
assign {es_alu_op      ,  //156:125 
        es_load_op     ,  //123:123 +1
        es_src1_is_sa  ,  //122:122 +1
        es_src1_is_pc  ,  //121:121 +1
        es_src2_is_imm ,  //120:120 +1
        es_src2_is_zero_extended_imm,//120:120
        es_src2_is_8   ,  //119:119
        es_gr_we       ,  //118:118
        es_mem_we      ,  //117:117
        es_dest        ,  //116:112
        es_imm         ,  //111:96
        es_rs_value    ,  //95 :64
        es_rt_value    ,  //63 :32
        es_pc             //31 :0
       } = ds_to_es_bus_r;

wire [31:0] es_alu_src1   ;
wire [31:0] es_alu_src2   ;
wire [31:0] es_alu_result ;
wire [15:0] es_extend_bus;

wire        es_res_from_mem;

assign es_extend_bus[3:0] = es_load_op ? es_alu_op[23:20] : 16'h0000;//lb lbu lh lhu

assign es_res_from_mem = es_load_op;
assign es_to_ms_bus = es_to_ms_valid ? {es_extend_bus,//85:71
                       es_res_from_mem,  //70:70
                       es_gr_we       ,  //69:69
                       es_dest        ,  //68:64
                       es_alu_result  ,  //63:32
                       es_pc             //31:0
                      } : 0;

wire divider_ready; //ADD: divider的ready信号
assign es_ready_go    = ds_to_es_bus_r==0 ? 1 : divider_ready;
assign es_allowin     = !es_valid || es_ready_go && ms_allowin;
assign es_to_ms_valid =  es_valid && es_ready_go;
always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;

    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end

    if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end

assign es_alu_src1 = es_src1_is_sa  ? {27'b0, es_imm[10:6]} : 
                     es_src1_is_pc  ? es_pc[31:0] :
                     es_inst_is_mfhi ? HI :
                     es_inst_is_mflo ? LO :
                                      es_rs_value;
assign es_alu_src2 = es_src2_is_imm ? {{16{es_imm[15]}}, es_imm[15:0]} : 
                     es_src2_is_8   ? 32'd8 :
                     es_src2_is_zero_extended_imm ? {{16{1'b0}}, es_imm[15:0]} : //添加0拓展
                     es_rt_value;

alu u_alu(
    .alu_op     (es_alu_op    ),
    .alu_src1   (es_alu_src1  ), //bug fixed2: es_alu_src1
    .alu_src2   (es_alu_src2  ),
    .alu_result (es_alu_result)
    );

//------------------------乘除法部件------------------------
reg [31:0] HI;
reg [31:0] LO;
wire [63:0] unsigned_mult_result;
wire [63:0] signed_mult_result;
wire [31:0] hi_result;
wire [31:0] lo_result;
wire es_inst_is_mult = es_alu_op[12];
wire es_inst_is_multu  = es_alu_op[13];
wire es_inst_is_div = es_alu_op[14];
wire es_inst_is_divu = es_alu_op[15];
assign unsigned_mult_result = es_alu_src1 * es_alu_src2;
assign signed_mult_result   = $signed(es_alu_src1) * $signed(es_alu_src2);

reg s_axis_divisor_tvalid;
reg s_axis_dividend_tvalid;
wire s_axis_divisor_tready_signed;
wire s_axis_divisor_tready_unsigned;
//wire s_axis_divisor_tready;
wire s_axis_dividend_tready_signed;
wire s_axis_dividend_tready_unsigned;
//wire s_axis_dividend_tready;
wire [31:0] s_axis_divisor_tdata;
wire [31:0] s_axis_dividend_tdata;
//assign s_axis_divisor_tready = s_axis_divisor_tready_signed || s_axis_divisor_tready_unsigned;
//assign s_axis_dividend_tready = s_axis_dividend_tready_signed || s_axis_dividend_tready_unsigned;

assign s_axis_dividend_tdata = es_inst_is_div|es_inst_is_divu?  es_alu_src1 :32'h00000000 ;
assign s_axis_divisor_tdata = es_inst_is_div|es_inst_is_divu ?  es_alu_src2 :32'h00000000 ;

wire m_axis_dout_tvalid_signed;
wire m_axis_dout_tvalid_unsigned;
wire [63:0] m_axis_dout_tdata_signed;
wire [63:0] m_axis_dout_tdata_unsigned;
wire [63:0] m_axis_dout_tdata;
assign m_axis_dout_tdata = es_inst_is_div ? m_axis_dout_tdata_signed :
                           es_inst_is_divu ? m_axis_dout_tdata_unsigned :
                           64'b0;

assign {hi_result, lo_result} = es_inst_is_mult ? signed_mult_result :
                                 es_inst_is_multu ? unsigned_mult_result :
                                 es_inst_is_div&&m_axis_dout_tvalid_signed ? {m_axis_dout_tdata[31:0],m_axis_dout_tdata[63:32]} :
                                 es_inst_is_divu&&m_axis_dout_tvalid_unsigned ?  {m_axis_dout_tdata[31:0],m_axis_dout_tdata[63:32]} :
                                 64'b0;

assign divider_ready = (es_inst_is_div && !m_axis_dout_tvalid_signed)||(es_inst_is_divu && !m_axis_dout_tvalid_unsigned) ? 1'b0 :
                       1'b1;

always @(posedge clk ) begin
    if (reset) begin
        s_axis_divisor_tvalid <= 1'b0;
        s_axis_dividend_tvalid <= 1'b0;
    end else if (ds_to_es_valid && es_allowin) begin
        s_axis_divisor_tvalid <= ds_to_es_bus[125+14]|| ds_to_es_bus[125+15];
        s_axis_dividend_tvalid <= ds_to_es_bus[125+14]|| ds_to_es_bus[125+15];


        if (es_inst_is_mthi ) begin
            HI <= es_rs_value;
        end else if (es_inst_is_mtlo ) begin
            LO <= es_rs_value;
        end

    end

    if ( (s_axis_dividend_tready_signed&s_axis_divisor_tready_signed&es_inst_is_div) ||  (s_axis_divisor_tready_unsigned&s_axis_dividend_tready_unsigned&es_inst_is_divu) ) begin
        s_axis_divisor_tvalid <= 1'b0;
        s_axis_dividend_tvalid <= 1'b0;
    end

    if (es_inst_is_mult || es_inst_is_multu) begin
        HI <= hi_result;
        LO <= lo_result;
    end else if ((m_axis_dout_tvalid_signed & es_inst_is_div)|| (m_axis_dout_tvalid_unsigned & es_inst_is_divu )) begin
        HI <= hi_result;
        LO <= lo_result;
    end 
        
    
end

div_gen_0 signed_divider (
  .aclk(clk),                                      // input wire aclk
  .s_axis_divisor_tvalid(s_axis_divisor_tvalid),    // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tready(s_axis_divisor_tready_signed),    // output wire s_axis_divisor_tready
  .s_axis_divisor_tdata(s_axis_divisor_tdata),      // input wire [31 : 0] s_axis_divisor_tdata

  .s_axis_dividend_tvalid(s_axis_dividend_tvalid),  // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tready(s_axis_dividend_tready_signed),  // output wire s_axis_dividend_tready
  .s_axis_dividend_tdata(s_axis_dividend_tdata),    // input wire [31 : 0] s_axis_dividend_tdata

  .m_axis_dout_tvalid(m_axis_dout_tvalid_signed),          // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata(m_axis_dout_tdata_signed)            // output wire [63 : 0] m_axis_dout_tdata
);

div_gen_1_unsigned unsigned_divider (
  .aclk(clk),                                      // input wire aclk
  .s_axis_divisor_tvalid(s_axis_divisor_tvalid),    // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tready(s_axis_divisor_tready_unsigned),    // output wire s_axis_divisor_tready
  .s_axis_divisor_tdata(s_axis_divisor_tdata),      // input wire [31 : 0] s_axis_divisor_tdata

  .s_axis_dividend_tvalid(s_axis_dividend_tvalid),  // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tready(s_axis_dividend_tready_unsigned),  // output wire s_axis_dividend_tready
  .s_axis_dividend_tdata(s_axis_dividend_tdata),    // input wire [31 : 0] s_axis_dividend_tdata

  .m_axis_dout_tvalid(m_axis_dout_tvalid_unsigned),          // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata(m_axis_dout_tdata_unsigned)            // output wire [63 : 0] m_axis_dout_tdata
);

//------------------------HI/LO存取-----------------------
wire es_inst_is_mfhi = es_alu_op[16];
wire es_inst_is_mflo = es_alu_op[17];
wire es_inst_is_mthi = es_alu_op[18];
wire es_inst_is_mtlo = es_alu_op[19];

// assign es_alu_result = es_inst_is_mfhi ? HI :
//                        es_inst_is_mflo ? LO :
//                        es_alu_result;



// data sram interface，EXE发送，MEM执行/接收
assign data_sram_en    = 1'b1;
assign data_sram_wen   = es_mem_we&&es_valid ? 4'hf : 4'h0;
assign data_sram_addr  = es_alu_result;
assign data_sram_wdata = es_rt_value;

endmodule
