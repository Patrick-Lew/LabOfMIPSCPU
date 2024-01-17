`include "mycpu.h"

module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //from data-sram
    input  [31                 :0] data_sram_rdata
);

reg         ms_valid;
wire        ms_ready_go;
wire [15:0] ms_extend_bus;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
wire [31:0] ms_rt_value;
assign {ms_rt_value    ,  //118:87
        ms_extend_bus  ,  //86:71
        ms_res_from_mem,  //70:70
        ms_gr_we       ,  //69:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

wire [31:0] mem_result;
wire [31:0] ms_final_result;
wire [3:0] ms_byte_we;
wire inst_lb;
wire inst_lbu;
wire inst_lh;
wire inst_lhu;
wire inst_lwl;
wire inst_lwr;
wire [1:0] offset;
assign offset = ms_alu_result[1:0];
assign {inst_lhu,inst_lh,inst_lbu,inst_lb} = ms_res_from_mem ? ms_extend_bus[3:0]
                                                                : 4'b0; //lb lbu lh lhu用extend_bus的低4位表示
assign {inst_lwr, inst_lwl} = ms_res_from_mem ? ms_extend_bus[5:4]
                                              : 2'b0; //lwr lwl用extend_bus的第5、6位表示
assign ms_to_ws_bus = ms_to_ws_valid ? { ms_byte_we,//73:70
                       ms_gr_we       ,  //69:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      } : 0;

assign ms_ready_go    = 1'b1;
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin;
assign ms_to_ws_valid = ms_valid && ms_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ms_valid <= 1'b0;
    end
    else if (ms_allowin) begin
        ms_valid <= es_to_ms_valid;
    end

    if (es_to_ms_valid && ms_allowin) begin
        es_to_ms_bus_r  = es_to_ms_bus;
    end
end
//lb对offset对应的那个byte进行符号扩展

wire [7:0] mem_extend_result_byte;
wire [15:0] mem_extend_result_half;
wire [31:0] mem_signed_extend_result_byte;
wire [31:0] mem_unsigned_extend_result_byte;
wire [31:0] mem_signed_extend_result_half;
wire [31:0] mem_unsigned_extend_result_half;

assign mem_extend_result_byte = offset==2'b00 ? data_sram_rdata[7:0]:
                                offset==2'b01 ? data_sram_rdata[15:8]:
                                offset==2'b10 ? data_sram_rdata[23:16]:
                                                data_sram_rdata[31:24];
assign mem_extend_result_half = offset==2'b00 ? data_sram_rdata[15:0]:
                                offset==2'b01 ? data_sram_rdata[23:8]:
                                offset==2'b10 ? data_sram_rdata[31:16]:
                                                data_sram_rdata[15:0];
                                                
assign mem_signed_extend_result_byte = { {24{mem_extend_result_byte[7]}}, mem_extend_result_byte[7:0]};
assign mem_unsigned_extend_result_byte = { {24{1'b0}}, mem_extend_result_byte[7:0]};

assign mem_signed_extend_result_half = { {16{mem_extend_result_half[15]}}, mem_extend_result_half[15:0]};
assign mem_unsigned_extend_result_half = { {16{1'b0}}, mem_extend_result_half[15:0]};

assign mem_result = data_sram_rdata;

wire [31:0] ms_byte_result;


assign ms_final_result = ms_res_from_mem&inst_lb ? mem_signed_extend_result_byte:
                         ms_res_from_mem&inst_lbu ? mem_unsigned_extend_result_byte:
                         ms_res_from_mem&inst_lh ? mem_signed_extend_result_half:
                         ms_res_from_mem&inst_lhu ? mem_unsigned_extend_result_half:
                         ms_res_from_mem&(inst_lwl|inst_lwr) ? ms_byte_result:
                         ms_res_from_mem? mem_result
                                         : ms_alu_result;



// assign ms_byte_we = offset==2'b00&inst_lwl? 4'b1000:
//                     offset==2'b01&inst_lwl? 4'b1100:
//                     offset==2'b10&inst_lwl? 4'b1110:
//                     offset==2'b11&inst_lwl? 4'b1111:
//                     offset==2'b00&inst_lwr? 4'b1111:
//                     offset==2'b01&inst_lwr? 4'b0111:
//                     offset==2'b10&inst_lwr? 4'b0011:
//                     offset==2'b11&inst_lwr? 4'b0001:
//                                             4'b0000;

assign ms_byte_result [31:0] = offset==2'b00&inst_lwl? {data_sram_rdata[7:0],ms_rt_value[23:0]}:
                               offset==2'b01&inst_lwl? {data_sram_rdata[15:0],ms_rt_value[15:0]}:
                               offset==2'b10&inst_lwl? {data_sram_rdata[23:0],ms_rt_value[7:0]}:
                               offset==2'b11&inst_lwl? {data_sram_rdata[31:0]}:
                               offset==2'b00&inst_lwr? {data_sram_rdata[31:0]}:
                               offset==2'b01&inst_lwr? {ms_rt_value[31:24],data_sram_rdata[31:8]}:
                               offset==2'b10&inst_lwr? {ms_rt_value[31:16],data_sram_rdata[31:16]}:
                               offset==2'b11&inst_lwr? {ms_rt_value[31:8],data_sram_rdata[31:24]}:
                                                        32'b0;
                             
                                                         

endmodule
