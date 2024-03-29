`include "mycpu.h" //bug fixed: include mycpu.h here

module mycpu_top(
    input         clk,
    input         resetn,
    // inst sram interface
    output        inst_sram_en,
    output [ 3:0] inst_sram_wen,
    output [31:0] inst_sram_addr,
    output [31:0] inst_sram_wdata,
    input  [31:0] inst_sram_rdata,
    // data sram interface
    output        data_sram_en,
    output [ 3:0] data_sram_wen,
    output [31:0] data_sram_addr,
    output [31:0] data_sram_wdata,
    input  [31:0] data_sram_rdata,
    // trace debug interface
    output [31:0] debug_wb_pc,
    output [ 3:0] debug_wb_rf_wen,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

wire         ds_allowin; //允许IF stage进入ID stage
wire         es_allowin; //允许ID stage进入EXE stage
wire         ms_allowin; //允许EXE stage进入MEM stage
wire         ws_allowin; //允许MEM stage进入WB stage
wire         fs_to_ds_valid; //IF stage到ID stage的有效信号
wire         ds_to_es_valid; //ID stage到EXE stage的有效信号
wire         es_to_ms_valid; //EXE stage到MEM stage的有效信号
wire         ms_to_ws_valid; //MEM stage到WB stage的有效信号
wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus; //IF stage到ID stage的总线，64位
wire [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus; //ID stage到EXE stage的总线，136位
wire [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus; //EXE stage到MEM stage的总线，71位
wire [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus; //MEM stage到WB stage的总线，70位
wire [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus; //WB stage到RF的总线，38位
wire [`BR_BUS_WD       -1:0] br_bus;//branch相关的总线，32位
wire [`CP0_OUT_BUS_WIDTH - 1:0] CP0_out_bus; //CP0相关的总线，35位
wire [5:0] ext_int = 6'b0; //ADD：新增ext_int,假设。

// IF stage
if_stage if_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ds_allowin     (ds_allowin     ),
    //brbus
    .br_bus         (br_bus         ),
    //outputs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    // inst sram interface
    .inst_sram_en   (inst_sram_en   ),
    .inst_sram_wen  (inst_sram_wen  ),
    .inst_sram_addr (inst_sram_addr ),
    .inst_sram_wdata(inst_sram_wdata),
    .inst_sram_rdata(inst_sram_rdata),
    .cp0_to_fs_bus  (CP0_out_bus    )
);
// ID stage
id_stage id_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .es_allowin     (es_allowin     ),
    .ds_allowin     (ds_allowin     ),
    //from fs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    //to es
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to fs
    .br_bus         (br_bus         ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //bypass input
    .es_to_ms_bus   (es_to_ms_bus   ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    .cp0_to_ds_bus  (CP0_out_bus    )
);
// EXE stage
exe_stage exe_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ms_allowin     (ms_allowin     ),
    .es_allowin     (es_allowin     ),
    //from ds
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to ms
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    // data sram interface
    .data_sram_en   (data_sram_en   ),
    .data_sram_wen  (data_sram_wen  ),
    .data_sram_addr (data_sram_addr ),
    .data_sram_wdata(data_sram_wdata),
    .cp0_to_es_bus  (CP0_out_bus    ),
    .ms_to_ws_bus   (ms_to_ws_bus   )
);
// MEM stage
mem_stage mem_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    .ms_allowin     (ms_allowin     ),
    //from es
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ws
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //from data-sram
    .data_sram_rdata(data_sram_rdata),
    .cp0_to_ms_bus  (CP0_out_bus    )
);
// WB stage
wb_stage wb_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    .ext_int        (ext_int        ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    //from ms
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //trace debug interface
    .debug_wb_pc      (debug_wb_pc      ),
    .debug_wb_rf_wen  (debug_wb_rf_wen  ),
    .debug_wb_rf_wnum (debug_wb_rf_wnum ),
    .debug_wb_rf_wdata(debug_wb_rf_wdata),
    .CP0_out_bus      (CP0_out_bus      )
);

endmodule
