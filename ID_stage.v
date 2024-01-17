`include "mycpu.h"

module id_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from fs
    input                          fs_to_ds_valid,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to fs
    output [`BR_BUS_WD       -1:0] br_bus        ,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus  ,
    //bypass input
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    input  [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus
);

reg         ds_valid   ;
wire        ds_ready_go;

wire [31                 :0] fs_pc;
reg  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;
assign fs_pc = fs_to_ds_bus[31:0];

wire [31:0] ds_inst;
wire [31:0] ds_pc  ;
assign {ds_inst,
        ds_pc  } = fs_to_ds_bus_r;

wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

assign {rf_we   ,  //37:37
        rf_waddr,  //36:32
        rf_wdata   //31:0
       } = ws_to_rf_bus;

wire        br_stall; //ADD：新增br_stall
wire        br_taken;
wire [31:0] br_target;

wire [31:0] alu_op; //ADD：拓宽到32位
wire        load_op;
wire        src1_is_sa;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_is_zero_extended_imm;//第二个操作数是低16位0拓展至32位
wire        src2_is_8;
wire        res_from_mem;
wire        gr_we;
wire        mem_we;
wire [ 4:0] dest;
wire [15:0] imm;
wire [31:0] rs_value;
wire [31:0] rt_value;

wire [ 5:0] op;
wire [ 4:0] rs;
wire [ 4:0] rt;
wire [ 4:0] rd;
wire [ 4:0] sa;
wire [ 5:0] func;
wire [25:0] jidx;
wire [63:0] op_d;
wire [31:0] rs_d;
wire [31:0] rt_d;
wire [31:0] rd_d;
wire [31:0] sa_d;
wire [63:0] func_d;

wire        inst_addu;
wire        inst_subu;
wire        inst_slt;
wire        inst_sltu;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_nor;
wire        inst_sll;
wire        inst_srl;
wire        inst_sra;
wire        inst_addiu;
wire        inst_lui;
wire        inst_lw;
wire        inst_sw;
wire        inst_beq;
wire        inst_bne;
wire        inst_jal;
wire        inst_jr;

wire        dst_is_r31;  
wire        dst_is_rt;   

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;

wire        rs_eq_rt;
wire        rs_ge_rt;
wire        rs_gt_rt;

//bypass 
wire es_res_from_mem;
wire es_gr_we;
wire [ 4:0] es_dest;
wire [31:0] es_alu_result;

assign es_res_from_mem = es_to_ms_bus[70];
assign es_gr_we        = es_to_ms_bus[69];
assign es_dest         = es_to_ms_bus[68:64];
assign es_alu_result   = es_to_ms_bus[63:32];

wire ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_final_result;


assign ms_gr_we        = ms_to_ws_bus[69];
assign ms_dest         = ms_to_ws_bus[68:64];
assign ms_final_result = ms_to_ws_bus[63:32];

// wire        rf_we   ;
// wire [ 4:0] rf_waddr;
// wire [31:0] rf_wdata;


wire is_i_instr;
wire is_r_instr;
wire is_j_instr;//is_j_instr单独处理
wire is_mem_instr;

assign is_i_instr = op[5:3] == 3'b001;
assign is_r_instr = op[5:0] == 6'b000000;
assign is_mem_instr = op[5:4] == 2'b10;


assign br_bus       = {br_stall,br_taken,br_target};


assign ds_to_es_bus = {alu_op      ,  //156:125
                       load_op     ,  //124:124
                       src1_is_sa  ,  //123:123
                       src1_is_pc  ,  //122:122
                       src2_is_imm ,  //121:121
                       src2_is_zero_extended_imm,//120:120
                       src2_is_8   ,  //119:119
                       gr_we       ,  //118:118
                       mem_we      ,  //117:117
                       dest        ,  //116:112
                       imm         ,  //111:96
                       rs_value    ,  //95 :64
                       rt_value    ,  //63 :32
                       ds_pc          //31 :0
                      };

assign ds_ready_go    = fs_to_ds_bus_r==0 ? 1 : !(
                         (es_res_from_mem && is_r_instr|inst_lwl|inst_lwr && (es_dest==rs || es_dest == rt))
                        || (es_res_from_mem && is_i_instr && es_dest==rs)
                        || (es_res_from_mem && is_mem_instr && es_dest==rs)
                        ); //这里只有load指令会暂停，因为load指令需要等待访存结果，其他指令都不会暂停

assign ds_allowin     = !ds_valid || ds_ready_go && es_allowin; //用assign连线保证所有的逐渐互锁在一个周期内完成
assign ds_to_es_valid = ds_valid && ds_ready_go;
always @(posedge clk) begin
    if (reset) begin //bug fixed3: reset后，ds_valid置0
        ds_valid <= 1'b0;
        fs_to_ds_bus_r <= 0;
    end else if (ds_allowin) begin
        ds_valid <= fs_to_ds_valid;
    end

    if (fs_to_ds_valid && ds_allowin) begin//IF送来的数据有效，且ID允许进入
        fs_to_ds_bus_r <= fs_to_ds_bus; //IF stage到ID stage的总线，64位
    end
end

assign op   = ds_inst[31:26];
assign rs   = ds_inst[25:21];
assign rt   = ds_inst[20:16];
assign rd   = ds_inst[15:11];
assign sa   = ds_inst[10: 6];
assign func = ds_inst[ 5: 0];
assign imm  = ds_inst[15: 0];
assign jidx = ds_inst[25: 0];

decoder_6_64 u_dec0(.in(op  ), .out(op_d  ));
decoder_6_64 u_dec1(.in(func), .out(func_d));
decoder_5_32 u_dec2(.in(rs  ), .out(rs_d  ));
decoder_5_32 u_dec3(.in(rt  ), .out(rt_d  ));
decoder_5_32 u_dec4(.in(rd  ), .out(rd_d  ));
decoder_5_32 u_dec5(.in(sa  ), .out(sa_d  ));

//------------------添加新的运算类指令------------------
wire inst_add;
wire inst_addi;
wire inst_sub;

wire inst_slti;
wire inst_sltiu;

wire inst_andi;
wire inst_ori;
wire inst_xori;

wire inst_sllv;
wire inst_srlv;
wire inst_srav;

wire inst_mult;
wire inst_multu;
wire inst_div;
wire inst_divu;

wire inst_mfhi;
wire inst_mflo;
wire inst_mthi;
wire inst_mtlo;



assign inst_add    = op_d[6'h00] & func_d[6'h20] & sa_d[5'h00];
assign inst_addi   = op_d[6'h08];
assign inst_sub    = op_d[6'h00] & func_d[6'h22] & sa_d[5'h00];//这三条暂不考虑溢出

assign inst_slti   = op_d[6'h0a]; 
assign inst_sltiu  = op_d[6'h0b];

assign inst_andi   = op_d[6'h0c];
assign inst_ori    = op_d[6'h0d];
assign inst_xori   = op_d[6'h0e];

assign inst_sllv   = op_d[6'h00] & func_d[6'h04] & sa_d[5'h00];
assign inst_srlv   = op_d[6'h00] & func_d[6'h06] & sa_d[5'h00];
assign inst_srav   = op_d[6'h00] & func_d[6'h07] & sa_d[5'h00];

assign inst_mult   = op_d[6'h00] & func_d[6'h18] & sa_d[5'h00] & rd_d[5'h00];
assign inst_multu  = op_d[6'h00] & func_d[6'h19] & sa_d[5'h00] & rd_d[5'h00];
assign inst_div    = op_d[6'h00] & func_d[6'h1a] & sa_d[5'h00] & rd_d[5'h00];
assign inst_divu   = op_d[6'h00] & func_d[6'h1b] & sa_d[5'h00] & rd_d[5'h00];

assign inst_mfhi   = op_d[6'h00] & func_d[6'h10] & rt_d[5'h00] & rs_d[5'h00] & sa_d[5'h00];
assign inst_mflo   = op_d[6'h00] & func_d[6'h12] & rt_d[5'h00] & rs_d[5'h00] & sa_d[5'h00];
assign inst_mthi   = op_d[6'h00] & func_d[6'h11] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_mtlo   = op_d[6'h00] & func_d[6'h13] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];

//------------------添加新的运算类指令------------------

//------------------添加新的转移和访存类指令------------------
wire inst_bgez;
wire inst_bgtz;
wire inst_blez;
wire inst_bltz;

wire inst_j;

wire inst_bltzal;
wire inst_bgezal;

wire inst_jalr;

wire inst_lb;
wire inst_lbu;
wire inst_lh;
wire inst_lhu;

wire inst_sb;
wire inst_sh;

wire inst_lwl;
wire inst_lwr;
wire inst_swl;
wire inst_swr;


assign inst_bgez   = op_d[6'h01] & rt_d[5'h01];
assign inst_bgtz   = op_d[6'h07] & rt_d[5'h00];
assign inst_blez   = op_d[6'h06] & rt_d[5'h00];
assign inst_bltz   = op_d[6'h01] & rt_d[5'h00];
assign inst_j      = op_d[6'h02];
assign inst_bltzal = op_d[6'h01] & rt_d[5'h10];
assign inst_bgezal = op_d[6'h01] & rt_d[5'h11];
assign inst_jalr   = op_d[6'h00] & func_d[6'h09] & rt_d[5'h00] & sa_d[5'h00];

assign inst_lb     = op_d[6'h20];
assign inst_lbu    = op_d[6'h24];
assign inst_lh     = op_d[6'h21];
assign inst_lhu    = op_d[6'h25];

assign inst_sb     = op_d[6'h28];
assign inst_sh     = op_d[6'h29];

assign inst_lwl    = op_d[6'h22];
assign inst_lwr    = op_d[6'h26];
assign inst_swl    = op_d[6'h2a];
assign inst_swr    = op_d[6'h2e];



//------------------添加新的转移和访存类指令------------------

//------------------添加中断与特权指令------------------
wire inst_mfc0;
wire inst_mtc0;
wire inst_eret;
wire syscall;

assign inst_mfc0   = op_d[6'h10] & rs_d[5'h00] & sa_d[5'h00];
assign inst_mtc0   = op_d[6'h10] & rs_d[5'h04] & sa_d[5'h00];
assign inst_eret   = op_d[6'h10] & rs_d[5'h10] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];

assign syscall     = op_d[6'h00] & func_d[6'h0c];

//------------------添加中断与特权指令------------------


assign inst_addu   = op_d[6'h00] & func_d[6'h21] & sa_d[5'h00];
assign inst_subu   = op_d[6'h00] & func_d[6'h23] & sa_d[5'h00];
assign inst_slt    = op_d[6'h00] & func_d[6'h2a] & sa_d[5'h00];
assign inst_sltu   = op_d[6'h00] & func_d[6'h2b] & sa_d[5'h00];
assign inst_and    = op_d[6'h00] & func_d[6'h24] & sa_d[5'h00];
assign inst_or     = op_d[6'h00] & func_d[6'h25] & sa_d[5'h00];
assign inst_xor    = op_d[6'h00] & func_d[6'h26] & sa_d[5'h00];
assign inst_nor    = op_d[6'h00] & func_d[6'h27] & sa_d[5'h00];
assign inst_sll    = op_d[6'h00] & func_d[6'h00] & rs_d[5'h00];
assign inst_srl    = op_d[6'h00] & func_d[6'h02] & rs_d[5'h00];
assign inst_sra    = op_d[6'h00] & func_d[6'h03] & rs_d[5'h00];
assign inst_addiu  = op_d[6'h09];
assign inst_lui    = op_d[6'h0f] & rs_d[5'h00];
assign inst_lw     = op_d[6'h23];
assign inst_sw     = op_d[6'h2b];
assign inst_beq    = op_d[6'h04];
assign inst_bne    = op_d[6'h05];
assign inst_jal    = op_d[6'h03];
assign inst_jr     = op_d[6'h00] & func_d[6'h08] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];

assign alu_op[ 0] = inst_addu | inst_addiu | inst_lwl | inst_lwr | inst_lw | inst_lb | inst_lbu | inst_lh | inst_lhu | inst_sw |inst_swr|inst_swl|inst_sb |inst_sh| inst_jal | inst_bltzal | inst_bgezal | inst_jalr | inst_add | inst_addi;
assign alu_op[ 1] = inst_subu | inst_sub;
assign alu_op[ 2] = inst_slt  | inst_slti; 
assign alu_op[ 3] = inst_sltu | inst_sltiu;
assign alu_op[ 4] = inst_and  | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or   | inst_ori;
assign alu_op[ 7] = inst_xor  | inst_xori;
assign alu_op[ 8] = inst_sll  | inst_sllv;
assign alu_op[ 9] = inst_srl  | inst_srlv;
assign alu_op[10] = inst_sra  | inst_srav;
assign alu_op[11] = inst_lui;
assign alu_op[12] = inst_mult;
assign alu_op[13] = inst_multu;
assign alu_op[14] = inst_div;
assign alu_op[15] = inst_divu;
assign alu_op[16] = inst_mfhi;//借用aluop传输，实际上不走ALU
assign alu_op[17] = inst_mflo;//·
assign alu_op[18] = inst_mthi;//·
assign alu_op[19] = inst_mtlo;//·
assign alu_op[20] = inst_lb;
assign alu_op[21] = inst_lbu;
assign alu_op[22] = inst_lh;
assign alu_op[23] = inst_lhu;
assign alu_op[24] = inst_sb;
assign alu_op[25] = inst_sh;
assign alu_op[26] = inst_lwl;
assign alu_op[27] = inst_lwr;
assign alu_op[28] = inst_swl;
assign alu_op[29] = inst_swr;



assign load_op      = inst_lwr   | inst_lwl | inst_lw | inst_lb | inst_lbu | inst_lh | inst_lhu;//bug fixed4: load_op只有lw指令才为1
assign src1_is_sa   = inst_sll   | inst_srl | inst_sra;
assign src1_is_pc   = inst_jal   | inst_jalr| inst_bltzal | inst_bgezal;
assign src2_is_imm  = inst_addiu | inst_lui | inst_lwr   | inst_lwl | inst_lw | inst_lb | inst_lbu | inst_lhu| inst_lh | inst_sw |inst_sb | inst_sh | inst_swr | inst_swl | inst_addi | inst_slti | inst_sltiu;
assign src2_is_zero_extended_imm = inst_andi | inst_ori | inst_xori;
assign src2_is_8    = inst_jal   | inst_jalr | inst_bltzal | inst_bgezal;
assign res_from_mem = inst_lw    | inst_lwr  | inst_lwl | inst_lb | inst_lbu | inst_lhu | inst_lh;
assign dst_is_r31   = inst_jal   | inst_bltzal | inst_bgezal;
assign dst_is_rt    = inst_addiu | inst_lui | inst_lw | inst_lwr  | inst_lwl | inst_lb | inst_lbu | inst_lhu |inst_lh| inst_addi | inst_andi | inst_ori | inst_xori |inst_slti |inst_sltiu;
assign gr_we        = ~inst_sw &~inst_sb &~inst_sh &~inst_swr &~inst_swl & ~inst_beq & ~inst_bne & ~inst_jr &~inst_bgez &~inst_bgtz &~inst_blez &~inst_bltz  &~inst_j & ~inst_mult & ~inst_multu & ~inst_div & ~inst_divu & ~inst_mthi & ~inst_mtlo;//mtlo和mthi指令写的是HI和LO寄存器，不是通用寄存器
//jal&bltzal&bgezal指令也要写寄存器，但是不是通用寄存器，而是31号寄存器
assign mem_we       = inst_sw | inst_sb | inst_sh | inst_swr |inst_swl;
assign dest         = dst_is_r31 ? 5'd31 :
                      dst_is_rt  ? rt    : 
                                   rd;

assign rf_raddr1 = rs;
assign rf_raddr2 = rt;
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

//------------------------by-pass------------------------
//只负责传递，不负责暂停
assign rs_value = (rf_raddr1==es_dest && !es_res_from_mem && es_gr_we) ? es_alu_result :
(rf_raddr1==ms_dest && ms_gr_we)? ms_final_result :
(rf_raddr1==rf_waddr && rf_we)? rf_wdata : rf_rdata1;

assign rt_value = (rf_raddr2==es_dest && !es_res_from_mem && es_gr_we) ? es_alu_result :
(rf_raddr2==ms_dest && ms_gr_we)? ms_final_result :
(rf_raddr2==rf_waddr && rf_we)? rf_wdata : 
(inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bltzal|inst_bgezal)? 32'b0 : rf_rdata2; //因为bgez指令的rt字段是指定00001，所以这里rt_value指定为0


//------------------------分支跳转------------------------
assign rs_eq_rt = (rs_value == rt_value);
assign rs_ge_rt = ($signed(rs_value) >= $signed(rt_value));//后续把这里改成gez和gtz
assign rs_gt_rt = ($signed(rs_value) >  $signed(rt_value));
assign br_taken = (   inst_beq  &&  rs_eq_rt
                   || inst_bne  && !rs_eq_rt
                   || inst_jal
                   || inst_jr
                   || inst_j
                   || inst_jalr
                   || inst_bgez &&  rs_ge_rt
                   || inst_bgtz &&  rs_gt_rt
                   || inst_blez && !rs_gt_rt  
                   || inst_bltz && !rs_ge_rt
                   || inst_bltzal && !rs_ge_rt
                   || inst_bgezal &&  rs_ge_rt
                  ) && ds_valid;
assign br_target = (inst_beq || inst_bne || inst_bgez || inst_bgtz || inst_blez || inst_bltz || inst_bgezal || inst_bltzal) ? (fs_pc + {{14{imm[15]}}, imm[15:0], 2'b0}) :
                   (inst_jr || inst_jalr)              ? rs_value :
                  /*inst_jal&inst_j*/              {fs_pc[31:28], jidx[25:0], 2'b0};
assign br_stall  = fs_to_ds_bus_r==0 ? 0 : (es_res_from_mem && (es_dest == rs || es_dest == rt) && (inst_beq || inst_bne || inst_bgez || inst_bgtz || inst_blez || inst_bltz )) || (es_res_from_mem && es_dest == rs && (inst_jr || inst_jalr || inst_bgezal || inst_bltzal)); 

endmodule
