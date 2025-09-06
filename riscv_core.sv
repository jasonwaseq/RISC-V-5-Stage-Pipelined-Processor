module riscv_core (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [31:0] instr_data,
    input  logic [31:0] mem_data_in,
    output logic [31:0] pc,
    output logic        instr_read,
    output logic [31:0] mem_addr,
    output logic [31:0] mem_data_out,
    output logic        mem_write,
    output logic        mem_read
);
    
    import riscv_pkg::*;
    
    // Pipeline registers
    logic [31:0] if_instr, if_pc;
    logic [31:0] id_instr, id_pc;
    ctrl_signals_t id_ctrl, ex_ctrl, mem_ctrl, wb_ctrl;
    logic [31:0] ex_rs1_data, ex_rs2_data, ex_imm, ex_pc;
    logic [4:0]  ex_rs1_addr, ex_rs2_addr, ex_rd_addr;
    logic [31:0] mem_alu_result, mem_rs2_data, mem_pc_plus_4;
    logic [4:0]  mem_rd_addr;
    logic [31:0] wb_alu_result, wb_mem_data, wb_pc_plus_4;
    logic [4:0]  wb_rd_addr;
    
    // Hazard detection
    logic stall, flush;
    logic forward_a, forward_b;
    logic [1:0] forward_a_src, forward_b_src;
    
    // Branch prediction
    logic branch_taken;
    logic [31:0] branch_target;
    
    // ALU inputs
    logic [31:0] alu_a, alu_b;
    logic [31:0] alu_result;
    logic        alu_zero;
    
    // Register file
    logic [31:0] rs1_data, rs2_data;
    logic        reg_write;
    logic [31:0] write_data;
    
    // Immediate generation
    logic [31:0] imm;
    
    // Program counter
    logic [31:0] next_pc;
    logic [31:0] pc_plus_4;
    
    // ==================== IF STAGE ====================
    assign pc_plus_4 = pc + 4;
    assign next_pc = (branch_taken) ? branch_target : pc_plus_4;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 32'h00000000;
        end else if (!stall) begin
            pc <= next_pc;
        end
    end
    
    assign instr_read = 1'b1;
    assign if_instr = instr_data;
    assign if_pc = pc;
    
    // IF/ID Pipeline Register
    if_id_reg if_id (
        .clk(clk),
        .rst_n(rst_n),
        .stall(stall),
        .flush(flush),
        .pc_in(if_pc),
        .instr_in(if_instr),
        .pc_out(id_pc),
        .instr_out(id_instr)
    );
    
    // ==================== ID STAGE ====================
    control_unit ctrl (
        .opcode(id_instr[6:0]),
        .funct3(id_instr[14:12]),
        .funct7(id_instr[31:25]),
        .ctrl(id_ctrl)
    );
    
    imm_gen imm_gen (
        .instr(id_instr),
        .imm_out(imm)
    );
    
    reg_file reg_file (
        .clk(clk),
        .rst_n(rst_n),
        .we(reg_write),
        .addr_rs1(id_instr[19:15]),
        .addr_rs2(id_instr[24:20]),
        .addr_rd(wb_rd_addr),
        .data_rd(write_data),
        .data_rs1(rs1_data),
        .data_rs2(rs2_data)
    );
    
    hazard_unit hazard (
        .id_rs1(id_instr[19:15]),
        .id_rs2(id_instr[24:20]),
        .ex_rd(ex_rd_addr),
        .ex_mem_read(ex_ctrl.mem_read),
        .mem_rd(mem_rd_addr),
        .mem_reg_write(mem_ctrl.reg_write),
        .wb_rd(wb_rd_addr),
        .wb_reg_write(wb_ctrl.reg_write),
        .stall(stall),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .forward_a_src(forward_a_src),
        .forward_b_src(forward_b_src)
    );
    
    branch_unit branch (
        .pc(id_pc),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data),
        .alu_op(id_ctrl.alu_op),
        .imm(imm),
        .branch_taken(branch_taken),
        .target_addr(branch_target)
    );
    
    assign flush = branch_taken;
    
    // ID/EX Pipeline Register
    id_ex_reg id_ex (
        .clk(clk),
        .rst_n(rst_n),
        .flush(flush),
        .ctrl_in(id_ctrl),
        .pc_in(id_pc),
        .rs1_data_in(rs1_data),
        .rs2_data_in(rs2_data),
        .imm_in(imm),
        .rs1_addr_in(id_instr[19:15]),
        .rs2_addr_in(id_instr[24:20]),
        .rd_addr_in(id_instr[11:7]),
        .ctrl_out(ex_ctrl),
        .pc_out(ex_pc),
        .rs1_data_out(ex_rs1_data),
        .rs2_data_out(ex_rs2_data),
        .imm_out(ex_imm),
        .rs1_addr_out(ex_rs1_addr),
        .rs2_addr_out(ex_rs2_addr),
        .rd_addr_out(ex_rd_addr)
    );
    
    // ==================== EX STAGE ====================
    // Forwarding multiplexers
    always_comb begin
        case (forward_a_src)
            2'b00: alu_a = ex_rs1_data;
            2'b01: alu_a = mem_alu_result;
            2'b10: alu_a = write_data;
            default: alu_a = ex_rs1_data;
        endcase
        
        case (forward_b_src)
            2'b00: alu_b = ex_rs2_data;
            2'b01: alu_b = mem_alu_result;
            2'b10: alu_b = write_data;
            default: alu_b = ex_rs2_data;
        endcase
    end
    
    logic [31:0] alu_b_operand;
    assign alu_b_operand = (ex_ctrl.alu_src) ? ex_imm : alu_b;
    
    alu alu (
        .a(alu_a),
        .b(alu_b_operand),
        .alu_op(ex_ctrl.alu_op),
        .result(alu_result),
        .zero(alu_zero)
    );
    
    // EX/MEM Pipeline Register
    ex_mem_reg ex_mem (
        .clk(clk),
        .rst_n(rst_n),
        .ctrl_in(ex_ctrl),
        .alu_result_in(alu_result),
        .rs2_data_in(alu_b),
        .rd_addr_in(ex_rd_addr),
        .pc_plus_4_in(ex_pc + 4),
        .ctrl_out(mem_ctrl),
        .alu_result_out(mem_alu_result),
        .rs2_data_out(mem_rs2_data),
        .rd_addr_out(mem_rd_addr),
        .pc_plus_4_out(mem_pc_plus_4)
    );
    
    // ==================== MEM STAGE ====================
    assign mem_addr = mem_alu_result;
    assign mem_data_out = mem_rs2_data;
    assign mem_write = mem_ctrl.mem_write;
    assign mem_read = mem_ctrl.mem_read;
    
    // MEM/WB Pipeline Register
    mem_wb_reg mem_wb (
        .clk(clk),
        .rst_n(rst_n),
        .ctrl_in(mem_ctrl),
        .alu_result_in(mem_alu_result),
        .mem_data_in(mem_data_in),
        .rd_addr_in(mem_rd_addr),
        .pc_plus_4_in(mem_pc_plus_4),
        .ctrl_out(wb_ctrl),
        .alu_result_out(wb_alu_result),
        .mem_data_out(wb_mem_data),
        .rd_addr_out(wb_rd_addr),
        .pc_plus_4_out(wb_pc_plus_4)
    );
    
    // ==================== WB STAGE ====================
    assign write_data = (wb_ctrl.mem_to_reg) ? wb_mem_data : 
                       (wb_ctrl.jump != 2'b00) ? wb_pc_plus_4 : 
                       wb_alu_result;
    
    assign reg_write = wb_ctrl.reg_write;
    
endmodule