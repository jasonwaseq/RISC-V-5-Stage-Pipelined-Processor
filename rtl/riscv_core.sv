// RISC-V 5-Stage Pipelined Processor
// F: Fetch, D: Decode, X: Execute, M: Memory, W: Writeback

module riscv_core (
    input  logic        clk,
    input  logic        rst_n,
    // Instruction memory interface
    output logic [31:0] imem_addr,
    input  logic [31:0] imem_data,
    // Data memory interface
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    input  logic [31:0] dmem_rdata,
    output logic        dmem_we,
    output logic [3:0]  dmem_be,
    // Debug interface
    input  logic [4:0]  dbg_reg_addr,
    output logic [31:0] dbg_reg_data
);

    // =========================================================================
    // Register File
    // =========================================================================
    logic [31:0] regs [0:31];
    
    // Read ports
    logic [4:0]  rs1, rs2, rd;
    logic [31:0] rs1_data, rs2_data;
    
    // Write port
    logic [4:0]  wb_rd;
    logic [31:0] wb_data;
    logic        wb_we;
    
    // Extract register addresses
    assign rs1 = imem_data[19:15];
    assign rs2 = imem_data[24:20];
    assign rd  = imem_data[11:7];
    
    // Registered reads
    logic [31:0] rs1_data_r, rs2_data_r;
    always_ff @(posedge clk) begin
        rs1_data_r <= (rs1 == 5'd0) ? 32'd0 : regs[rs1];
        rs2_data_r <= (rs2 == 5'd0) ? 32'd0 : regs[rs2];
    end
    assign rs1_data = rs1_data_r;
    assign rs2_data = rs2_data_r;
    
    // Debug read
    assign dbg_reg_data = (dbg_reg_addr == 5'd0) ? 32'd0 : regs[dbg_reg_addr];
    
    // Write back
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 1; i < 32; i++) regs[i] <= 32'd0;
            regs[0] <= 32'd0;
        end else if (wb_we && wb_rd != 5'd0) begin
            regs[wb_rd] <= wb_data;
        end
    end

    // =========================================================================
    // FETCH Stage
    // =========================================================================
    logic [31:0] pc;
    always_ff @(posedge clk) begin
        if (!rst_n)
            pc <= 32'd0;
        else
            pc <= pc + 32'd4;
    end
    assign imem_addr = pc;

    // Pipeline registers: IF/ID
    logic [31:0] if_id_instr, if_id_pc;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            if_id_instr <= 32'h00000013;  // NOP
            if_id_pc    <= 32'd0;
        end else begin
            if_id_instr <= imem_data;
            if_id_pc    <= pc;
        end
    end

    // =========================================================================
    // DECODE Stage
    // =========================================================================
    logic [6:0]  opcode, funct7;
    logic [2:0]  funct3;
    logic [31:0] imm;
    
    assign opcode = if_id_instr[6:0];
    assign funct3 = if_id_instr[14:12];
    assign funct7 = if_id_instr[31:25];
    
    // Immediate generation
    always_comb begin
        case (opcode)
            7'b0010011, 7'b0000011: imm = {{20{if_id_instr[31]}}, if_id_instr[31:20]};  // I-type
            7'b0100011: imm = {{20{if_id_instr[31]}}, if_id_instr[31:25], if_id_instr[11:7]};  // S-type
            default: imm = 32'd0;
        endcase
    end
    
    // Use registered operands
    logic [31:0] d_rs1, d_rs2;
    assign d_rs1 = rs1_data;
    assign d_rs2 = rs2_data;

    // Pipeline registers: ID/EX
    logic [31:0] id_ex_rs1, id_ex_rs2, id_ex_imm, id_ex_pc;
    logic [4:0]  id_ex_rd;
    logic [6:0]  id_ex_opcode;
    logic [2:0]  id_ex_funct3;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            id_ex_rs1 <= 32'd0;
            id_ex_rs2 <= 32'd0;
            id_ex_imm <= 32'd0;
            id_ex_pc <= 32'd0;
            id_ex_rd <= 5'd0;
            id_ex_opcode <= 7'd0;
            id_ex_funct3 <= 3'd0;
        end else begin
            id_ex_rs1 <= d_rs1;
            id_ex_rs2 <= d_rs2;
            id_ex_imm <= imm;
            id_ex_pc <= if_id_pc;
            id_ex_rd <= if_id_instr[11:7];
            id_ex_opcode <= opcode;
            id_ex_funct3 <= funct3;
        end
    end

    // =========================================================================
    // EXECUTE Stage
    // =========================================================================
    logic [31:0] ex_alu_a, ex_alu_b, ex_alu_result;
    logic        ex_is_imm, ex_is_load, ex_is_store;
    
    assign ex_is_imm = (id_ex_opcode == 7'b0010011 || id_ex_opcode == 7'b0000011);
    assign ex_is_load = (id_ex_opcode == 7'b0000011);
    assign ex_is_store = (id_ex_opcode == 7'b0100011);
    
    assign ex_alu_a = id_ex_rs1;
    assign ex_alu_b = ex_is_imm ? id_ex_imm : id_ex_rs2;
    
    // Simple ALU - ADD only
    assign ex_alu_result = ex_alu_a + ex_alu_b;

    // Pipeline registers: EX/MEM
    logic [31:0] ex_mem_alu, ex_mem_rs2;
    logic [4:0]  ex_mem_rd;
    logic        ex_mem_is_load, ex_mem_is_store;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ex_mem_alu <= 32'd0;
            ex_mem_rs2 <= 32'd0;
            ex_mem_rd <= 5'd0;
            ex_mem_is_load <= 1'b0;
            ex_mem_is_store <= 1'b0;
        end else begin
            ex_mem_alu <= ex_alu_result;
            ex_mem_rs2 <= id_ex_rs2;
            ex_mem_rd <= id_ex_rd;
            ex_mem_is_load <= ex_is_load;
            ex_mem_is_store <= ex_is_store;
        end
    end

    // =========================================================================
    // MEMORY Stage
    // =========================================================================
    assign dmem_addr = ex_mem_alu;
    assign dmem_wdata = ex_mem_rs2;
    assign dmem_we = ex_mem_is_store;
    assign dmem_be = 4'b1111;

    // Pipeline registers: MEM/WB
    logic [31:0] mem_wb_alu, mem_wb_mem_data;
    logic [4:0]  mem_wb_rd;
    logic        mem_wb_is_load;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            mem_wb_alu <= 32'd0;
            mem_wb_mem_data <= 32'd0;
            mem_wb_rd <= 5'd0;
            mem_wb_is_load <= 1'b0;
        end else begin
            mem_wb_alu <= ex_mem_alu;
            mem_wb_mem_data <= dmem_rdata;
            mem_wb_rd <= ex_mem_rd;
            mem_wb_is_load <= ex_mem_is_load;
        end
    end

    // =========================================================================
    // WRITEBACK Stage
    // =========================================================================
    assign wb_we = (mem_wb_rd != 5'd0);
    assign wb_rd = mem_wb_rd;
    assign wb_data = mem_wb_is_load ? mem_wb_mem_data : mem_wb_alu;

endmodule
