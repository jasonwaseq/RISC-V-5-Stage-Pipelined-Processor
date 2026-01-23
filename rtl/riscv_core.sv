// RISC-V 5-Stage Pipelined Processor
// F: Fetch, D: Decode, X: Execute, M: Memory, W: Writeback

module riscv_core (
    input  logic        clk,
    input  logic        rst_n,       // Pipeline reset (can be used for halt)
    input  logic        rf_rst_n,    // Register file reset (only on hard reset)
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
    // Register File (only reset on hard reset, not on halt)
    // =========================================================================
    logic [31:0] regs [0:31];
    
    // Write port signals (from writeback stage)
    logic [4:0]  wb_rd;
    logic [31:0] wb_data;
    logic        wb_we;
    
    // Debug read - combinational
    assign dbg_reg_data = (dbg_reg_addr == 5'd0) ? 32'd0 : regs[dbg_reg_addr];
    
    // Register file write - only reset on hard reset
    always_ff @(posedge clk) begin
        if (!rf_rst_n) begin
            for (int i = 0; i < 32; i++) regs[i] <= 32'd0;
        end else if (wb_we && wb_rd != 5'd0) begin
            regs[wb_rd] <= wb_data;
        end
    end

    // =========================================================================
    // FETCH Stage
    // =========================================================================
    logic [31:0] pc, next_pc;
    
    assign next_pc = pc + 32'd4;
    
    always_ff @(posedge clk) begin
        if (!rst_n)
            pc <= 32'd0;
        else
            pc <= next_pc;
    end
    
    // Fetch instruction at current PC (not next PC)
    assign imem_addr = pc;

    // =========================================================================
    // Pipeline Register: IF/ID
    // =========================================================================
    logic [31:0] if_id_instr, if_id_pc;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            if_id_instr <= 32'h00000013;  // NOP (ADDI x0, x0, 0)
            if_id_pc    <= 32'd0;
        end else begin
            if_id_instr <= imem_data;
            if_id_pc    <= pc;
        end
    end

    // =========================================================================
    // DECODE Stage
    // =========================================================================
    // Extract fields from instruction
    logic [6:0]  d_opcode, d_funct7;
    logic [2:0]  d_funct3;
    logic [4:0]  d_rs1_addr, d_rs2_addr, d_rd_addr;
    logic [31:0] d_imm;
    logic [31:0] d_rs1_data, d_rs2_data;
    
    assign d_opcode   = if_id_instr[6:0];
    assign d_funct3   = if_id_instr[14:12];
    assign d_funct7   = if_id_instr[31:25];
    assign d_rs1_addr = if_id_instr[19:15];
    assign d_rs2_addr = if_id_instr[24:20];
    assign d_rd_addr  = if_id_instr[11:7];
    
    // Register file read (combinational)
    assign d_rs1_data = (d_rs1_addr == 5'd0) ? 32'd0 : regs[d_rs1_addr];
    assign d_rs2_data = (d_rs2_addr == 5'd0) ? 32'd0 : regs[d_rs2_addr];
    
    // Immediate generation
    always_comb begin
        case (d_opcode)
            7'b0010011, // I-type ALU (ADDI, etc)
            7'b0000011: // Load
                d_imm = {{20{if_id_instr[31]}}, if_id_instr[31:20]};
            7'b0100011: // Store (S-type)
                d_imm = {{20{if_id_instr[31]}}, if_id_instr[31:25], if_id_instr[11:7]};
            default: 
                d_imm = 32'd0;
        endcase
    end

    // =========================================================================
    // Pipeline Register: ID/EX
    // =========================================================================
    logic [31:0] id_ex_rs1, id_ex_rs2, id_ex_imm, id_ex_pc;
    logic [4:0]  id_ex_rd;
    logic [6:0]  id_ex_opcode, id_ex_funct7;
    logic [2:0]  id_ex_funct3;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            id_ex_rs1    <= 32'd0;
            id_ex_rs2    <= 32'd0;
            id_ex_imm    <= 32'd0;
            id_ex_pc     <= 32'd0;
            id_ex_rd     <= 5'd0;
            id_ex_opcode <= 7'b0010011;  // NOP opcode
            id_ex_funct3 <= 3'd0;
            id_ex_funct7 <= 7'd0;
        end else begin
            id_ex_rs1    <= d_rs1_data;
            id_ex_rs2    <= d_rs2_data;
            id_ex_imm    <= d_imm;
            id_ex_pc     <= if_id_pc;
            id_ex_rd     <= d_rd_addr;
            id_ex_opcode <= d_opcode;
            id_ex_funct3 <= d_funct3;
            id_ex_funct7 <= d_funct7;
        end
    end

    // =========================================================================
    // EXECUTE Stage
    // =========================================================================
    logic [31:0] ex_alu_a, ex_alu_b, ex_alu_result;
    logic        ex_is_imm, ex_is_load, ex_is_store, ex_is_r_type;
    logic [4:0]  ex_shamt;
    
    // Decode instruction type
    assign ex_is_r_type = (id_ex_opcode == 7'b0110011);  // R-type (ADD, etc)
    assign ex_is_imm    = (id_ex_opcode == 7'b0010011);  // I-type ALU (ADDI)
    assign ex_is_load   = (id_ex_opcode == 7'b0000011);  // Load
    assign ex_is_store  = (id_ex_opcode == 7'b0100011);  // Store
    
    // ALU operand selection
    assign ex_alu_a = id_ex_rs1;
    assign ex_alu_b = (ex_is_imm || ex_is_load || ex_is_store) ? id_ex_imm : id_ex_rs2;
    
    // Shift amount (from immediate for I-type, rs2 for R-type)
    assign ex_shamt = ex_is_imm ? id_ex_imm[4:0] : id_ex_rs2[4:0];
    
    // Full ALU implementation
    always_comb begin
        ex_alu_result = 32'd0;
        
        if (ex_is_load || ex_is_store) begin
            // Load/Store: address calculation (ADD)
            ex_alu_result = ex_alu_a + ex_alu_b;
        end
        else begin
            // ALU operations based on funct3
            case (id_ex_funct3)
                3'b000: begin // ADD/SUB/ADDI
                    if (ex_is_r_type && id_ex_funct7[5])
                        ex_alu_result = ex_alu_a - ex_alu_b;  // SUB
                    else
                        ex_alu_result = ex_alu_a + ex_alu_b;  // ADD/ADDI
                end
                3'b001: // SLL/SLLI
                    ex_alu_result = ex_alu_a << ex_shamt;
                3'b010: // SLT/SLTI (signed comparison)
                    ex_alu_result = ($signed(ex_alu_a) < $signed(ex_alu_b)) ? 32'd1 : 32'd0;
                3'b011: // SLTU/SLTIU (unsigned comparison)
                    ex_alu_result = (ex_alu_a < ex_alu_b) ? 32'd1 : 32'd0;
                3'b100: // XOR/XORI
                    ex_alu_result = ex_alu_a ^ ex_alu_b;
                3'b101: begin // SRL/SRLI/SRA/SRAI
                    if (id_ex_funct7[5])
                        ex_alu_result = $signed(ex_alu_a) >>> ex_shamt;  // SRA/SRAI
                    else
                        ex_alu_result = ex_alu_a >> ex_shamt;  // SRL/SRLI
                end
                3'b110: // OR/ORI
                    ex_alu_result = ex_alu_a | ex_alu_b;
                3'b111: // AND/ANDI
                    ex_alu_result = ex_alu_a & ex_alu_b;
                default:
                    ex_alu_result = 32'd0;
            endcase
        end
    end

    // =========================================================================
    // Pipeline Register: EX/MEM
    // =========================================================================
    logic [31:0] ex_mem_alu, ex_mem_rs2;
    logic [4:0]  ex_mem_rd;
    logic        ex_mem_is_load, ex_mem_is_store;
    logic        ex_mem_reg_write;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ex_mem_alu       <= 32'd0;
            ex_mem_rs2       <= 32'd0;
            ex_mem_rd        <= 5'd0;
            ex_mem_is_load   <= 1'b0;
            ex_mem_is_store  <= 1'b0;
            ex_mem_reg_write <= 1'b0;
        end else begin
            ex_mem_alu       <= ex_alu_result;
            ex_mem_rs2       <= id_ex_rs2;
            ex_mem_rd        <= id_ex_rd;
            ex_mem_is_load   <= ex_is_load;
            ex_mem_is_store  <= ex_is_store;
            // Register write for R-type, I-type ALU, and loads
            ex_mem_reg_write <= ex_is_r_type || ex_is_imm || ex_is_load;
        end
    end

    // =========================================================================
    // MEMORY Stage
    // =========================================================================
    assign dmem_addr  = ex_mem_alu;
    assign dmem_wdata = ex_mem_rs2;
    assign dmem_we    = ex_mem_is_store;
    assign dmem_be    = 4'b1111;

    // =========================================================================
    // Pipeline Register: MEM/WB
    // =========================================================================
    logic [31:0] mem_wb_alu, mem_wb_mem_data;
    logic [4:0]  mem_wb_rd;
    logic        mem_wb_is_load;
    logic        mem_wb_reg_write;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            mem_wb_alu       <= 32'd0;
            mem_wb_mem_data  <= 32'd0;
            mem_wb_rd        <= 5'd0;
            mem_wb_is_load   <= 1'b0;
            mem_wb_reg_write <= 1'b0;
        end else begin
            mem_wb_alu       <= ex_mem_alu;
            mem_wb_mem_data  <= dmem_rdata;
            mem_wb_rd        <= ex_mem_rd;
            mem_wb_is_load   <= ex_mem_is_load;
            mem_wb_reg_write <= ex_mem_reg_write;
        end
    end

    // =========================================================================
    // WRITEBACK Stage
    // =========================================================================
    assign wb_we   = mem_wb_reg_write && (mem_wb_rd != 5'd0);
    assign wb_rd   = mem_wb_rd;
    assign wb_data = mem_wb_is_load ? mem_wb_mem_data : mem_wb_alu;

endmodule
