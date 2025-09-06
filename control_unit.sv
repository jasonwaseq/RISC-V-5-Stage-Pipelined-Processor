module control_unit (
    input  logic [6:0] opcode,
    input  logic [2:0] funct3,
    input  logic [6:0] funct7,
    output riscv_pkg::ctrl_signals_t ctrl
);
    
    import riscv_pkg::*;
    
    logic is_rtype;
    
    assign is_rtype = (opcode == OP_OP);
    
    always_comb begin
        // Default values
        ctrl.reg_write = 1'b0;
        ctrl.mem_to_reg = 1'b0;
        ctrl.mem_write = 1'b0;
        ctrl.mem_read = 1'b0;
        ctrl.branch = 1'b0;
        ctrl.alu_src = 1'b0;
        ctrl.alu_src_b = 2'b00;
        ctrl.jump = 2'b00;
        ctrl.alu_op = ALU_ADD;
        ctrl.funct3 = funct3;
        
        case (opcode)
            OP_LOAD: begin
                ctrl.reg_write = 1'b1;
                ctrl.mem_to_reg = 1'b1;
                ctrl.mem_read = 1'b1;
                ctrl.alu_src = 1'b1;
                ctrl.alu_op = ALU_ADD;
            end
            
            OP_STORE: begin
                ctrl.mem_write = 1'b1;
                ctrl.alu_src = 1'b1;
                ctrl.alu_op = ALU_ADD;
            end
            
            OP_BRANCH: begin
                ctrl.branch = 1'b1;
                case (funct3)
                    3'b000: ctrl.alu_op = ALU_BEQ;
                    3'b001: ctrl.alu_op = ALU_BNE;
                    3'b100: ctrl.alu_op = ALU_BLT;
                    3'b101: ctrl.alu_op = ALU_BGE;
                    3'b110: ctrl.alu_op = ALU_BLTU;
                    3'b111: ctrl.alu_op = ALU_BGE;
                endcase
            end
            
            OP_JALR: begin
                ctrl.reg_write = 1'b1;
                ctrl.jump = 2'b10;
                ctrl.alu_src = 1'b1;
            end
            
            OP_JAL: begin
                ctrl.reg_write = 1'b1;
                ctrl.jump = 2'b01;
            end
            
            OP_OP_IMM: begin
                ctrl.reg_write = 1'b1;
                ctrl.alu_src = 1'b1;
                case (funct3)
                    3'b000: ctrl.alu_op = ALU_ADD;
                    3'b010: ctrl.alu_op = ALU_SLT;
                    3'b011: ctrl.alu_op = ALU_SLTU;
                    3'b100: ctrl.alu_op = ALU_XOR;
                    3'b110: ctrl.alu_op = ALU_OR;
                    3'b111: ctrl.alu_op = ALU_AND;
                    3'b001: ctrl.alu_op = ALU_SLL;
                    3'b101: ctrl.alu_op = (funct7[5] ? ALU_SRA : ALU_SRL);
                endcase
            end
            
            OP_OP: begin
                ctrl.reg_write = 1'b1;
                case ({funct7[5], funct3})
                    4'b0000: ctrl.alu_op = ALU_ADD;
                    4'b1000: ctrl.alu_op = ALU_SUB;
                    4'b0001: ctrl.alu_op = ALU_SLL;
                    4'b0010: ctrl.alu_op = ALU_SLT;
                    4'b0011: ctrl.alu_op = ALU_SLTU;
                    4'b0100: ctrl.alu_op = ALU_XOR;
                    4'b0101: ctrl.alu_op = ALU_SRL;
                    4'b1101: ctrl.alu_op = ALU_SRA;
                    4'b0110: ctrl.alu_op = ALU_OR;
                    4'b0111: ctrl.alu_op = ALU_AND;
                endcase
            end
            
            OP_LUI: begin
                ctrl.reg_write = 1'b1;
                ctrl.alu_src_b = 2'b10;
                ctrl.alu_op = ALU_LUI;
            end
            
            OP_AUIPC: begin
                ctrl.reg_write = 1'b1;
                ctrl.alu_src_b = 2'b01;
                ctrl.alu_op = ALU_ADD;
            end
        endcase
    end
    
endmodule