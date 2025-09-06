package riscv_pkg;
    // Opcodes
    typedef enum logic [6:0] {
        OP_LOAD     = 7'b0000011,
        OP_STORE    = 7'b0100011,
        OP_BRANCH   = 7'b1100011,
        OP_JALR     = 7'b1100111,
        OP_JAL      = 7'b1101111,
        OP_OP_IMM   = 7'b0010011,
        OP_OP       = 7'b0110011,
        OP_LUI      = 7'b0110111,
        OP_AUIPC    = 7'b0010111
    } opcode_t;
    
    // ALU operations
    typedef enum logic [3:0] {
        ALU_ADD     = 4'b0000,
        ALU_SUB     = 4'b0001,
        ALU_AND     = 4'b0010,
        ALU_OR      = 4'b0011,
        ALU_XOR     = 4'b0100,
        ALU_SLL     = 4'b0101,
        ALU_SRL     = 4'b0110,
        ALU_SRA     = 4'b0111,
        ALU_SLT     = 4'b1000,
        ALU_SLTU    = 4'b1001,
        ALU_LUI     = 4'b1010,
        ALU_BEQ     = 4'b1011,
        ALU_BNE     = 4'b1100,
        ALU_BLT     = 4'b1101,
        ALU_BGE     = 4'b1110,
        ALU_BLTU    = 4'b1111
    } alu_op_t;
    
    // Control signals
    typedef struct packed {
        logic       reg_write;
        logic       mem_to_reg;
        logic       mem_write;
        logic       mem_read;
        logic       branch;
        logic       alu_src;
        logic [1:0] alu_src_b;
        logic [1:0] jump;
        alu_op_t    alu_op;
        logic [2:0] funct3;
    } ctrl_signals_t;
    
endpackage