module hazard_unit (
    input  logic [4:0] id_rs1,
    input  logic [4:0] id_rs2,
    input  logic [4:0] ex_rd,
    input  logic       ex_mem_read,
    input  logic [4:0] mem_rd,
    input  logic       mem_reg_write,
    input  logic       wb_reg_write,
    input  logic [4:0] wb_rd,
    output logic       stall,
    output logic       forward_a,
    output logic       forward_b,
    output logic [1:0] forward_a_src,
    output logic [1:0] forward_b_src
);
    
    // Data hazard detection
    logic ex_hazard_rs1, ex_hazard_rs2;
    logic mem_hazard_rs1, mem_hazard_rs2;
    
    assign ex_hazard_rs1 = (id_rs1 != 0) && (id_rs1 == ex_rd) && ex_mem_read;
    assign ex_hazard_rs2 = (id_rs2 != 0) && (id_rs2 == ex_rd) && ex_mem_read;
    
    assign stall = ex_hazard_rs1 || ex_hazard_rs2;
    
    // Forwarding logic
    always_comb begin
        forward_a = 1'b0;
        forward_b = 1'b0;
        forward_a_src = 2'b00;
        forward_b_src = 2'b00;
        
        // EX hazard
        if ((id_rs1 != 0) && (id_rs1 == ex_rd) && !ex_mem_read) begin
            forward_a = 1'b1;
            forward_a_src = 2'b01;
        end
        if ((id_rs2 != 0) && (id_rs2 == ex_rd) && !ex_mem_read) begin
            forward_b = 1'b1;
            forward_b_src = 2'b01;
        end
        
        // MEM hazard
        if ((id_rs1 != 0) && (id_rs1 == mem_rd) && mem_reg_write) begin
            forward_a = 1'b1;
            forward_a_src = 2'b10;
        end
        if ((id_rs2 != 0) && (id_rs2 == mem_rd) && mem_reg_write) begin
            forward_b = 1'b1;
            forward_b_src = 2'b10;
        end
    end
    
endmodule