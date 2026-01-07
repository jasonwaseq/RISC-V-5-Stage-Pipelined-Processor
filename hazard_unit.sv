module hazard_unit (
    id_rs1,
    id_rs2,
    ex_rd,
    ex_mem_read,
    mem_rd,
    mem_reg_write,
    wb_reg_write,
    wb_rd,
    stall,
    forward_a,
    forward_b,
    forward_a_src,
    forward_b_src
);
    input [4:0] id_rs1;
    input [4:0] id_rs2;
    input [4:0] ex_rd;
    input ex_mem_read;
    input [4:0] mem_rd;
    input mem_reg_write;
    input wb_reg_write;
    input [4:0] wb_rd;
    output stall;
    output forward_a;
    output forward_b;
    output [1:0] forward_a_src;
    output [1:0] forward_b_src;
    reg stall;
    reg forward_a;
    reg forward_b;
    reg [1:0] forward_a_src;
    reg [1:0] forward_b_src;

    wire ex_hazard_rs1, ex_hazard_rs2;
    assign ex_hazard_rs1 = (id_rs1 != 0) && (id_rs1 == ex_rd) && ex_mem_read;
    assign ex_hazard_rs2 = (id_rs2 != 0) && (id_rs2 == ex_rd) && ex_mem_read;

    always @* begin
        stall = ex_hazard_rs1 || ex_hazard_rs2;
        forward_a = 0;
        forward_b = 0;
        forward_a_src = 2'b00;
        forward_b_src = 2'b00;
        // EX hazard
        if ((id_rs1 != 0) && (id_rs1 == ex_rd) && !ex_mem_read) begin
            forward_a = 1;
            forward_a_src = 2'b01;
        end
        if ((id_rs2 != 0) && (id_rs2 == ex_rd) && !ex_mem_read) begin
            forward_b = 1;
            forward_b_src = 2'b01;
        end
        // MEM hazard
        if ((id_rs1 != 0) && (id_rs1 == mem_rd) && mem_reg_write) begin
            forward_a = 1;
            forward_a_src = 2'b10;
        end
        if ((id_rs2 != 0) && (id_rs2 == mem_rd) && mem_reg_write) begin
            forward_b = 1;
            forward_b_src = 2'b10;
        end
    end
endmodule