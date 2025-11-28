`include "IKA87AD_mnemonics.sv"

module IKA87AD_microcode (
    input   wire                i_CLK,
    input   wire                i_MCROM_READ_TICK,
    input   wire    [7:0]       i_MCROM_ADDR,
    output  wire    [17:0]      o_MCROM_DATA
);

(* ramstyle = "M10K" *) reg     [17:0]  mc;
assign  o_MCROM_DATA = mc;

always @(posedge i_CLK) if(i_MCROM_READ_TICK) begin
    case(i_MCROM_ADDR)
        //1-cycle opcode group
        NOP             : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};

        //2-cycle opcode group
        MVI_R_IM        : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};
        MVI_R_IM+1      : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_R, T0_DEU_MOV, RD4};


        default         : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        IRD             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b1, RD4};   //wait for decoding
    endcase
end

endmodule