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
        //                       MCTYPE   FLAG  SKIP
        //Move an immediate data
        MVI_R_IM        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MVI_R_IM+1      : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_R, T0_DEU_MOV, RD4};            //R<-MD, RD4

        LXI_RP2_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LXI_RP2_IM+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LXI_RP2_IM+2    : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_RP2, T0_DEU_MOV, RD4};          //rp2<-MD, RD4

        //MVIX_RPA_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MVIX_RPA_IM+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_RPA, SA_DST_MA, 2'b00, WR3};                   //MA<-RPA, WR3
        //MVIX_RPA_IM+2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        //MVIW_WA_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, RD3};   //swap MD output order, RD3
        //MVIW_WA_IM+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MVIW_WA_IM+2    : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        //MVIW_WA_IM+3    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        //no operation
        NOP             : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};


        default         : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        IRD             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b1, RD4};   //wait for decoding
    endcase
end

endmodule