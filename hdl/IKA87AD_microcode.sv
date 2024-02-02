`include "IKA87AD_mnemonics.sv"

module IKA87AD_microcode (
    input   wire                i_CLK,
    input   wire                i_MCROM_READ_TICK,
    input   wire    [7:0]       i_MCROM_ADDR,
    output  reg     [17:0]      o_MCROM_DATA
);

reg     [17:0]  mc;
assign  o_MCROM_DATA = mc;

always @(posedge i_CLK) if(i_MCROM_READ_TICK) begin
    case(i_MCROM_ADDR)
        MVI_R_BYTE      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3}; //nop, RD3
        MVI_R_BYTE+1    : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_R, 2'b00, RD4}; //r<-MD, RD4

        STAX_RPA        : mc <= {MCTYPE1, 1'b0, 1'b0, SD_RPA, SC_DST_MA, 4'b1010, WR3}; //MA<-RPA, WR3 //A is automatically loaded to MD
        STAX_RPA+1      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4

        LDAX_RPA        : mc <= {MCTYPE1, 1'b0, 1'b0, SD_RPA, SC_DST_MA, 4'b1010, RD3}; //MA<-RPA, WR3
        LDAX_RPA+1      : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_A, 2'b00, RD4}; //A<-MD, RD4

        NOP             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4
        default         : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4
    endcase
end

endmodule