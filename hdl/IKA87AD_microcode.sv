`include "IKA87AD_mnemonics.sv"

module IKA87AD_microcode (
    input   wire                i_CLK,
    input   wire                i_MCROM_READ_TICK,
    input   wire    [7:0]       i_MCROM_ADDR,
    output  wire    [17:0]      o_MCROM_DATA
);

reg     [17:0]  mc;
assign  o_MCROM_DATA = mc;

always @(posedge i_CLK) if(i_MCROM_READ_TICK) begin
    case(i_MCROM_ADDR)
        //                       MCTYPE   FLAG  SKIP
        MVI_R_BYTE      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3}; //nop, RD3
        MVI_R_BYTE+1    : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_R, 2'b00, RD4}; //r<-MD, RD4

        //rpa version of STAX
        STAX_RPA        : mc <= {MCTYPE1, 1'b1, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, WR3}; //MA<-RPA, WR3 //A will be loaded into MD automatically
        STAX_RPA+1      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4

        LDAX_RPA        : mc <= {MCTYPE1, 1'b1, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3}; //MA<-RPA, WR3
        LDAX_RPA+1      : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4}; //A<-MD, RD4

        LXI             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3}; //nop*2, RD3
        LXI+1           : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_RP2, 2'b00, RD4}; //rp2<-MD, RD4

        INX_RP2         : mc <= {MCTYPE0, 1'b1, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE}; //MA<-MA+1, IDLE(stop PC increment)
        INX_RP2+1       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_RP2, 2'b01, RD4}; //RP2<-RP2+1

        INX_EA          : mc <= {MCTYPE0, 1'b1, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE}; //MA<-MA+1, IDLE(stop PC increment)
        INX_EA+1        : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_EA, 2'b01, RD4}; //EA<-EA+1

        DCX_RP2         : mc <= {MCTYPE0, 1'b1, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE}; //MA<-MA+1, IDLE(stop PC increment)
        DCX_RP2+1       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_RP2, 2'b01, RD4}; //RP2<-RP2-1

        DCX_EA          : mc <= {MCTYPE0, 1'b1, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE}; //MA<-MA+1, IDLE(stop PC increment)
        DCX_EA+1        : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_EA, 2'b01, RD4}; //EA<-EA-1

        JMP             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3}; //nop*2, RD3
        JMP+1           : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_PC, 2'b00, RD4}; //PC<-MD

        JR              : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE}; //nop*3, IDLE
        JR+1            : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADDR_REL_S, SA_DST_PC, 2'b01, RD4}; //PC<-PC+jdisp1(short)

        //addx, adcx, addncx, subx, sbbx, subncx, anax, orax, xrax, gtax, ltax, neax, eqax, onax, offax
        ALUX_RPA        : mc <= {MCTYPE1, 1'b1, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};
        ALUX_RPA+1      : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b10, RD4};


        NOP             : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4

        
        ID              : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //wait for decoding
        default         : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4
    endcase
end

endmodule