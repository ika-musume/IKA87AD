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
        /*
            8-bit data transfer instructions
        */
        //MOV_R1_A        : mc <= {MCTYPE0, 1'b0, 1'b1, SB_A, SA_DST_R1, 2'b00, RD4};                     //r1<-A, RD4
        //MOV_A_R1        : mc <= {MCTYPE0, 1'b0, 1'b1, SB_R1, SA_DST_A, 2'b00, RD4};                     //A<-r1, RD4
        //MOV_SR_A        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_SR_A+1      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        //MOV_SR_A+2      : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_SR_SR1, 2'b00, RD4};                 //sr<-A, RD4
        //MOV_A_SR1       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_A_SR1+1     : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        //MOV_A_SR1+2     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SR_SR1, SA_DST_A, 2'b00, RD4};                 //sr<-A, RD4
        //MOV_R_MEM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_R_MEM+1     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_R_MEM+2     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MD, SA_DST_MA, 2'b00, RD3};                    //MA<-MD, RD3
        //MOV_R_MEM+3     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_R, 2'b00, RD4};                     //r<-MD, RD4
        //MOV_MEM_R       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_R, SA_DST_MDL, 2'b00, RD3};                    //MD<-R
        //MOV_MEM_R+1     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_MEM_R+2     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MDI, SA_DST_MA, 2'b00, WR3};                   //MA<-MDI, WR3
        //MOV_MEM_R+3     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        MVI_R_IM        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MVI_R_IM+1      : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_R, T0_DEU_MOV, RD4};            //R<-MD, RD4
        // * MVI_SR2_IM is included in ALUI_SR2_IM instruction
        //MVIW_WA_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, RD3};   //swap MD output order, RD3
        //MVIW_WA_IM+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MVIW_WA_IM+2    : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        //MVIW_WA_IM+3    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //MVIX_RPA_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MVIX_RPA_IM+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_RPA, SA_DST_MA, 2'b00, WR3};                   //MA<-RPA, WR3
        //MVIX_RPA_IM+2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //LDAW            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //LDAW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        //LDAW+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MDH, SA_DST_A, 2'b00, RD4};                    //A<-MD, RD4
        //STAW            : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_MDL, 2'b00, RD3};                    //MD<-A
        //STAW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        //STAW+2          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //LDAX_A_RPA      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};                 //MA<-RPA, WR3
        //LDAX_A_RPA+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4};                     //A<-MD, RD4
        //STAX_RPA_A      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, WR3};                 //MA<-RPA, WR3 //A will be loaded into MD automatically
        //STAX_RPA_A+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //LDAX_A_RPA2     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        //LDAX_A_RPA2+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        //LDAX_A_RPA2+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, RD3};                  //MA<-MA+RPA2, RD3
        //LDAX_A_RPA2+3   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4};                     //A<-MD
        //STAX_RPA2_A     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        //STAX_RPA2_A+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        //STAX_RPA2_A+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, WR3};                  //MA<-MA+RPA2, WR3
        //STAX_RPA2_A+3   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //EXX             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b01, 1'b0, 3'b000, RD4};   //exx mod
        //EXA             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b10, 1'b0, 3'b000, RD4};   //exa mod
        //EXH             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b11, 1'b0, 3'b000, RD4};   //exh mod
        //BLOCK           : mc <= {MCTYPE1, 1'b0, 1'b1, SD_HL, SC_DST_MA, 4'b1001, RD3};                  //MA<-pop HL, RD3
        //BLOCK+1         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_DE, SC_DST_MA, 4'b1001, WR3};                  //MA<-pop DE, WR3
        //BLOCK+2         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_C, 2'b01, IDLE};                  //C<-C-1(dec), IDLE
        //BLOCK+3         : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b1, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //conditional PC decrement, RD4


        /*
            16-bit data transfer instructions
        */
        LXI_RP2_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LXI_RP2_IM+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LXI_RP2_IM+2    : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_RP2, T0_DEU_MOV, RD4};          //rp2<-MD, RD4
        //TABLE           : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD2, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+A, IDLE
        //TABLE+1         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_MA, 2'b01, RD3};                     //MA<-MA+2, IDLE
        //TABLE+2         : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //TABLE+3         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_MD, SC_DST_BC, 4'b0000, RD4};                  //BC<-MD, RD4

        
        /*
            8-bit register-accumulator arithmetic instructions
        */

        /*
            8-bit memory-accumulator arithmetic instructions
        */
        
        /*
            8-bit immediate-register arithmetic instructions
        */

        /*
            8-bit working register arithmetic instructions
        */

        /*
            16-bit register-accumulator arithmetic instructions
        */

        /*
            16-bit multiplication/division instructions
        */
        MUL             : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_R2, T0_DST_EA, T0_DEU_MUL, IDLE};          //EA<-A*r2, IDLE
        MUL+1           : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        DIV             : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_R2, T0_DST_EA, T0_DEU_DIV, IDLE};          //EA<-EA/r2, IDLE
        DIV+1           : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_AUX, T0_DST_R2, T0_DEU_MOV, RD4};          //r2<-ALU temp, RD4

        //no operation
        NOP             : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        default         : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        IRD             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b1, RD4};   //wait for decoding
    endcase
end

endmodule