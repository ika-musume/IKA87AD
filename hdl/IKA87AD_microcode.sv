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
        TEST        : mc <= {16'b11_0_0_10011_0_0_000_0_0, RD3};
        TEST+1      : mc <= {16'b11_0_0_10011_0_0_000_0_0, RD4};

        NOP         : mc <= {16'b11_0_0_10000_0_0_000_0_0, RD4};
        default     : mc <= {16'b11_0_0_10000_0_0_000_0_0, RD4};
    endcase
end

endmodule