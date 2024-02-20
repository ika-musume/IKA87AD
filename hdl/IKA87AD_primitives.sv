module IKA87AD_iflag (
    input   wire            i_MRST_n,
    input   wire            i_EMUCLK,
    input   wire            i_TICK,

    input   wire            i_IRQ,
    input   wire            i_IS_ENABLED,
    input   wire    [4:0]   i_IRQ_CODE_UNIQUE,
    input   wire    [4:0]   i_IRQ_CODE_TO_BE_ACKD,
    input   wire            i_MULTI_IRQ_ENABLED,
    input   wire            i_MANUAL_ACK,
    input   wire            i_AUTO_ACK,

    output  reg             o_IFLAGREG
);

always @(posedge i_EMUCLK) begin
    if(!i_MRST_n) o_IFLAGREG <= 1'b0;
    else begin if(i_TICK) begin
        if(i_IRQ) o_IFLAGREG <= 1'b1;
        else begin
            if(i_IS_ENABLED) begin
                if(i_MULTI_IRQ_ENABLED) begin
                    if(i_MANUAL_ACK && i_IRQ_CODE_TO_BE_ACKD == i_IRQ_CODE_UNIQUE) o_IFLAGREG <= 1'b0; //manual ack
                end
                else begin
                    if(i_AUTO_ACK) o_IFLAGREG <= 1'b0; //auto ack
                end
            end
        end
    end end
end

endmodule