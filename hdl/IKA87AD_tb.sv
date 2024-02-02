`timescale 10ps/10ps
module IKA87AD_tb;

//test inputs
reg             EMUCLK = 1'b1;
reg             RST_n = 1'b1;

//generate clock
always #1 EMUCLK = ~EMUCLK;
reg     [1:0]   prescaler = 2'd0;
wire            PCEN = prescaler == 2'd3;
always @(posedge EMUCLK) begin
    prescaler <= prescaler + 2'd1;
end

//reset
initial begin
    #30 RST_n = 1'b0;
    #130 RST_n = 1'b1;
end


reg     [7:0]   mem[0:511];
wire    [15:0]  mem_addr;
reg     [7:0]   mem_data;

initial begin
    $readmemh("IKA87AD_testmem.txt", mem);
end

always @(*) begin
    mem_data = mem[mem_addr[8:0]];
end


wire            cpu_rd_n;
wire            cpu_wr_n;
wire    [7:0]   cpu_dbus = cpu_rd_n ? 8'hZZ : mem_data;


IKA87AD u_dut (
    .i_EMUCLK                       (EMUCLK                     ),
    .i_MCUCLK_PCEN                  (PCEN                       ),

    .i_RESET_n                      (RST_n                      ),
    .i_STOP_n                       (                           ),

    .o_ALE                          (                           ),
    .o_RD_n                         (cpu_rd_n                   ),
    .o_WR_n                         (cpu_wr_n                   ),

    .i_NMI_n                        (                           ),
    .i_INT1                         (                           ),

    .i_PC_I                         (                           ),
    .o_PC_O                         (                           ),
    .o_PC_DIR                       (                           ),

    .i_PD_I                         (cpu_dbus                   ),
    .o_PD_O                         (                           ),
    .o_PD_DIR                       (                           ),

    .o_FULL_ADDRESS_DEBUG           (mem_addr                   ),
    .o_OUTPUT_DATA_DEBUG            (                           )
);




endmodule