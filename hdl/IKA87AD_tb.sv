`timescale 10ps/10ps
module IKA87AD_tb;

//test inputs
reg             EMUCLK = 1'b1;
reg             RST_n = 1'b1;
reg             STOP_n = 1'b1;
reg             NMI_n = 1'b1;
reg             INT1 = 1'b0;
reg             INT2_n = 1'b1;

//generate clock
always #1 EMUCLK = ~EMUCLK;
reg     [1:0]   prescaler = 2'd0;
wire            PCEN = prescaler == 2'd3;
always @(posedge EMUCLK) begin
    prescaler <= prescaler + 2'd1;
end

//reset
initial begin
    #10  RST_n = 1'b0;
    #110 RST_n = 1'b1;
end


reg     [7:0]   mem[0:511];
wire    [15:0]  mem_addr;

wire            cpu_rd_n, cpu_wr_n;
wire    [7:0]   cpu_do;
reg     [7:0]   mem_do;
reg     [7:0]   dbus;

initial begin
    $readmemh("IKA87AD_testmem.txt", mem);
end

always @(*) begin
    if(!cpu_wr_n) begin
        mem_do = 8'hZZ;
        mem[mem_addr[8:0]] = dbus;
    end
    else begin
        mem_do = mem[mem_addr[8:0]];
    end

    if(!cpu_wr_n) dbus = cpu_do;
    else if(!cpu_rd_n) dbus = mem_do;
    else dbus = 8'hZZ;
end


IKA87AD u_dut (
    .i_EMUCLK                       (EMUCLK                     ),
    .i_MCUCLK_PCEN                  (PCEN                       ),

    .i_RESET_n                      (RST_n                      ),
    .i_STOP_n                       (STOP_n                     ),

    .o_M1_n                         (                           ),
    .o_IO_n                         (                           ),

    .o_ALE                          (                           ),
    .o_RD_n                         (cpu_rd_n                   ),
    .o_WR_n                         (cpu_wr_n                   ),
    .o_ALE_OE                       (                           ),
    .o_RD_n_OE                      (                           ),
    .o_WR_n_OE                      (                           ),

    .o_A                            (mem_addr                   ),
    .i_DI                           (dbus                       ),
    .o_DO                           (cpu_do                     ),
    .o_PD_DO_OE                     (                           ),
    .o_DO_OE                        (                           ),

    .o_MEMSTRUCT                    (                           ),

    .i_NMI_n                        (NMI_n                      ),
    .i_INT1                         (INT1                       ),
    .i_INT2_n                       (INT2_n                     ),

    .i_PA_I                         (                           ),
    .o_PA_O                         (                           ),
    .o_PA_OE                        (                           ),

    .i_PB_I                         (                           ),
    .o_PB_O                         (                           ),
    .o_PB_OE                        (                           ),

    .i_PC_I                         (                           ),
    .o_PC_O                         (                           ),
    .o_PC_OE                        (                           ),

    .i_PD_I                         (                           ),
    .o_PD_O                         (                           ),
    .o_PD_OE                        (                           ),

    .i_PF_I                         (                           ),
    .o_PF_O                         (                           ),
    .o_PF_OE                        (                           )
);


initial begin
    #350 NMI_n = 1'b0;
    #400 NMI_n = 1'b1;
    #800 INT1 = 1'b1; INT2_n = 1'b0;
    #400 INT1 = 1'b0; INT2_n = 1'b1;
end


endmodule