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

/*
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
*/




wire    [15:0]  cpu_addr;
wire            cpu_rd_n, cpu_wr_n;
wire    [7:0]   cpu_do;
reg     [7:0]   dbus;

//bootloader section
reg     [7:0]   bootloader[0:4095];
wire    [11:0]  bootloader_addr = cpu_addr[11:0];
wire            bootloader_cs_n = ~(cpu_addr < 16'h1000);
wire            bootloader_rd_n = cpu_rd_n;
reg     [7:0]   bootloader_dout;
initial $readmemh("cchip_bootloader.txt", bootloader);
always @(*) bootloader_dout = bootloader_cs_n || bootloader_rd_n ? 8'hZZ : bootloader[bootloader_addr];

//embedded memory section
reg     [7:0]   ram[0:255];
wire    [7:0]   ram_addr = cpu_addr[7:0];
wire            ram_cs_n = ~(cpu_addr > 16'hFEFF);
wire            ram_rd_n = cpu_rd_n;
wire            ram_wr_n = cpu_wr_n;
reg     [7:0]   ram_dout;
always @(*) begin
    ram_dout = 8'hZZ;

    if(!ram_cs_n) begin
        if(!ram_rd_n) ram_dout = ram[ram_addr];
        else if(!ram_wr_n) ram[ram_addr] = dbus;
    end
end

//cpu write data
always @(*) begin
    dbus = 8'hZZ;

    if(!cpu_wr_n) dbus = cpu_do;
    else if(!cpu_rd_n) begin
        if(!bootloader_cs_n) dbus = bootloader_dout;
        else if(!ram_cs_n) dbus = ram_dout;
        else if(cpu_addr == 16'h1401) dbus = 8'hEE;
    end
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

    .o_A                            (cpu_addr                   ),
    .i_DI                           (dbus                       ),
    .o_DO                           (cpu_do                     ),
    .o_PD_DO_OE                     (                           ),
    .o_DO_OE                        (                           ),

    .o_REG_MM                       (                           ),

    .i_NMI_n                        (NMI_n                      ),
    .i_INT1                         (INT1                       ),
    .i_INT2_n                       (INT2_n                     ),

    .i_PA_I                         (8'h69                      ),
    .o_PA_O                         (                           ),
    .o_PA_OE                        (                           ),

    .i_PB_I                         (8'h74                      ),
    .o_PB_O                         (                           ),
    .o_PB_OE                        (                           ),

    .i_PC_I                         (8'hAC                      ),
    .o_PC_O                         (                           ),
    .o_PC_OE                        (                           ),

    .i_PD_I                         (8'hC8                      ),
    .o_PD_O                         (                           ),
    .o_PD_OE                        (                           ),

    .i_PF_I                         (8'hEE                      ),
    .o_PF_O                         (                           ),
    .o_PF_OE                        (                           ),

    .o_ADC_CH                       (                           ),
    .i_ADC_DATA                     (8'hEF                      ),
    .o_ADC_RD_n                     (                           )
);

/*
initial begin
    #350 NMI_n = 1'b0;
    #400 NMI_n = 1'b1;
    #800 INT1 = 1'b1; INT2_n = 1'b0;
    #400 INT1 = 1'b0; INT2_n = 1'b1;
end
*/

/*
    tri-state bus input emulation guide:

    use IKA87AD_tsio #(8)::port_input(wire to "o_PA_O", input data, wire to "o_PA_OE")
*/

endmodule