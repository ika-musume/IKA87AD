module IKA87AD (
    input   wire            i_EMUCLK,
    input   wire            i_MCUCLK_PCEN
);







///////////////////////////////////////////////////////////
//////  CLOCK AND RESET
////

wire            emuclk = i_EMUCLK;
wire            mcuclk_pcen = i_MCUCLK_PCEN;
wire            cycle_tick, bus_data_latch_tick, mc_read_tick;
wire            mrst_n;



///////////////////////////////////////////////////////////
//////  MICROCODE OUTPUT SIGNALS
////

localparam MCTYPE0 = 2'b00;
localparam MCTYPE1 = 2'b01;

//source 1/destination types
localparam S1_DST_MA = 4'b1010;
localparam S1_DST_PC = 4'b1011;
localparam S1_DST_SP = 4'b1100;

//source 2 types
localparam S2_SP_POP = 5'b01010;
localparam S2_SP_PUSH = 5'b01011;

//bus cycle types
localparam IDLE = 2'b00;
localparam RD4 = 2'b01;
localparam RD3 = 2'b10;
localparam WR3 = 2'b11;

wire    [1:0]   mc_type;
wire    [4:0]   mc_s2;
wire    [3:0]   mc_S1_DST;
wire    [1:0]   mc_next_bus_acc;

wire    [15:0]  reg_wrdata; //ALU output




///////////////////////////////////////////////////////////
//////  TIMING GENERATOR
////

reg     [11:0]  timing_sr_4cyc;
reg     [8:0]   timing_sr_3cyc;
reg             timing_sr_start;

assign  cycle_tick = timing_sr_4cyc[11] | timing_sr_3cyc[8];
assign  bus_data_latch_tick = timing_sr_4cyc[5] | timing_sr_3cyc[5];
assign  mc_read_tick = timing_sr_4cyc[8] | timing_sr_4cyc[11] | timing_sr_3cyc[8];

always @(posedge emuclk) begin
    if(!mrst_n) begin
        timing_sr_4cyc <= 12'h0;
        timing_sr_3cyc <= 9'h0;
        timing_sr_start <= 1'h1;
    end
    else begin
        if(mcuclk_pcen) begin
            if(mc_next_bus_acc == RD4) begin //OPCODE fetch cycle
                //4cyc timing bit entrance
                timing_sr_4cyc[0] <= timing_sr_4cyc[11] | timing_sr_3cyc[8] | timing_sr_start;
                timing_sr_4cyc[11:1] <= timing_sr_4cyc[10:0];

                //3cyc timing bit entrance
                timing_sr_3cyc[0] <= 1'b0;
                timing_sr_3cyc[8:1] <= timing_sr_3cyc[7:0];
            end
            else begin
                //4cyc timing bit entrance
                timing_sr_4cyc[0] <= 1'b0;
                timing_sr_4cyc[11:1] <= timing_sr_4cyc[10:0];

                //3cyc timing bit entrance
                timing_sr_3cyc[0] <= timing_sr_4cyc[11] | timing_sr_3cyc[8];
                timing_sr_3cyc[8:1] <= timing_sr_3cyc[7:0];
            end
        end
    end
end






///////////////////////////////////////////////////////////
//////  MICROCODE ENGINE
////



///////////////////////////////////////////////////////////
//////  MICROCODE ROM
////



/*
    MICROCODE TYPE DESCRIPTION

    1. DOUBLE-OPERAND ALU
    00_X_X_X_XXXXX_XXXX_XX_XX

    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: EOI(end of instruction) marker

    D[12:8] source2, 
        00000: r, OPCODE[]         
        00001: r2, OPCODE[]
        00010: r1, OPCODE[]
        00011: rp2, OPCODE[]
        00100: rp, OPCODE[]
        00101: rp1, OPCODE[]
        00110: sr/sr1, OPCODE[]
        00111: sr2, OPCODE[]
        01000: sr4, OPCODE[0]
        01001: MEM DATA(read data)
        01010: SP_POP
        01011: SP_PUSH
        01100: ALU temp register
        01101: BC(CALB)
        01110:
        01111:
        10000: word
        10001: {V, wa}
        10010: 00001_11bit
        10011: 00000000_10_ta_0
        10100: jdisp1
        10101: jdisp
        10110: -3
        10111: -2
        11000: -1
        11001: 1
        11010: 2
        11011: 3
        11100: *interrupt address
        11101: *address from reg pair(BC, DE, HL), rpa1: OPCODE[1:0], rpa: OPCODE[2], OPCODE[0], rpa2: OPCODE[3:2]
        11110: *rpa addend select
        11111: *rpa3 addend select
    D[7:4] source1, destination register type, decoded by the external circuit:
        0000: r, OPCODE[]         
        0001: r2, OPCODE[]
        0010: r1, OPCODE[]
        0011: rp2, OPCODE[]
        0100: rp, OPCODE[]
        0101: rp1, OPCODE[]
        0110: sr/sr1, OPCODE[]
        0111: sr2, OPCODE[]
        1000: sr3, OPCODE[0]
        1001: MEM DATA(write data)
        1010: MEM ADDR
        1011: PC
        1100: SP
        1101: ALU temp register
        1110: 
        1111: *address from reg pair(BC, DE, HL), rpa1: OPCODE[1:0], rpa: OPCODE[2], OPCODE[0], rpa2: OPCODE[3:2]

    D[3:2] ALU operation type:
        00: bypass(source2 -> source1)
        01: add
        10: ALU operation(field type 0) - OPCODE[6:3]
        11: ALU operation(field type 1) - OPCODE[6:4], OPCODE[0] (single byte inst) 
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read

    *automatically decoded by external logic


    2. SINGLE-OPERAND ALU
    01_X_X_X_X_X

    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: EOI(end of instruction) marker


    3. BOOKKEEPING
    10_X_X_X_X_X

    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: EOI(end of instruction) marker
    D[12]: record interrupt status(1=enabled, 0=disabled)
    D[11]: IACK
    D[10]: set address output as stack pointer
    D[9]: address output decrease mode

*/




///////////////////////////////////////////////////////////
//////  REGISTER FILE
////

//address source selector
localparam PC = 2'b00;
localparam SP = 2'b01;
localparam MA = 2'b10;
localparam MA_RESERVED = 2'b11;
reg     [1:0]   address_source_sel; //00 = PC, 01 = SP, 10 = MA, 11 = reserved
reg     [15:0]  memory_access_address;

//declare PC, SP, MA register
reg             reg_SP_inc_ndec,//PC, SP inc/dec flag
wire            reg_PC_wr = mc_S1_DST == S1_DST_PC && mc_type == MCTYPE0;
wire            reg_SP_wr = mc_S1_DST == S1_DST_SP && mc_type == MCTYPE0;
wire            reg_MA_wr = mc_S1_DST == S1_DST_MA && mc_type == MCTYPE0;
reg     [15:0]  reg_PC, reg_SP, reg_MA;

//this block defines the LOAD, INC and DEC operations of the PC/SP/MA register
always @(posedge emuclk) begin
    //ADDRESS OUTPUT SOURCE SELECT
    if(!mrst_n) begin
        address_source_sel <= 2'b00;
    end
    else begin
        if(cycle_tick) begin
            if(mc_end_of_instruction) address_source_sel <= 2'b00;
            else begin
                if(reg_PC_wr) address_source_sel <= 2'b00; //select PC
                else if((mc_s2 == S2_SP_POP || mc_s2 == S2_SP_PUSH) && reg_MA_wr) address_source_sel <= 2'b01; //select SP
                else if(reg_MA_wr) address_source_sel <= 2'b10; //select MA
                else address_source_sel <= address_source_sel;
            end
        end
    end

    //INC/DEC FLAG
    if(!mrst_n) begin
        reg_SP_inc_ndec <= 1'b1;
    end
    else begin
        if(cycle_tick) begin
            if(mc_end_of_instruction) reg_SP_inc_ndec <= 1'b1;
            else begin
                if(mc_s2 == S2_SP_PUSH && reg_MA_wr) reg_SP_inc_ndec <= 1'b0; //sub 1 from SP -> set SP auto decrement mode
            end
        end
    end

    //REGISTERS
    if(!mrst_n) begin
        reg_PC <= 16'h0000;
        reg_SP <= 16'h0000;
        reg_MA <= 16'h0000;
    end
    else begin
        if(cycle_tick) begin
            //Program Counter load/auto increment conditions
            if(reg_PC_wr) reg_PC <= reg_wrdata;
            else begin
                if(mc_next_bus_acc == RD4 || mc_next_bus_acc == RD3) begin //if there's a 3cyc/4cyc read access,
                    if(!(address_source_sel == PC) || reg_MA_wr) reg_PC <= reg_PC; //but check if it's a mem acc other than a next instruction fetch
                    else begin //Instruction read cycle? increase PC
                        if(reg_PC == 16'hFFFF) reg_PC <= 16'h0000;
                        else reg_PC <= reg_PC + 16'h0001;
                    end
                end
                else reg_PC <= reg_PC;
            end

            //Stack Pointer load/auto inc/dec conditions
            if(reg_SP_wr) reg_SP <= reg_wrdata;
            else begin
                if(mc_next_bus_acc == RD3 || mc_next_bus_acc == WR3) begin //if there's a 3cyc read/write access,
                    if(address_source_sel == SP) begin //but check if it's a mem acc using the SP value
                        if(reg_SP_inc_ndec) begin
                            if(reg_SP == 16'hFFFF) reg_SP <= 16'h0000;
                            else reg_SP <= reg_SP + 16'h0001;
                        end
                        else begin
                            if(reg_SP == 16'h0000) reg_SP <= 16'hFFFF;
                            else reg_SP <= reg_SP - 16'h0001;
                        end
                    end
                    else reg_SP <= reg_SP;
                end
                else reg_SP <= reg_SP;
            end

            //Memory Address load/auto inc conditions
            if(reg_MA_wr) reg_MA <= reg_wrdata;
            else begin
                if(mc_end_of_instruction) reg_MA <= 16'h0000;
                else begin
                    if(mc_next_bus_acc == RD3 || mc_next_bus_acc == WR3) begin //if there's a 3cyc read/write access,
                        if(address_source_sel == MA) begin
                            if(reg_MA == 16'hFFFF) reg_MA <= 16'h0000;
                            else reg_MA <= reg_MA + 16'h0001;
                        end
                        else reg_MA <= reg_MA;
                    end
                    else reg_MA <= reg_MA;
                end
            end
        end
    end
end

always @(*) begin
    case(address_source_sel)
        PC: memory_access_address = reg_PC;
        SP: memory_access_address = reg_SP;
        MA: memory_access_address = reg_MA;
        MA_RESERVED: memory_access_address = 16'hFFFF;
    endcase
end




///////////////////////////////////////////////////////////
//////  BUS CONTROLLER
////

wire    [1:0]   mc_transaction_type;
reg     [15:0]  mc_transaction_address;









endmodule