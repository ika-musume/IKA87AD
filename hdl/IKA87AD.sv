module IKA87AD (
    input   wire            i_EMUCLK,
    input   wire            i_MCUCLK_PCEN,
    input   wire            i_RESET_n,

    output  wire            o_ALE,
    output  wire            o_RD_n,
    output  wire            o_WR_n,

    input   wire    [7:0]   i_PD_I,
    output  wire    [7:0]   o_PD_O,
    output  wire    [7:0]   o_PD_DIR

);







///////////////////////////////////////////////////////////
//////  CLOCK AND RESET
////

wire            emuclk = i_EMUCLK;
wire            mcuclk_pcen = i_MCUCLK_PCEN;
wire            mrst_n = i_RESET_n;



///////////////////////////////////////////////////////////
//////  MICROCODE OUTPUT SIGNALS
////





//bus cycle types
localparam IDLE = 2'b00;
localparam RD4 = 2'b01;
localparam RD3 = 2'b10;
localparam WR3 = 2'b11;

//microcode types
localparam MCTYPE0 = 2'b00;
localparam MCTYPE1 = 2'b01;
wire    [1:0]   mc_type;
wire            mc_end_of_instruction = mc_next_bus_acc == RD4;

//microcode type 0 fields
//source 1/destination types
localparam S1_DST_MD = 4'b1001;
localparam S1_DST_MA = 4'b1010;
localparam S1_DST_PC = 4'b1011;
localparam S1_DST_SP = 4'b1100;
//source 2 types
localparam S2_SP_POP = 5'b01010;
localparam S2_SP_PUSH = 5'b01011;
wire    [4:0]   mc_s2;
wire    [3:0]   mc_s1_dst;
wire    [1:0]   mc_next_bus_acc;

//microcode type 2 fields
wire    [10:0]  mc_bookkeeping;


wire    [15:0]  alu_wrdata; //ALU output
wire    [15:0]  alu_ma_wrdata; //ALU output for the memory address register


///////////////////////////////////////////////////////////
//////  REGISTERS
////

reg     [15:0]  reg_PC, reg_SP, reg_MA;



///////////////////////////////////////////////////////////
//////  TIMING GENERATOR
////

reg     [11:0]  timing_sr;
reg     [1:0]   current_bus_acc;

wire    opcode_tick = timing_sr[11] & current_bus_acc == RD4;
wire    rw_tick = timing_sr[8] & current_bus_acc != RD4;
wire    cycle_tick = opcode_tick | rw_tick;

wire    mc_read_tick = timing_sr[8] | timing_sr[11];

wire    opcode_inlatch_tick = timing_sr[6] & current_bus_acc == RD4;
wire    md_inlatch_tick = timing_sr[6] & current_bus_acc == RD3;
wire    full_opcode_inlatch_tick_debug = timing_sr[6] & (current_bus_acc == RD4 | current_bus_acc == RD3);

always @(posedge emuclk) begin
    if(!mrst_n) begin
        timing_sr <= 12'b000_000_000_001;
        current_bus_acc <= RD4;
    end
    else begin
        if(mcuclk_pcen) begin
            if(current_bus_acc == RD4) begin
                if(timing_sr[11]) begin
                    current_bus_acc <= mc_next_bus_acc;
                    timing_sr[0] <= timing_sr[11];
                    timing_sr[11:1] <= timing_sr[10:0];
                end
                else begin
                    timing_sr[0] <= timing_sr[11];
                    timing_sr[11:1] <= timing_sr[10:0];
                end
            end
            else begin
                if(timing_sr[8]) begin
                    current_bus_acc <= mc_next_bus_acc;
                    timing_sr[0] <= timing_sr[8];
                    timing_sr[8:1] <= timing_sr[7:0];
                    timing_sr[9] <= 1'b0;
                    timing_sr[11:10] <= timing_sr[10:9];
                end
                else begin
                    timing_sr[0] <= timing_sr[8];
                    timing_sr[11:1] <= timing_sr[10:0];
                end
            end
        end
    end
end



///////////////////////////////////////////////////////////
//////  OPCODE DECODER
////

reg     [7:0]   reg_OPCODE;
reg     [7:0]   reg_FULL_OPCODE_debug[0:3];



///////////////////////////////////////////////////////////
//////  MICROCODE ENGINE
////



///////////////////////////////////////////////////////////
//////  MICROCODE ROM
////

/*
    MICROCODE TYPE DESCRIPTION

    1. ALU-REGISTER 1
    00_X_X_X_XXXXX_XXXX_XX_XX

    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13:9] source2
        00000: (b) r, OPCODE[]         
        00001: (b) r2, OPCODE[]
        00010: (b) r1, OPCODE[]
        00011: (w) rp2, OPCODE[]
        00100: (w) rp, OPCODE[]
        00101: (w) rp1, OPCODE[]
        00110: (b) sr/sr1, OPCODE[]
        00111: () sr2, OPCODE[]
        01000: () sr4, OPCODE[0]
        01001: (b) MD_high_byte
        01010: (w) MD_word
        01011: (w) SP_PUSH_PC, transfer SP-1, auto decrement
        01100: (w) SP_POP_PC
        01101: (w) SP_PUSH_DATA, transfer SP-1
        01110: (w) SP_POP_DATA(RETI)
        01111: (b) A
        10000: (w) EA
        10001: (w) ADDR_SOFTI
        10010: (w) ADDR_V_WA 
        10011: (w) ADDR_IM    
        10100: (w) ADDR_DIR  
        10101: (w) ADDR_REL_S
        10110: (w) ADDR_REL_L
        10111: (w) -1
        11000: (w) 1
        11001: (w) 2
        11010: (w) ALU temp register
        11011: (w) *ADDR_INT, interrupt address
        11100: (w) *ADDR_RPA_SINGLE, +, --
        11101: (w) *ADDR_RPA_DOUBLE, ++, --
        11110: (w) *ADDR_RPA_BASE
        11111: () *ADDR_RPA_OFFSET, rpa/rpa3 addend select
    D[8:3] source1, destination register type, decoded by the external circuit, :
        00000: (b) r, OPCODE[]         
        00001: (b) r2, OPCODE[]
        00010: (b) r1, OPCODE[]
        00011: (w) rp2, OPCODE[]
        00100: (w) rp, OPCODE[]
        00101: (w) rp1, OPCODE[]
        00110: () sr/sr1, OPCODE[]
        00111: () sr2, OPCODE[]
        01000: () sr3, OPCODE[0]
        01001: (b) MD_low_byte
        01010: (w) MD_word
        01011: (w) MA
        01100: (w) PC
        01101: (w) SP
        01110: (b) A 
        01111: (w) EA
        10000: (b) C 
        10001:
        10010:
        10011:
        10100:
        10101:
        10110:
        10111:
        11000:
        11001:
        11010:
        11011: 
        11100: (w)ALU temp register
        11101: (w)*ADDR_RPA_SINGLE, +, --     
        11110: (w)*ADDR_RPA_DOUBLE, ++, --
        11111: ()*ADDR_RPA_OFFSET      
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


    2. ALU-REGISTER 2
    01_X_X_XXXX_XXXX_?_XXX_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13:10] source2
    0000: HL
    0001: A
    0010: EA
    0011: BC(CALB)
    0100: DE
    0101: HL
    0110: MD_high_byte
    0111: MD_word
    1000: PC
    1001: PSW
    1010: DE+(BLOCK)
    1011: HL+(BLOCK)
    1100: 
    1101: 
    1110:
    1111:
    D[9:6] source1
    0000: r2
    0001: A
    0010: EA
    0011: MD_low_byte
    0100: MD_word
    0101: MA
    0110: PSW
    0111:
    1000:
    1001:
    1010:
    1011:
    1100:
    1101:
    1110:
    1111:
    D[4:2] ALU operation type:
    000: bypass
    001: NEGA(negate)
    010: DAA(what the fuck is that)
    011: RLD(rotate left digit)
    100: RRD(rotate right digit)
    101: MUL
    110: DIV
    111: shift operation, OPCODE[4], OPCODE[2]
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read


    3. SPECIAL OPERATION
    10_X_X_X_X_X_X_X_X_?_?_?_XXX_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: EI/DIO(1=enabled, 0=disabled)
    D[12]: CARRY
    D[11]: EXX
    D[10]: EXA
    D[9]: EXH
    D[8]: BIT
    D[7]: 
    D[6]: 
    D[5]: 
    D[4:2]: CPU control
        000: SK
        001: SKN
        010: SKIT
        011: SKNIT
        100: HLT
        101: STOP
        110: 
        111:
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read

    4. CONDITIONAL OPERATION
    11_X_X_X_XXXX_X_X_X_X_X_?_?_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: nop
    D[12:9]: nop cycles 0=>1cycle, 15=16cycles
    D[8]: conditional PC write(BLOCK)
    D[7]: conditional read(rpa+byte or register)
    D[6]: conditional branch on ALU type
    D[5:4]: branch+ steps 0=>+2 3=>+5
    D[3]: swap MA input order
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read
*/


///////////////////////////////////////////////////////////
//////  MICROCODE OUTPUT DECODER
////


wire            reg_PC_wr = mc_type == MCTYPE0 && mc_s1_dst == S1_DST_PC;
wire            reg_SP_wr = mc_type == MCTYPE0 && mc_s1_dst == S1_DST_SP;
wire            reg_MA_wr = mc_type == MCTYPE0 && mc_s1_dst == S1_DST_MA;
wire            reg_MA_swap_input_order = mc_type == MCTYPE2 && mc_bookkeeping[4];
wire            

wire            reg_SP_pop = mc_type == MCTYPE0 && mc_s2 == S2_SP_POP;
wire            reg_SP_push = mc_type == MCTYPE0 && mc_s2 == S2_SP_PUSH;



///////////////////////////////////////////////////////////
//////  ALU
////






///////////////////////////////////////////////////////////
//////  REGISTER FILE
////

//
//  PC, SP, MA registers with auto increment/decrement feature
//

//address source selector
localparam PC = 2'b00;
localparam SP = 2'b01;
localparam MA = 2'b10;
localparam MA_RESERVED = 2'b11;
reg     [1:0]   address_source_sel; //00 = PC, 01 = SP, 10 = MA, 11 = reserved
reg     [15:0]  memory_access_address;

//SP auto inc/dec flag
reg             reg_SP_inc_ndec; //SP inc/dec flag

//this block defines the LOAD, INC and DEC operations of the PC/SP/MA register
always @(posedge emuclk) begin
    //ADDRESS OUTPUT SOURCE SELECT
    if(!mrst_n) begin
        address_source_sel <= PC;
    end
    else begin
        if(cycle_tick) begin
            if(mc_end_of_instruction) address_source_sel <= PC;
            else begin
                if(reg_PC_wr) address_source_sel <= PC; //select PC
                else if((reg_SP_pop || reg_SP_push) && reg_MA_wr) address_source_sel <= 2'b01; //select SP
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
                if(reg_SP_push && reg_MA_wr) reg_SP_inc_ndec <= 1'b0; //sub 1 from SP -> set SP auto decrement mode
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
            if(reg_PC_wr) reg_PC <= alu_wrdata;
            else begin
                if(mc_next_bus_acc == RD4 || mc_next_bus_acc == RD3) begin //if there's a 3cyc/4cyc read access,
                    if(address_source_sel != PC || reg_MA_wr) begin //but check if it's a mem acc other than a next instruction fetch
                        reg_PC <= reg_PC;
                    end
                    else begin //Instruction read cycle? increase PC
                        reg_PC <= reg_PC == 16'hFFFF ? 16'h0000 : reg_PC + 16'h0001;
                    end
                end
                else reg_PC <= reg_PC;
            end

            //Stack Pointer load/auto inc/dec conditions
            if(reg_SP_wr) reg_SP <= alu_wrdata;
            else begin
                if(mc_next_bus_acc == RD3 || mc_next_bus_acc == WR3) begin //if there's a 3cyc read/write access,
                    if(address_source_sel == SP) begin //but check if it's a mem acc using the SP value
                        if(reg_SP_inc_ndec) reg_SP <= reg_SP == 16'hFFFF ? 16'h0000 : reg_SP + 16'h0001;
                        else reg_SP <= reg_SP == 16'h0000 ? 16'hFFFF : reg_SP - 16'h0001;
                    end
                    else reg_SP <= reg_SP;
                end
                else reg_SP <= reg_SP;
            end

            //Memory Address load/auto inc conditions
            if(reg_MA_wr) reg_MA <= alu_ma_wrdata;
            else begin
                if(opcode_tick) reg_MA <= reg_PC; ///4사이클 읽기 끝나면 새 PC로 MA업데이트하는 코드 작성하기
                else begin
                    if(mc_next_bus_acc == RD3 || mc_next_bus_acc == WR3) begin //if there's a 3cyc read/write access,
                        if(address_source_sel == MA) reg_MA <= reg_MA == 16'hFFFF ? 16'h0000 : reg_MA + 16'h0001;
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
        RPA2: memory_access_address = 16'hFFFF;
    endcase
end


//
//  General purpose registers
//

reg_SP <= reg_SP == 16'hFFFF ? 16'h0000 : reg_SP + 16'h0001;


///////////////////////////////////////////////////////////
//////  BUS CONTROLLER
////

//multiplexed addr/data selector
reg             addr_data_sel;
always @(posedge emuclk) begin
    if(!mrst_n) addr_data_sel <= 1'b0; //reset
    else begin
        if(cycle_tick) addr_data_sel <= 1'b0; //reset
        else begin
            if(current_bus_acc != IDLE) if(timing_sr[2]) addr_data_sel <= 1'b1;
        end
    end
end


//memory data byte hi/lo sel
reg             md_out_byte_sel, md_in_byte_sel;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        md_out_byte_sel <= 1'b0;
        md_in_byte_sel <= 1'b0;
    end
    else begin
        if(cycle_tick) begin
            if(mc_end_of_instruction) begin
                md_out_byte_sel <= 1'b0;
                md_in_byte_sel <= 1'b0;
            end
            else begin
                //swap output data order(to HI->LO) when the current instruction is PUSH
                if(reg_SP_push && reg_MA_wr) md_out_byte_sel <= 1'b1;
                else begin
                    if(current_bus_acc == WR3) md_out_byte_sel <= ~md_out_byte_sel;
                end

                //swap input data order(to HI->LO) when the bookkeeping bit is hot, this is for WA, BYTE instruction
                if(reg_MA_swap_input_order) md_in_byte_sel <= 1'b1;
                else begin
                    if(current_bus_acc == RD3) md_in_byte_sel <= ~md_in_byte_sel;
                end
            end
        end
    end
end


//OPCODE/memory data IO
wire            reg_MD_wr = (mc_s1_dst == S1_DST_MD && mc_type == MCTYPE0);
reg     [15:0]  reg_MD; //byte [15:8], word[15:0]

always @(posedge emuclk) begin
    if(!mrst_n) begin
        reg_MD <= 16'h0000;
        reg_OPCODE <= 8'h00;
    end
    else begin
        if(mcuclk_pcen) begin
            //Memory Data register load
            if(cycle_tick) if(reg_MD_wr) reg_MD <= alu_wrdata;
            else if(md_inlatch_tick) begin
                if(md_in_byte_sel) reg_MD[15:8] <= i_PD_I;
                else reg_MD[7:0] <= i_PD_I;
            end

            //Opcode register load
            if(opcode_inlatch_tick) reg_OPCODE <= i_PD_I;
        
            //Full opcode register for the disassembler
            if(full_opcode_inlatch_tick_debug) begin
                reg_FULL_OPCODE_debug[0] <= i_PD_I;
                reg_FULL_OPCODE_debug[1] <= reg_FULL_OPCODE_debug[0];
                reg_FULL_OPCODE_debug[2] <= reg_FULL_OPCODE_debug[1];
                reg_FULL_OPCODE_debug[3] <= reg_FULL_OPCODE_debug[2];
            end
        end
    end
end


//address high, multiplexed address low/byte data output
wire    [7:0]   md_out_byte_data = md_out_byte_sel == 1'b1 ? reg_MD[15:8] : reg_MD[7:0]; //MD출력 시 CALL에는 PC를 곧바로 출력, strax rpa시에는 EA를 출력
wire    [7:0]   addr_hi_out = memory_access_address[15:8];
wire    [7:0]   addr_lo_data_out = addr_data_sel ? md_out_byte_data : memory_access_address[7:0];


//ALE, /RD, /WR
reg             ale_out, rd_out, wr_out;
assign o_ALE = ale_out;
assign o_RD_n = ~rd_out;
assign o_WR_n = ~wr_out;

always @(posedge emuclk) begin
    if(!mrst_n) begin
        ale_out <= 1'b0;
        rd_out <= 1'b0;
        wr_out <= 1'b0;
    end
    else begin
        if(cycle_tick) begin
            if(mc_next_bus_acc != IDLE) ale_out <= 1'b1;
        end
        else begin
            //ALE off
            if(timing_sr[1]) ale_out <= 1'b0;

            //RD control
            if(current_bus_acc == RD4) begin
                if(timing_sr[2]) rd_out <= 1'b1;
                else if(timing_sr[8]) rd_out <= 1'b0;
            end
            else if(current_bus_acc == RD3) begin
                if(timing_sr[2]) rd_out <= 1'b1;
                else if(timing_sr[6]) rd_out <= 1'b0;
            end
            else rd_out <= 1'b0;

            //WR control
            if(current_bus_acc == WR3) begin
                if(timing_sr[2]) wr_out <= 1'b1;
                else if(timing_sr[6]) wr_out <= 1'b0;
            end
            else wr_out <= 1'b0;
        end
    end
end




endmodule