module IKA87AD (
    //clock
    input   wire            i_EMUCLK,
    input   wire            i_MCUCLK_PCEN,

    //system control
    input   wire            i_RESET_n,
    input   wire            i_STOP_n,

    //R/W control
    output  wire            o_ALE,
    output  wire            o_RD_n,
    output  wire            o_WR_n,

    //interrupt control
    input   wire            i_NMI_n,
    input   wire            i_INT1,

    //port C I/O and DIRECTION
    input   wire    [7:0]   i_PC_I,
    output  wire    [7:0]   o_PC_O,
    output  wire    [7:0]   o_PC_DIR,

    //port D I/O and DIRECTION
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
//////  OPCODE DECODER
////

reg     [2:0]   opcode_page;
reg     [7:0]   reg_OPCODE;
reg     [7:0]   reg_FULL_OPCODE_debug[0:3];






///////////////////////////////////////////////////////////
//////  MICROCODE OUTPUT SIGNALS
////

wire    [17:0]  mc_output;

//bus cycle types
localparam IDLE = 2'b00;
localparam RD4 = 2'b01;
localparam RD3 = 2'b10;
localparam WR3 = 2'b11;

//microcode types
localparam MCTYPE0 = 2'd0;
localparam MCTYPE1 = 2'd1;
localparam MCTYPE2 = 2'd2;
localparam MCTYPE3 = 2'd3;
wire    [1:0]   mc_type;

//next bus access; assert RD3 when the operation mode is RPA2/3, DE/HL+byte
wire    [1:0]   mc_next_bus_acc =   (mc_type == MCTYPE3 && mc_output[7]) ? 
                                        (reg_OPCODE[0] & reg_OPCODE[1]) ? 
                                            RD3
                                            : mc_output[1:0]
                                        : mc_output[1:0];
wire            mc_end_of_instruction = mc_next_bus_acc == RD4;

//MICROCODE TYPE 0 FIELDS

//source a/destination types
localparam SA_DST_R      = 5'b00000;
localparam SA_DST_R2     = 5'b00001;
localparam SA_DST_R1     = 5'b00010;
localparam SA_DST_RP2    = 5'b00011;
localparam SA_DST_RP     = 5'b00100;
localparam SA_DST_RP1    = 5'b00101;
localparam SA_DST_SR_SR1 = 5'b00110;
localparam SA_DST_SR2    = 5'b00111;
localparam SA_DST_SR3    = 5'b01000;
localparam SA_DST_MDL    = 5'b01001;
localparam SA_DST_MD     = 5'b01010;
localparam SA_DST_MA     = 5'b01011;
localparam SA_DST_PC     = 5'b01100;
localparam SA_DST_SP     = 5'b01101;
localparam SA_DST_A      = 5'b00000;
localparam SA_DST_EA     = 5'b00000;
localparam SA_DST_C      = 5'b00000;
localparam SA_DST_RPA1   = 5'b11110;
localparam SA_DST_RPA2   = 5'b11111;

//source b types
localparam SB_R          = 5'b00000;
localparam SB_R2         = 5'b00001;
localparam SB_R1         = 5'b00010;
localparam SB_RP2        = 5'b00011;
localparam SB_RP         = 5'b00100;
localparam SB_RP1        = 5'b00101;
localparam SB_SR_SR1     = 5'b00110;
localparam SB_SR2        = 5'b00111;
localparam SB_SR4        = 5'b01000;
localparam SB_MDH        = 5'b01001;
localparam SB_MD         = 5'b01010;
localparam SB_A          = 5'b01110;
localparam SB_EA         = 5'b01111;
localparam SB_ADDR_V_WA  = 5'b10001;
localparam SB_ADDR_TA    = 5'b10010;
localparam SB_ADDR_FA    = 5'b10011;
localparam SB_ADDR_REL_S = 5'b10100;
localparam SB_ADDR_REL_L = 5'b10101;
localparam SB_ADDR_INT   = 5'b10110;
localparam SB_SUB2       = 5'b10111;
localparam SB_SUB1       = 5'b11000;
localparam SB_ZERO       = 5'b11001;
localparam SB_ADD1       = 5'b11010;
localparam SB_ADD2       = 5'b11011;
localparam SB_TEMP       = 5'b11100;
localparam SB_RPA1       = 5'b11101;
localparam SB_RPA2       = 5'b11110;
localparam SB_OFFSET     = 5'b11111;

wire    [4:0]   mc_sb; //microcode type 0, source b
wire    [3:0]   mc_sa_dst; //microcode type 0, source a
wire    [1:0]   mc_t0_alusel;


//MICROCODE TYPE 1 FIELDS

//source c types
localparam SC_DST_R2     = 4'b0000;
localparam SC_DST_A      = 4'b0001;
localparam SC_DST_EA     = 4'b0010;
localparam SC_DST_MDL    = 4'b0011;
localparam SC_DST_MD     = 4'b0100;
localparam SC_DST_MA     = 4'b0101;
localparam SC_DST_PSW    = 4'b0110;
localparam SC_DST_RPA    = 4'b1111;

//source d types
localparam SD_A          = 4'b0000;
localparam SD_EA         = 4'b0001;
localparam SD_BC         = 4'b0010;
localparam SD_DE         = 4'b0011;
localparam SD_HL         = 4'b0100;
localparam SD_MDH        = 4'b0101;
localparam SD_MD         = 4'b0110;
localparam SD_PC         = 4'b0111;
localparam SD_SP         = 4'b1000;
localparam SD_PSW        = 4'b1001;
localparam SD_RPA        = 4'b1111;

wire    [3:0]   mc_sd; //microcode type 1, source d
wire    [3:0]   mc_sc_dst; //microcode type 1, source c
wire    [3:0]   mc_t1_alusel;

//MICROCODE TYPE 2 FIELDS
wire    [15:0]   mc_bookkeeping;

//MICROCODE TYPE 3 FIELDS
wire    [15:0]   mc_conditional;





///////////////////////////////////////////////////////////
//////  TIMING GENERATOR
////

reg     [11:0]  timing_sr;
reg     [1:0]   current_bus_acc;

wire    opcode_tick = timing_sr[11] & current_bus_acc == RD4 & mcuclk_pcen;
wire    rw_tick = timing_sr[8] & current_bus_acc != RD4 & mcuclk_pcen;
wire    cycle_tick = opcode_tick | rw_tick;

wire    mc_read_tick = timing_sr[8] | timing_sr[11] & mcuclk_pcen;

wire    opcode_inlatch_tick = timing_sr[6] & current_bus_acc == RD4 & mcuclk_pcen;
wire    md_inlatch_tick = timing_sr[6] & current_bus_acc == RD3 & mcuclk_pcen;
wire    full_opcode_inlatch_tick_debug = timing_sr[6] & (current_bus_acc == RD4 | current_bus_acc == RD3) & mcuclk_pcen;

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
//////  INTERRUPT HANDLER
////


reg             int_nmi; //nNMI physical pin input, takes maximum 10us to suppress glitch
reg             int_timer0, int_timer1; //timer 0/1 match interrupt
reg             int_pint1, int_nint2; //INT1, nINT2 physical pin input, takes 12+2 mcuclk cycles to suppress gluitch
reg             int_cntr0, int_cntr1; //timer/event counter 0/1 match interrupt
reg             int_ncntrcin; //falling edge of the timer/event countr input (CI input) or timer output (TO) -> from the datasheet
reg             int_adc; //adc conversion complete
reg             int_buffull, int_bufempty; //UART buffer full/empty
wire    [2:0]   int_lv;



reg     [15:0]  int_addr;
always @(*) begin
    case(int_lv)
        3'b000: int_addr = 16'h0060; //SOFTI
        3'b001: int_addr = 16'h0004; //NMI
        3'b010: int_addr = 16'h0018; //TIMER
        3'b011: int_addr = 16'h0010; //INT PIN
        3'b100: int_addr = 16'h0018; //COUNTER RELATED
        3'b101: int_addr = 16'h0020; //ADC
        3'b110: int_addr = 16'h0028; //SERIAL INTERFACE
        3'b111: int_addr = 16'h0000; //not specified
    endcase
end








///////////////////////////////////////////////////////////
//////  MICROCODE ENGINE
////

reg             skip_check_alu;



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
    D[13:9] source B
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
        01011:
        01100:
        01101:
        01110: (w) A
        01111: (b) EA
        10000:
        10001: (w) ADDR_V_WA 
        10010: (w) ADDR_TA
        10011: (w) ADDR_FA   
        10100: (w) ADDR_REL_S
        10101: (w) ADDR_REL_L
        10110: (w) *ADDR_INT, interrupt address, including software interrupt
        10111: (w) -2
        11000: (w) -1
        11001: (w) 0
        11010: (w) 1
        11011: (w) 2
        11100: (w) ALU temp register 
        11101: (w) *RPA1
        11110: (w) *RPA2
        11111: (w) *RPA_OFFSET, rpa2/rpa3 A, B, EA, byte addend select
    D[8:3] source A, destination register type, decoded by the external circuit, :
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
        11100: 
        11101: 
        11110: 
        11111: 
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
    01_X_X_XXXX_XXXX_XXXX_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13:10] source D
    0000: A
    0001: EA
    0010: BC
    0011: DE
    0100: HL
    0101: MD_high_byte
    0110: MD_word
    0111: PC
    1000: SP
    1001: PSW
    1010:
    1011:
    1100: 
    1101: 
    1110:
    1111: (w) *RPA
    D[9:7] source C, destination
    0000: (b) r2
    0001: (b) A
    0010: (w) EA
    0011: (b) MD_low_byte
    0100: (w) MD_word
    0101: (w) MA
    0110: (b) PSW
    0111: 
    1000: 
    1001: 
    1010: 
    1011: 
    1100: 
    1101: 
    1110: 
    1111: (w) *RPA

    D[6:2] ALU operation type:
    0000: bypass
    0001: NEGA(negate)
    0010: DAA(what the fuck is that)
    0011: RLD(rotate left digit)
    0100: RRD(rotate right digit)
    0101: MUL
    0110: DIV
    0111: shift operation, OPCODE[7], OPCODE[2], OPCODE[5:4], 
    1000: (-A)push operation: alu out=A-1, ma out=A-1
    1001: (A+)pop operation: alu out=A+1, ma out=A
    1010: rpa auto decrement/increment operation, use opcode field
    1011: INC +0x01
    1100: DEC +0xFF
    1101:
    1110:
    1111:
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read


    3. BOOKKEEPING OPERATION
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
    D[7:5]: CPU control
        000: SK
        001: SKN
        010: SKIT
        011: SKNIT
        100: HLT
        101: STOP
        110: 
        111:
    D[4]: reserved
    D[3]: reserved
    D[2]: reserved
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read

    4. SPECIAL OPERATION
    11_X_X_X_XXXX_X_X_XX_X_?_?_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: nop
    D[12:9]: nop cycles 0=>1cycle, 15=16cycles
    D[8]: conditional PC decrement(BLOCK)
    D[7]: conditional read(rpa+byte or register)
    D[6]: conditional branch on ALU type
    D[5:4]: branch+ steps 0=>+2 3=>+5
    D[3]: swap MD input order
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read
*/


///////////////////////////////////////////////////////////
//////  MICROCODE/ALU OUTPUT DECODER
////

//
//  ALU
//

reg     [15:0]  alu_output; //ALU output
reg     [15:0]  alu_ma_output; //ALU output for the memory address register
reg     [15:0]  alu_temp_output; //ALU temp register
reg             alu_muldiv_reg_TEMP_wr, alu_digrot_TEMP_wr, alu_muldiv_reg_EA_wr;
wire            reg_TEMP_wr = alu_muldiv_reg_TEMP_wr | alu_digrot_TEMP_wr;


//
//  Microcode
//

//GPR write
wire    [2:0]   r_addr = reg_OPCODE[2:0];
wire    [1:0]   r2_addr = reg_OPCODE[1:0];
wire    [2:0]   r1_addr = reg_OPCODE[2:0];
wire    [2:0]   rp2_addr = reg_OPCODE[6:4];
wire    [1:0]   rp_addr = reg_OPCODE[5:4];
wire    [2:0]   rp1_addr = reg_OPCODE[2:0];
wire    [1:0]   rpa_incdec_addr = {reg_OPCODE[2], reg_OPCODE[0]};

wire            reg_EAL_wr = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_EA ||                      //direct designation
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_EA ||                      //direct designation
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd1 || //rp1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd4 || //rp2 word
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd4 || //rp1 word
                             alu_muldiv_reg_EA_wr;

wire            reg_EAH_wr = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_EA ||                      //direct designation
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_EA ||                      //direct designation
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd0 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd4 || //rp2 word
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd4 || //rp1 word
                             alu_muldiv_reg_EA_wr;
                             
wire            reg_V_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd0 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd0;   //rp1 word

wire            reg_A_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_A ||                       //direct designation
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_A ||                       //direct designation
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd1 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd1 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd1 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd0;   //rp1 word

wire            reg_B_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd2 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd2 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd2 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd2 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd1 || //rp2 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd1 || //rp byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd1;   //rp1 byte

wire            reg_C_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_C ||                       //direct designation
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd3 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd3 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd3 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd3 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd1 || //rp2 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd1 || //rp byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd1;   //rp1 byte

wire            reg_D_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd4 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd4 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd4 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd4 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd2 || //rp2 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd2 || //rp byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd2 || //rp1 byte
                             mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd2;    //rpa auto inc/dec condition

wire            reg_E_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd5 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd5 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd5 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd5 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd2 || //rp2 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd2 || //rp byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd2 || //rp1 byte
                             mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd2;    //rpa auto inc/dec condition

wire            reg_H_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd6 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd6 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd6 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd6 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd3 || //rp2 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd3 || //rp byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd3 || //rp1 byte
                             mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd3;    //rpa auto inc/dec condition

wire            reg_L_wr   = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd7 || //r byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd7 || //r2 byte(mc0)
                             mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd7 || //r2 byte(mc1)
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd7 || //r1 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd3 || //rp2 byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd3 || //rp byte
                             mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd3 || //rp1 byte
                             mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd3;    //rpa auto inc/dec condition

wire            reg_wr_hilo = mc_sa_dst == SA_DST_RP2 || mc_sa_dst == SA_DST_RP || mc_sa_dst == SA_DST_RP1 || mc_sc_dst == SC_DST_RPA;

//PC/SP
wire            reg_PC_wr = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_PC ||
                            mc_type == MCTYPE3 && mc_conditional[3];
wire            reg_SP_wr = mc_type == MCTYPE0 && mc_sa_dst == SA_DST_SP;

//Memory IO related registers, MA=Memory Address, MD=Memory Data
wire            reg_MA_dec_mode = mc_type == MCTYPE1 && mc_t1_alusel == 4'h8;
wire            reg_MA_wr   = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_MA) ||
                              (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_MA);

wire            reg_MDL_wr_A  = (mc_type == MCTYPE0 && mc_sb == SB_RPA2 && mc_sa_dst == SA_DST_MA && opcode_page == 3'd0);
wire            reg_MD_wr_EA = (mc_type == MCTYPE0 && mc_sb == SB_RPA2 && mc_sa_dst == SA_DST_MA && opcode_page == 3'd1);
wire            reg_MDL_wr  = (mc_type == MCTYPE0 && (mc_sa_dst == SA_DST_MDL || mc_sa_dst == SA_DST_MD)) || 
                             (mc_type == MCTYPE1 && (mc_sc_dst == SC_DST_MDL || mc_sc_dst == SC_DST_MD));
wire            reg_MDH_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_MD) || 
                             (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_MD);
wire            reg_MD_swap_input_order = mc_type == MCTYPE3 && mc_conditional[0]; //swaps MD input order, from lo->hi to hi->lo
wire            reg_MD_swap_output_order = mc_type == MCTYPE1 && mc_sc_dst == SC_DST_MA && mc_t1_alusel == 4'b1000; //swaps MD output order, push MD to stack

//ALU control
wire            alu_mul_start = mc_type == MCTYPE1 && mc_t1_alusel == 4'b0101;
wire            alu_div_start = mc_type == MCTYPE1 && mc_t1_alusel == 4'b0110;





///////////////////////////////////////////////////////////
//////  REGISTER FILE
////

/*
    TODO
    인터럽트 샘플링 시 RD4 3사이클(12 x mcupcen = 800ns)동안 신호가 유지되어야함, 2 x mcupcen동안 뭔가 시프트
*/

//
//  General purpose registers
//

//register pair select switch
reg             flag_EXX, flag_EXA, flag_EXH;
wire            sel_BCDE = flag_EXX;
wire            sel_VAEA = flag_EXA;
wire            sel_HL = flag_EXX ^ flag_EXH;

always @(posedge emuclk) begin
    if(!mrst_n) begin
        flag_EXX <= 1'b0;
        flag_EXA <= 1'b0;
        flag_EXH <= 1'b0;
    end
    else begin
        if(mc_type == MCTYPE2 && mc_bookkeeping[9]) flag_EXX <= ~flag_EXX;
        if(mc_type == MCTYPE2 && mc_bookkeeping[8]) flag_EXA <= ~flag_EXA;
        if(mc_type == MCTYPE2 && mc_bookkeeping[7]) flag_EXH <= ~flag_EXH;
    end
end

//register pairs and write control

reg     [7:0]   regpair_EAH, regpair_EAL, regpair_V, regpair_A, regpair_B, regpair_C, regpair_D, regpair_E, regpair_H, regpair_L [0:1];
always @(posedge emuclk) begin
    if(!mrst_n) begin
        regpair_EAH[0] <= 8'h00; regpair_EAH[1] <= 8'h00;
        regpair_EAL[0] <= 8'h00; regpair_EAL[1] <= 8'h00;
        regpair_V[0] <= 8'h00; regpair_V[1] <= 8'h00;
        regpair_A[0] <= 8'h00; regpair_A[1] <= 8'h00;
        regpair_B[0] <= 8'h00; regpair_B[1] <= 8'h00;
        regpair_C[0] <= 8'h00; regpair_C[1] <= 8'h00;
        regpair_D[0] <= 8'h00; regpair_D[1] <= 8'h00;
        regpair_E[0] <= 8'h00; regpair_E[1] <= 8'h00;
        regpair_H[0] <= 8'h00; regpair_H[1] <= 8'h00;
        regpair_L[0] <= 8'h00; regpair_L[1] <= 8'h00;
    end
    else begin
        if(reg_EAH_wr) regpair_EAH[sel_VAEA] <= alu_output[15:8];
        if(reg_EAL_wr) regpair_EAL[sel_VAEA] <= alu_output[7:0];
        if(reg_V_wr)   regpair_V[sel_VAEA]   <= reg_wr_hilo ? alu_output[15:8] : alu_output[7:0];
        if(reg_A_wr)   regpair_A[sel_VAEA]   <= alu_output[7:0];
        if(reg_B_wr)   regpair_B[sel_BCDE]   <= reg_wr_hilo ? alu_output[15:8] : alu_output[7:0];
        if(reg_C_wr)   regpair_C[sel_BCDE]   <= alu_output[7:0];
        if(reg_D_wr)   regpair_D[sel_BCDE]   <= reg_wr_hilo ? alu_output[15:8] : alu_output[7:0];
        if(reg_E_wr)   regpair_E[sel_BCDE]   <= alu_output[7:0];
        if(reg_H_wr)   regpair_H[sel_HL]     <= reg_wr_hilo ? alu_output[15:8] : alu_output[7:0];
        if(reg_L_wr)   regpair_L[sel_HL]     <= alu_output[7:0];
    end
end

//register pair output selectors
wire    [7:0]   reg_EAH = regpair_EAH[sel_VAEA];
wire    [7:0]   reg_EAL = regpair_EAL[sel_VAEA]; 
wire    [7:0]   reg_V = regpair_V[sel_VAEA]; 
wire    [7:0]   reg_A = regpair_A[sel_VAEA]; 
wire    [7:0]   reg_B = regpair_B[sel_BCDE]; 
wire    [7:0]   reg_C = regpair_C[sel_BCDE]; 
wire    [7:0]   reg_D = regpair_D[sel_BCDE]; 
wire    [7:0]   reg_E = regpair_E[sel_BCDE]; 
wire    [7:0]   reg_H = regpair_H[sel_HL]; 
wire    [7:0]   reg_L = regpair_L[sel_HL];


//
//  Arbitrarily made registers: unsure the original chip has them
//

reg     [15:0]  reg_INLATCH; //inlatch for data sampling
reg     [7:0]   reg_MDH, reg_MDL; //byte [15:8], word[15:0]
reg     [15:0]  reg_TEMP;


//
//  Special registers
//

reg     [7:0]   reg_MKL; //intrq disable register low ; ncntrcin, cntr1, cntr0, pint1, nint2, timer1, timer0, nmi(??) 
reg     [2:0]   reg_MKH; //intrq disable register high; -, -, -, -, -, empty, full, adc


//
//  Flags
//

reg             flag_Z, flag_SK, flag_CY, flag_HC, flag_L1, flag_L0;
wire    [7:0]   reg_PSW = {1'b1, flag_Z, flag_SK, flag_HC, flag_L1, flag_L0, 1'b0, flag_CY};


//
//  PC, SP, MA registers with auto increment/decrement feature
//

reg     [15:0]  reg_PC, reg_SP, reg_MA;

//address source selector
localparam PC = 2'b0;
localparam MA = 2'b1;
reg             address_source_sel;
reg             reg_PC_inc_stop, reg_MA_inc_ndec;
reg     [15:0]  memory_access_address;

//this block defines the operation of the PC/MA registers
always @(posedge emuclk) begin
    //ADDRESS OUTPUT SOURCE SELECT
    if(!mrst_n) begin
        address_source_sel <= PC;
        reg_PC_inc_stop <= 1'b0;

        reg_MA_inc_ndec <= 1'b1;
    end
    else begin
        if(cycle_tick) begin
            if(mc_end_of_instruction) begin
                address_source_sel <= PC;
                reg_PC_inc_stop <= 1'b0;

                reg_MA_inc_ndec <= 1'b1;
            end
            else begin
                if(reg_PC_wr) address_source_sel <= PC; //select PC
                else if(reg_MA_wr) begin
                    address_source_sel <= MA; //select MA
                    reg_PC_inc_stop <= 1'b1;
                end

                if(reg_MA_dec_mode) reg_MA <= 1'b0;
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
            if(reg_PC_wr) reg_PC <= alu_output;
            else begin
                if(reg_PC_inc_stop) reg_PC <= reg_PC;
                else begin
                    if(current_bus_acc == RD4 || current_bus_acc == RD3) reg_PC <= reg_PC == 16'hFFFF ? 16'h0000 : reg_PC + 16'h0001;
                end
            end

            //Stack Pointer load condition
            if(reg_SP_wr) reg_SP <= alu_output;

            //Memory Address load/auto inc conditions
            if(reg_MA_wr) reg_MA <= alu_ma_output;
            else begin
                if(opcode_tick) reg_MA <= reg_PC;
                else begin
                    if(current_bus_acc == RD3 || current_bus_acc == WR3) begin //if there was a 3cyc read/write access,
                        if(address_source_sel == MA) begin
                            if(reg_MA_inc_ndec) reg_MA <= reg_MA == 16'hFFFF ? 16'h0000 : reg_MA + 16'h0001;
                            else reg_MA <= reg_MA == 16'h0000 ? 16'hFFFF : reg_MA - 16'h0001;
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
        MA: memory_access_address = reg_MA;
    endcase
end





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
                //swap output data order(to HI->LO) when the current microcode operation is MD<-PC
                if(reg_MD_swap_output_order) md_out_byte_sel <= 1'b1;
                else begin
                    if(current_bus_acc == WR3) md_out_byte_sel <= ~md_out_byte_sel;
                end

                //swap input data order(to HI->LO) when the bookkeeping bit is hot, this is for WA, BYTE instruction
                if(reg_MD_swap_input_order) md_in_byte_sel <= 1'b1;
                else begin
                    if(current_bus_acc == RD3) md_in_byte_sel <= ~md_in_byte_sel;
                end
            end
        end
    end
end


//OPCODE/memory data IO
always @(posedge emuclk) begin
    if(!mrst_n) begin
        reg_INLATCH <= 16'h0000;
        reg_MDL <= 8'h00;
        reg_MDH <= 8'h00;
        reg_OPCODE <= 8'h00;
    end
    else begin
        if(mcuclk_pcen) begin
            //Memory Data register load
            if(cycle_tick) begin
                if(reg_MDL_wr_A) begin //save A to MDL(rpa2, stax)
                    reg_MDL <= reg_A;
                end
                else if(reg_MD_wr_EA) begin //save EA to MD(rpa2, steax)
                    reg_MDL <= reg_EAL;
                    reg_MDH <= reg_EAH;
                end
                else begin
                    if(reg_MDL_wr) reg_MDL <= alu_output[7:0];
                    else reg_MDL <= reg_INLATCH[7:0];

                    if(reg_MDH_wr) reg_MDH <= alu_output[15:8];
                    else reg_MDH <= reg_INLATCH[15:8];
                end
            end
            else if(md_inlatch_tick) begin
                if(md_in_byte_sel) reg_INLATCH[15:8] <= i_PD_I;
                else reg_INLATCH[7:0] <= i_PD_I;
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
wire    [7:0]   md_out_byte_data = md_out_byte_sel == 1'b1 ? reg_MDH : reg_MDL;
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







///////////////////////////////////////////////////////////
//////  ALU READ PORT MULTIPLEXERS
////

//r addressing
reg     [7:0]   reg_R, reg_R2, reg_R1;
always @(*) begin
    case(reg_OPCODE[2:0])
        3'b000: reg_R = reg_V;
        3'b001: reg_R = reg_A;
        3'b010: reg_R = reg_B;
        3'b011: reg_R = reg_C;
        3'b100: reg_R = reg_D;
        3'b101: reg_R = reg_E;
        3'b110: reg_R = reg_H;
        3'b111: reg_R = reg_L;
    endcase

    case(reg_OPCODE[2:0])
        3'b000: reg_R1 = reg_EAH;
        3'b001: reg_R1 = reg_EAL;
        3'b010: reg_R1 = reg_B;
        3'b011: reg_R1 = reg_C;
        3'b100: reg_R1 = reg_D;
        3'b101: reg_R1 = reg_E;
        3'b110: reg_R1 = reg_H;
        3'b111: reg_R1 = reg_L;
    endcase

    case(reg_OPCODE[1:0])
        2'b00: reg_R2 = reg_V;
        2'b01: reg_R2 = reg_A;
        2'b10: reg_R2 = reg_B;
        2'b11: reg_R2 = reg_C;
    endcase
end

//rp addressing
reg     [15:0]   reg_RP2, reg_RP, reg_RP1;
always @(*) begin
    case(reg_OPCODE[6:4])
        3'b000: reg_RP2 = reg_SP;
        3'b001: reg_RP2 = {reg_B, reg_C};
        3'b010: reg_RP2 = {reg_D, reg_E};
        3'b011: reg_RP2 = {reg_H, reg_L};
        3'b100: reg_RP2 = {reg_E, reg_A};
        3'b101: reg_RP2 = 16'h0000; //not specified on the datasheet
        3'b110: reg_RP2 = 16'h0000;
        3'b111: reg_RP2 = 16'h0000;
    endcase

    case(reg_OPCODE[2:0])
        3'b000: reg_RP1 = {reg_V, reg_A};
        3'b001: reg_RP1 = {reg_B, reg_C};
        3'b010: reg_RP1 = {reg_D, reg_E};
        3'b011: reg_RP1 = {reg_H, reg_L};
        3'b100: reg_RP1 = {reg_E, reg_A};
        3'b101: reg_RP1 = 16'h0000;
        3'b110: reg_RP1 = 16'h0000;
        3'b111: reg_RP1 = 16'h0000;
    endcase

    case(reg_OPCODE[1:0])
        2'b00: reg_RP = reg_SP;
        2'b01: reg_RP = {reg_B, reg_C};
        2'b10: reg_RP = {reg_D, reg_E};
        2'b11: reg_RP = {reg_H, reg_L};
    endcase
end

//rpa addressing
reg     [15:0]   reg_RPA1, reg_RPA, reg_RPA2, reg_RPA2_OFFSET;
always @(*) begin
    case(reg_OPCODE[1:0])
        2'b00: reg_RPA1 = 16'h0000;
        2'b01: reg_RPA1 = {reg_B, reg_C};
        2'b10: reg_RPA1 = {reg_D, reg_E};
        2'b11: reg_RPA1 = {reg_H, reg_L};
    endcase

    //rpa, including auto inc/dec
    case(reg_OPCODE[2:0])
        3'b000: reg_RPA = 16'h0000;
        3'b001: reg_RPA = {reg_B, reg_C};
        3'b010: reg_RPA = {reg_D, reg_E};
        3'b011: reg_RPA = {reg_H, reg_L};
        3'b100: reg_RPA = {reg_D, reg_E}; //A+, use alu type 1 for auto inc/dec
        3'b101: reg_RPA = {reg_H, reg_L}; //A+, use alu type 1 for auto inc/dec
        3'b110: reg_RPA = {reg_D, reg_E}; //-A, use alu type 1 for auto inc/dec
        3'b111: reg_RPA = {reg_H, reg_L}; //-A, use alu type 1 for auto inc/dec
    endcase

    //rpa2, +byte, +A, +B, +EA only
    case(reg_OPCODE[2:0])
        3'b000: reg_RPA2 = 16'h0000;
        3'b001: reg_RPA2 = 16'h0000;
        3'b010: reg_RPA2 = 16'h0000;
        3'b011: reg_RPA2 = {reg_D, reg_E};
        3'b100: reg_RPA2 = {reg_H, reg_L};
        3'b101: reg_RPA2 = {reg_H, reg_L};
        3'b110: reg_RPA2 = {reg_H, reg_L};
        3'b111: reg_RPA2 = {reg_H, reg_L};
    endcase

    //rpa2 addend select
    case(reg_OPCODE[2:0])
        3'b000: reg_RPA2_OFFSET = 16'h0000;
        3'b001: reg_RPA2_OFFSET = 16'h0000;
        3'b010: reg_RPA2_OFFSET = 16'h0000;
        3'b011: reg_RPA2_OFFSET = {8'h00, reg_MDL}; //use alu type 0 to select an addend automatically
        3'b100: reg_RPA2_OFFSET = {8'h00, reg_A}; 
        3'b101: reg_RPA2_OFFSET = {8'h00, reg_B};
        3'b110: reg_RPA2_OFFSET = {reg_EAH, reg_EAL};
        3'b111: reg_RPA2_OFFSET = {8'h00, reg_MDL};
    endcase
end



//ALU port A and B
reg     [15:0]  alu_pa, alu_pb; 
always @(*) begin
    if(mc_type == MCTYPE0) begin
        case(mc_sa_dst)
            SA_DST_R      : alu_pa = {8'h00, reg_R};
            SA_DST_R2     : alu_pa = {8'h00, reg_R2};
            SA_DST_R1     : alu_pa = {8'h00, reg_R1};
            SA_DST_RP2    : alu_pa = reg_RP2;
            SA_DST_RP     : alu_pa = reg_RP;
            SA_DST_RP1    : alu_pa = reg_RP1;
            SA_DST_SR_SR1 : alu_pa = 16'h0000; //to be fixed
            SA_DST_SR2    : alu_pa = 16'h0000; //to be fixed
            SA_DST_SR3    : alu_pa = 16'h0000; //to be fixed
            SA_DST_MDL    : alu_pa = {8'h00, reg_MDL};
            SA_DST_MD     : alu_pa = {reg_MDH, reg_MDL};
            SA_DST_MA     : alu_pa = reg_MA;
            SA_DST_PC     : alu_pa = reg_PC;
            SA_DST_SP     : alu_pa = reg_SP;
            SA_DST_A      : alu_pa = {8'h00, reg_A};
            SA_DST_EA     : alu_pa = {reg_EAH, reg_EAL};
            SA_DST_C      : alu_pa = {8'h00, reg_C};
            SA_DST_RPA1   : alu_pa = reg_RPA1;
            SA_DST_RPA2   : alu_pa = reg_RPA2;
            default       : alu_pa = 16'h0000;
        endcase

        case(mc_sb)
            SB_R          : alu_pb = {8'h00, reg_R};
            SB_R2         : alu_pb = {8'h00, reg_R2};
            SB_R1         : alu_pb = {8'h00, reg_R1};
            SB_RP2        : alu_pb = reg_RP2;
            SB_RP         : alu_pb = reg_RP;
            SB_RP1        : alu_pb = reg_RP1;
            SB_SR_SR1     : alu_pb = 16'h0000; //to be fixed
            SB_SR2        : alu_pb = 16'h0000; //to be fixed
            SB_SR4        : alu_pb = 16'h0000; //to be fixed
            SB_MDH        : alu_pb = {8'h00, reg_MDH};
            SB_MD         : alu_pb = {reg_MDH, reg_MDL};
            SB_A          : alu_pb = {8'h00, reg_A};
            SB_EA         : alu_pb = {reg_EAH, reg_EAL};
            SB_ADDR_V_WA  : alu_pb = {reg_V, reg_MD_swap_input_order ? reg_INLATCH[15:8] : reg_INLATCH[7:0]};
            SB_ADDR_TA    : alu_pb = {8'h00, 2'b10, reg_OPCODE[4:0], 1'b0};
            SB_ADDR_FA    : alu_pb = {5'b00001, reg_OPCODE[2:0], reg_INLATCH[7:0]};
            SB_ADDR_REL_S : alu_pb = {{11{reg_OPCODE[5]}}, reg_OPCODE[4:0]}; //sign extension
            SB_ADDR_REL_L : alu_pb = {{8{reg_OPCODE[0]}}, reg_INLATCH[7:0]};
            SB_ADDR_INT   : alu_pb = int_addr; //selected externally
            SB_SUB2       : alu_pb = 16'hFFFE;
            SB_SUB1       : alu_pb = 16'hFFFF;
            SB_ZERO       : alu_pb = 16'h0000;
            SB_ADD1       : alu_pb = 16'h0001;
            SB_ADD2       : alu_pb = 16'h0002;
            SB_TEMP       : alu_pb = reg_TEMP;
            SB_RPA1       : alu_pb = reg_RPA1;
            SB_RPA2       : alu_pb = reg_RPA2;
            SB_OFFSET     : alu_pb = reg_RPA2_OFFSET;
            default       : alu_pb = 16'h0000;
        endcase
    end
    else if(mc_type == MCTYPE1) begin
        case(mc_sc_dst)
            SC_DST_R2     : alu_pa = {8'h00, reg_R2};
            SC_DST_A      : alu_pa = reg_A;
            SC_DST_EA     : alu_pa = {reg_EAH, reg_EAL};
            SC_DST_MDL    : alu_pa = {8'h00, reg_MDL};
            SC_DST_MD     : alu_pa = {reg_MDH, reg_MDL};
            SC_DST_MA     : alu_pa = reg_MA;
            SC_DST_PSW    : alu_pa = reg_PSW;
            SC_DST_RPA    : alu_pa = reg_RPA;
            default       : alu_pa = 16'h0000;
        endcase

        case(mc_sd)
            SD_A          : alu_pb = reg_A;
            SD_EA         : alu_pb = {reg_EAH, reg_EAL};
            SD_BC         : alu_pb = {reg_B, reg_C};
            SD_DE         : alu_pb = {reg_D, reg_H};
            SD_HL         : alu_pb = {reg_H, reg_L};
            SD_MDH        : alu_pb = {8'h00, reg_MDH};
            SD_MD         : alu_pb = {reg_MDH, reg_MDL};
            SD_PC         : alu_pb = reg_PC;
            SD_SP         : alu_pb = reg_SP;
            SD_PSW        : alu_pb = reg_PSW;
            SD_RPA        : alu_pb = reg_RPA;
            default       : alu_pb = 16'h0000;
        endcase
    end
    else if(mc_type == MCTYPE3) begin
        if(mc_conditional[3]) begin
            alu_pa = reg_PC;
            alu_pb = reg_C == 8'hFF ? 16'h0000 : 16'hFFFF;
        end
    end
    else begin
        alu_pa = 16'h0000;
        alu_pb = 16'h0000;
    end
end




///////////////////////////////////////////////////////////
//////  ALU
////

//
//  ALU: full adder with nibble, byte, word carry outputs
//

reg     [15:0]  alu_adder_op0, alu_adder_op1;
reg             alu_adder_cin, alu_adder_co;

wire    [15:0]  alu_adder_out;
wire    [4:0]   alu_adder_nibble_lo, alu_adder_nibble_hi;
wire    [8:0]   alu_adder_byte_high;
wire            alu_adder_nibble_co = alu_adder_nibble_lo[4];
wire            alu_adder_byte_co = alu_adder_nibble_hi[4];
wire            alu_adder_word_co = alu_adder_byte_high[8];
reg             alu_adder_borrow_mode;
assign  alu_adder_out[3:0] = alu_adder_op0[3:0] + alu_adder_op1[3:0] + alu_adder_cin;
assign  alu_adder_out[7:4] = alu_adder_op0[7:4] + alu_adder_op1[7:4] + alu_adder_nibble_co;
assign  alu_adder_out[15:8] = alu_adder_op0[15:8] + alu_adder_op1[15:8] + alu_adder_byte_co;


//
//  ALU: shifter and rotator
//

reg     [15:0]  alu_shifter;
reg             alu_shifter_co;
always @(*) begin
    alu_shifter = 16'h00;
    alu_shifter_co = 1'b0;

    if(mc_type == MCTYPE1) if(mc_t1_alusel == 4'h6) begin
        case({reg_OPCODE[7], reg_OPCODE[2], reg_OPCODE[5:4]})
            4'b0000: begin alu_shifter[7] = 1'b0; 
                           alu_shifter[6:0] = alu_pb[7:1];
                           alu_shifter_co = alu_pb[0]; end //SLRC, skip condition: CARRY
            4'b0001: ; //no instruction specified
            4'b0010: begin alu_shifter[7] = 1'b0; 
                           alu_shifter[6:0] = alu_pb[7:1];
                           alu_shifter_co = alu_pb[0]; end //SLR
            4'b0011: begin alu_shifter[7] = flag_CY;
                           alu_shifter[6:0] = alu_pb[7:1];
                           alu_shifter_co = alu_pb[0]; end //RLR
            4'b0100: begin alu_shifter[0] = 1'b0; 
                           alu_shifter[7:1] = alu_pb[6:0];
                           alu_shifter_co = alu_pb[7]; end //SLLC, skip condition: CARRY
            4'b0101: ; //no instruction specified
            4'b0110: begin alu_shifter[0] = 1'b0; 
                           alu_shifter[7:1] = alu_pb[6:0];
                           alu_shifter_co = alu_pb[7]; end //SLL
            4'b0111: begin alu_shifter[0] = flag_CY;
                           alu_shifter[7:1] = alu_pb[6:0];
                           alu_shifter_co = alu_pb[7]; end //RLL

            4'b1000: ; //no instruction specified
            4'b1001: ; //no instruction specified
            4'b1010: begin alu_shifter[15] = 1'b0;
                           alu_shifter[14:0] = alu_pb[15:1];
                           alu_shifter_co = alu_pb[0]; end //DSLR
            4'b1011: begin alu_shifter[15] = flag_CY;
                           alu_shifter[14:0] = alu_pb[15:1];
                           alu_shifter_co = alu_pb[0]; end //DRLR
            4'b1100: ; //no instruction specified
            4'b1101: ; //no instruction specified
            4'b1110: begin alu_shifter[0] = 1'b0;
                           alu_shifter[15:1] = alu_pb[14:0];
                           alu_shifter_co = alu_pb[15]; end //DSLL
            4'b1111: begin alu_shifter[0] = flag_CY;
                           alu_shifter[15:1] = alu_pb[14:0];
                           alu_shifter_co = alu_pb[15]; end //DRLL
        endcase
    end
end


//
//  ALU: MUL/DIV sequencer
//

reg     [4:0]   alu_muldiv_cntr;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        alu_muldiv_cntr <= 5'd31;
    end
    else begin
        if(cycle_tick) begin
            if(alu_mul_start) if(alu_muldiv_cntr == 5'd31) alu_muldiv_cntr <= 5'd16;
            else if(alu_div_start) if(alu_muldiv_cntr == 5'd31) alu_muldiv_cntr <= 5'd0;
            else begin
                if(alu_muldiv_cntr != 5'd31) alu_muldiv_cntr <= alu_muldiv_cntr == 5'd23 || alu_muldiv_cntr == 5'd15 ? 5'd31 : alu_muldiv_cntr + 5'd1;
                else alu_muldiv_cntr <= 5'd31;
            end
        end
    end
end

wire    [15:0]  alu_mul_pb = alu_pb[alu_muldiv_cntr] ? {8'h00, reg_A} << alu_muldiv_cntr : 16'h0000;
wire    [15:0]  alu_div_pa = reg_TEMP;
wire    [31:0]  alu_div_out = alu_adder_out[15] ? {{reg_TEMP, {reg_EAH, reg_EAL}}[30:0], 1'b0} :
                                                  {{alu_adder_out, {reg_EAH, reg_EAL}}[30:0], 1'b1};


//
//  ALU: operator
//

always @(*) begin
    //maintain current destination register's data, if the port is not altered
    alu_adder_op0 = 16'h0000; alu_adder_op1 = 16'h0000; alu_adder_cin <= 1'b0; //FA inputs
    alu_adder_borrow_mode = 1'b0;

    alu_output = alu_pa; //result output
    alu_ma_output = alu_pa; //Memory Address output

    alu_digrot_TEMP_wr = 1'b0; //TEMP register write
    alu_temp_output = alu_pa; //TEMP register data output

    alu_muldiv_reg_TEMP_wr = 1'b0;
    alu_muldiv_reg_EA_wr = 1'b0;

    //pa = first operand, pb = second operand, like Vwa
    if(mc_type == MCTYPE0) begin
        if(mc_t0_alusel == 2'd0) begin
            alu_output = alu_pb; //out<-pb bypass
        end
        else if(mc_t0_alusel == 2'd1) begin
            alu_ma_output = alu_adder_out;
        end
        else begin
            case(mc_t0_alusel[0] ? {reg_OPCODE[0], reg_OPCODE[6:4]} : {reg_OPCODE[3], reg_OPCODE[6:4]})
                4'h0: alu_output = alu_pb;                        //MVI(move)
                4'h1: alu_output = alu_pa ^ alu_pb;               //XOR(bitwise XOR)
                4'h2: begin alu_output = alu_adder_out;           //ADDNC(check skip condition: NO CARRY)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin <= 1'b0; end
                4'h3: begin alu_output = alu_adder_out;           //SUBNB(check skip condition: NO BORROW; 2's complement)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
                4'h4: begin alu_output = alu_adder_out;           //ADD
                            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin <= 1'b0; end
                4'h5: begin alu_output = alu_adder_out;           //ADD with carry
                            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin <= flag_CY; end
                4'h6: begin alu_output = alu_adder_out;           //SUB
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
                4'h7: begin alu_output = alu_adder_out;           //SUB with borrow
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= flag_CY; 
                            alu_adder_borrow_mode = 1'b1; end
                4'h8: alu_temp_output = alu_pa & alu_pb;          //AND(bitwise AND)
                4'h9: alu_temp_output = alu_pa | alu_pb;          //OR(bitwise OR)
                4'hA: begin alu_temp_output = alu_adder_out;      //SGT(skip if greater than; PA-PB-1, adding the inverted PB has the same effect)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b0; 
                            alu_adder_borrow_mode = 1'b1; end
                4'hB: begin alu_temp_output = alu_adder_out;      //SLT(skip if less than; check skip condition: BORROW; 2's complement)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b1;
                            alu_adder_borrow_mode = 1'b1; end
                4'hC: alu_temp_output = alu_pa & alu_pb;          //AND(check skip condition: NO ZERO)
                4'hD: alu_temp_output = alu_pa | alu_pb;          //OR(check skip condition: ZERO)
                4'hE: begin alu_temp_output = alu_adder_out;      //SNE(skip on not equal; check skip condition: NO ZERO)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
                4'hF: begin alu_temp_output = alu_adder_out;      //SEQ(skip on equal; check skip condition: ZERO)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
            endcase
        end
    end
    else if(mc_type == MCTYPE1) begin
             if(mc_t1_alusel == 4'h0) alu_output = alu_pb; //out<-pb bypass
        else if(mc_t1_alusel == 4'h1) begin //2's complement
            alu_output = alu_adder_out;
            alu_adder_op0 = 16'h0000; alu_adder_op1 = ~alu_pb; alu_adder_cin <= 1'b1;
        end
        else if(mc_t1_alusel == 4'h2) begin //DAA, kinda shit
            alu_output = alu_adder_out;
            alu_adder_op0 = alu_pa; alu_adder_cin = 1'b0;
            if(flag_HC) begin
                if(flag_CY == 1'b0 && alu_pa[7:4] <= 4'h9) alu_adder_op1 = 16'h0006;
                else alu_adder_op1 = 16'h0066;
            end
            else begin
                if(alu_pa[3:0] <= 4'h9) begin
                    if(flag_CY == 1'b0 && alu_pa[7:4] <= 4'h9) alu_adder_op1 = 16'h0000;
                    else alu_adder_op1 = 16'h0060;
                end
                else begin
                    if(flag_CY == 1'b0 && alu_pa[7:4] <= 4'h9) alu_adder_op1 = 16'h0006;
                    else alu_adder_op1 = 16'h0066;
                end
            end
        end
        else if(mc_t1_alusel == 4'h3) begin //RLD PA=MDin, PB=reg_A
            alu_output = {alu_pa[3:0], alu_pb[3:0]}; //to MD
            alu_temp_output = {alu_pb[7:4], alu_pa[7:4]}; //to TEMP->A

            alu_digrot_TEMP_wr = 1'b1;
        end
        else if(mc_t1_alusel == 4'h4) begin //RRD PA=MDin, PB=reg_A
            alu_output = {alu_pb[3:0], alu_pa[7:4]}; //to MD
            alu_temp_output = {alu_pb[7:4], alu_pa[3:0]}; //to TEMP->A

            alu_digrot_TEMP_wr = 1'b1;
        end
        else if(mc_t1_alusel == 4'h5) begin //MUL pa=EA, pb=r2
            alu_output = 16'h0000; //reset EA
        end
        else if(mc_t1_alusel == 4'h6) begin //DIV
            alu_output = {alu_pa[14:0], 1'b0};
            alu_temp_output = {15'd0, alu_pa[15]};
        end
        else if(mc_t1_alusel == 4'h7) begin //shift 
            alu_output = alu_shifter;
        end
        else if(mc_t1_alusel == 4'h8) begin //PUSH, -ADDR
            alu_adder_op0 = 16'hFFFF; alu_adder_op1 = alu_pb; alu_adder_cin <= 1'b0;

            alu_output = alu_adder_out;
            alu_ma_output = alu_adder_out;
        end
        else if(mc_t1_alusel == 4'h9) begin //POP, ADDR+
            alu_adder_op0 = 16'h0001; alu_adder_op1 = alu_pb; alu_adder_cin <= 1'b0;

            alu_output = alu_adder_out;
            alu_ma_output = alu_pb;
        end
        else if(mc_t1_alusel == 4'hA) begin //rpa auto inc/dec
            if(reg_OPCODE[2]) begin
                alu_adder_op0 = reg_OPCODE[1] ? 16'hFFFF : 16'h0001;
            end
            else alu_adder_op0 = 16'h0000;
            alu_adder_op1 = alu_pb;
            alu_adder_cin = 1'b0;

            alu_output = alu_adder_out;
            alu_ma_output = alu_pb;
        end
        else if(mc_t1_alusel == 4'hB) begin //INC
            alu_adder_op0 = alu_pa; alu_adder_op1 = 16'h0001; alu_adder_cin = 1'b0;

            alu_output = alu_adder_out;
        end
        else if(mc_t1_alusel == 4'hC) begin //DEC
            alu_adder_op0 = alu_pa; alu_adder_op1 = 16'hFFFF; alu_adder_cin = 1'b1;
            alu_adder_borrow_mode = 1'b1;

            alu_output = alu_adder_out;
        end
    end
    else if(mc_type == MCTYPE3) begin
        if(mc_conditional[3]) begin
            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin = 1'b0;

            alu_output = alu_adder_out;
        end
    end

    if(alu_muldiv_cntr != 5'd31) begin
        if(alu_muldiv_cntr[4]) begin //multiply
            alu_muldiv_reg_EA_wr = 1'b1;

            alu_adder_op0 = alu_pa; //reg EA
            alu_adder_op1 = alu_mul_pb; //A * r2, shifted and masked
            alu_adder_cin = 1'b0;

            alu_output = alu_adder_out;
        end
        else begin
            alu_muldiv_reg_TEMP_wr = 1'b1;
            alu_muldiv_reg_EA_wr = 1'b1;

            alu_adder_op0 = alu_div_pa;  //reg EA
            alu_adder_op1 = ~alu_pb; //-r2
            alu_adder_cin = 1'b1;

            alu_output = alu_div_out[15:0];
            alu_temp_output = alu_div_out[31:16];
        end
    end
end




///////////////////////////////////////////////////////////
//////  FLAG GENERATOR
////

always @(posedge emuclk) begin
    if(!mrst_n) begin
        flag_Z <= 1'b0;
        flag_SK <= 1'b0;
        flag_CY <= 1'b0;
        flag_HC <= 1'b0;
        flag_L1 <= 1'b0;
        flag_L0 <= 1'b0;
    end
end



endmodule