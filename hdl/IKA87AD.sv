`timescale 10ps/10ps

module IKA87AD (
    //clock
    input   wire                i_EMUCLK,
    input   wire                i_MCUCLK_PCEN,

    //system control
    input   wire                i_RESET_n,
    input   wire                i_STOP_n,

    //R/W control and output enables
    output  wire                o_ALE,
    output  wire                o_RD_n,
    output  wire                o_WR_n,
    output  wire                o_ALE_OE,   //ALE output driver control(for CMOS variant, NMOS doesn't require this)
    output  wire                o_RD_n_OE,  //RD_n output driver control
    output  wire                o_WR_n_OE,  //WR_n output driver control

    //interrupt control
    input   wire                i_NMI_n,
    input   wire                i_INT1,

    //port C I/O and output enables
    input   wire    [7:0]       i_PC_I,
    output  wire    [7:0]       o_PC_O,
    output  wire    [7:0]       o_PC_OE,

    //port D I/O and output enables
    input   wire    [7:0]       i_PD_I,
    output  wire    [7:0]       o_PD_O,
    output  wire    [7:0]       o_PD_OE,

    output  wire    [15:0]      o_FULL_ADDRESS_DEBUG,
    output  wire    [7:0]       o_OUTPUT_DATA_DEBUG
);


//include mnemonic list
`include "IKA87AD_mnemonics.sv"

///////////////////////////////////////////////////////////
//////  CLOCK AND RESET
////

wire            emuclk = i_EMUCLK;
wire            mcuclk_pcen = i_MCUCLK_PCEN;
wire            mrst_n = i_RESET_n;



///////////////////////////////////////////////////////////
//////  OPCODE DECODER
////

reg     [2:0]   opcode_page; //page indicator
reg     [7:0]   reg_OPCODE; //opcode register

//disassembler
reg     [1:0]   reg_FULL_OPCODE_cntr;
reg     [7:0]   reg_FULL_OPCODE_debug[0:3]; //wtf

//opcode decoder
wire    [7:0]   mcrom_sa;
IKA87AD_opdec u_opdec (
    .i_OPCODE                   (reg_OPCODE                 ),
    .i_OPCODE_PAGE              (opcode_page                ),
    .o_MCROM_SA                 (mcrom_sa                   )
);



///////////////////////////////////////////////////////////
//////  MICROCODE OUTPUT SIGNALS
////

//microcode ROM control/raw output
wire            mcrom_read_tick; //BRAM read tick
wire    [17:0]  mcrom_data; //ROM output, registered

//modified control output
reg     [17:0]  mc_ctrl_output; //combinational

//fixed fields
wire    [1:0]   mc_type = mc_ctrl_output[17:16];
wire            mc_alter_flag = mcrom_data[15];
wire            mc_jump_to_next_inst = mcrom_data[14];

//bus control signals
wire    [1:0]   mc_next_bus_acc = mc_ctrl_output[1:0];

//MICROCODE TYPE 0 FIELDS
wire    [4:0]   mc_sb = mc_ctrl_output[13:9]; //microcode type 0, source b
wire    [4:0]   mc_sa_dst = mc_ctrl_output[8:4]; //microcode type 0, source a
wire    [1:0]   mc_t0_alusel = mc_ctrl_output[3:2];

//MICROCODE TYPE 1 FIELDS
wire    [3:0]   mc_sd = mc_ctrl_output[13:10]; //microcode type 1, source d
wire    [3:0]   mc_sc_dst = mc_ctrl_output[9:6]; //microcode type 1, source c
wire    [3:0]   mc_t1_alusel = mc_ctrl_output[5:2];

//MICROCODE TYPE 2 FIELDS
wire            mc_bk_carry_ctrl = mc_ctrl_output[9];
wire            mc_bk_irq_ctrl   = mc_ctrl_output[8];
wire    [1:0]   mc_bk_reg_exchg  = mc_ctrl_output[7:6];
wire            mc_bk_cpu_susp   = mc_ctrl_output[5];
wire    [2:0]   mc_bk_skip_ctrl  = mc_ctrl_output[4:2];

//MICROCODE TYPE 3 FIELDS
wire    [4:0]   mc_s_nop         = mcrom_data[13:9];
wire            mc_s_cond_pc_dec = mc_ctrl_output[8];
wire            mc_s_cond_read   = mc_ctrl_output[7];
wire    [2:0]   mc_s_bra_on_alu  = mc_ctrl_output[6:4];
wire            mc_s_swap_md_out = mc_ctrl_output[3];
wire            mc_s_ird         = mc_ctrl_output[2];

//ALU FIELDS
wire    [3:0]   arith_code = opcode_page == 3'd0 ? {reg_OPCODE[0], reg_OPCODE[6:4]} : {reg_OPCODE[3], reg_OPCODE[6:4]};
wire            is_arith_eval_op = arith_code > 4'h9; //is an arithmetic code the evaluation operation like GT, NE, OFF, ON, EQ, NE?
wire    [3:0]   shift_code = {reg_OPCODE[7], reg_OPCODE[2], reg_OPCODE[5:4]};

//END OF INSTRUCTION
wire            mc_end_of_instruction = mc_next_bus_acc == RD4 && !(mc_type == MCTYPE3 && mc_s_ird);



///////////////////////////////////////////////////////////
//////  TIMING GENERATOR
////

reg             soft_halt, soft_stop, hard_stop;
wire            susp_detected = soft_halt | soft_stop;

reg     [11:0]  timing_sr;
reg     [1:0]   current_bus_acc;

wire    opcode_tick = timing_sr[11] & current_bus_acc == RD4 & mcuclk_pcen;
wire    rw_tick = timing_sr[8] & current_bus_acc != RD4 & mcuclk_pcen;
wire    cycle_tick = opcode_tick | rw_tick;
wire    div3_tick = |{timing_sr[2], timing_sr[5], timing_sr[8], timing_sr[11]};

assign  mcrom_read_tick = (timing_sr[8] | timing_sr[11]) & mcuclk_pcen;

wire    opcode_inlatch_tick = timing_sr[6] & current_bus_acc == RD4 & mcuclk_pcen;
wire    md_inlatch_tick = timing_sr[6] & current_bus_acc == RD3 & mcuclk_pcen;
wire    full_opcode_inlatch_tick_debug = timing_sr[6] & (current_bus_acc == RD4 | current_bus_acc == RD3) & mcuclk_pcen;

always @(posedge emuclk) begin
    if(!mrst_n) begin
        timing_sr <= 12'b000_100_000_000;
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
                else if(timing_sr[10]) begin
                    if(!susp_detected) begin
                        timing_sr[0] <= timing_sr[11];
                        timing_sr[11:1] <= timing_sr[10:0];
                    end
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
                //else if(timing_sr[0]) begin
                //    if(!susp_detected) begin
                //        timing_sr[0] <= timing_sr[8];
                //        timing_sr[11:1] <= timing_sr[10:0];
                //    end
                //end
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

//interrupt related registers
wire    [10:0]  irq_mask_n; //interrupt mask register: (MSB)empty, full, adc, ncntrcin, cntr1, cntr0, pint1, nint2, timer1, timer0(LSB), 1'b1
reg             irq_enabled;

//interrupt sources(wire)
wire            is_NMI, is_TIMER0, is_TIMER1, is_pINT1, is_nINT2, is_CNTR0, is_CNTR1, is_nCNTRCIN, is_ADC, is_BUFFULL, is_BUFEMPTY;
wire    [10:0]  is = {is_BUFEMPTY, is_BUFFULL, is_ADC, is_nCNTRCIN, is_CNTR1, is_CNTR0, is_nINT2, is_pINT1, is_TIMER1, is_TIMER0, is_NMI};



//interrupt flags
wire    [10:0]  iflag; assign iflag[10:2] = 10'd0;
/*
wire            iflag_NMI       = iflag[0]; //nNMI physical pin input, takes maximum 10us to suppress glitch
wire            iflag_TIMER0    = iflag[1]; //timer 0/1 match interrupt
wire            iflag_TIMER1    = iflag[2]; 
wire            iflag_pINT1     = iflag[3]; //INT1, nINT2 physical pin input, takes 12+2 mcuclk cycles to suppress glitch
wire            iflag_nINT2     = iflag[4]; 
wire            iflag_CNTR0     = iflag[5]; //timer/event counter 0/1 match interrupt
wire            iflag_CNTR1     = iflag[6]; 
wire            iflag_nCNTRCIN  = iflag[7]; //falling edge of the timer/event countr input (CI input) or timer output (TO) -> from the datasheet
wire            iflag_ADC       = iflag[8]; //adc conversion complete
wire            iflag_BUFFULL   = iflag[9]; //UART buffer full/empty
wire            iflag_BUFEMPTY  = iflag[10];
*/

//softi/hardi processing cycle
wire            softi_proc_cyc = opcode_page == 3'd0 && reg_OPCODE == 8'h72;
wire            hardi_proc_cyc = opcode_page == 3'd0 && reg_OPCODE == 8'h73;

//A user should ack an iflag manually when both interrupt sources belonging to a same irq level are not masked
wire            iflag_manual_ack = mc_type == MCTYPE2 && mc_bk_skip_ctrl[2:1] == 2'b11; 

//An iflag is acknowledged when the HARDI instruction starts
wire            iflag_auto_ack = hardi_proc_cyc && mc_end_of_instruction;

//test
reg nmi = 1'b0;
reg t0 = 1'b0;
initial begin
    #832 nmi = 1'b1; t0 = 1'b0;
    #10  nmi = 1'b0; t0 = 1'b0;

    //#200 t0 = 1'b1;
    //#10 t0 = 1'b0;
end

//interrupt priority
wire    [5:0]   masked_irq =   { iflag[0] & irq_mask_n[0],
                                (iflag[1] & irq_mask_n[1]) | (iflag[2] & irq_mask_n[2]),
                                (iflag[3] & irq_mask_n[3]) | (iflag[4] & irq_mask_n[4]),
                                (iflag[5] & irq_mask_n[5]) | (iflag[6] & irq_mask_n[6]),
                                (iflag[7] & irq_mask_n[7]) | (iflag[8] & irq_mask_n[8]),
                                (iflag[9] & irq_mask_n[9]) | (iflag[10] & irq_mask_n[10])};

reg     [2:0]   irq_lv;
always @(*) begin
    casez(masked_irq)
        6'b1?????: irq_lv = 3'd7; //NMI
        6'b01????: irq_lv = 3'd6;
        6'b001???: irq_lv = 3'd5;
        6'b0001??: irq_lv = 3'd4;
        6'b00001?: irq_lv = 3'd3;
        6'b000001: irq_lv = 3'd2;
        6'b000000: irq_lv = 3'd1; //spurious interrupt
        default: irq_lv = 3'd0;
    endcase
end

//note that interrupt sampler uses an independent divided clock
//NMI interrupt flag set/reset
IKA87AD_iflag u_nmi         (mrst_n, emuclk, mcuclk_pcen, cycle_tick, nmi, 1'b1, 5'd0, reg_OPCODE[4:0], 
                            1'b0, 1'b0, iflag_auto_ack && irq_lv == 3'd7, iflag[0]);

//Timer0/1 interrupt flag set/reset
IKA87AD_iflag u_timer0      (mrst_n, emuclk, mcuclk_pcen, cycle_tick, t0, irq_mask_n[0], 5'd1, reg_OPCODE[4:0], 
                            &{irq_mask_n[2:1]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd6, iflag[1]);
IKA87AD_iflag u_timer1      (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[2], irq_mask_n[1], 5'd2, reg_OPCODE[4:0], 
                            &{irq_mask_n[2:1]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd6, iflag[2]);

//interrupt generation
reg     [2:0]   irq_lv_z;
reg             irq_pending;
wire            irq_detected = irq_pending & irq_enabled;
always @(posedge emuclk) if(mcuclk_pcen) begin
    irq_lv_z <= irq_lv;
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        irq_pending <= 1'b0;
    end
    else begin
        if(irq_pending) begin
            if(cycle_tick) if(iflag_auto_ack) irq_pending <= 1'b0; 
        end
        else begin
            if(mcuclk_pcen) if((irq_lv != 3'd1) && (irq_lv != irq_lv_z)) irq_pending <= 1'b1;
        end
    end
end

//1st(RD4) cycle of the special hardi insturction
reg             force_exec_hardi;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        force_exec_hardi <= 1'b0; 
    end
    else begin if(cycle_tick) begin
        if(irq_detected && mc_end_of_instruction) force_exec_hardi <= 1'b1;
        else force_exec_hardi <= 1'b0;
    end end
end
    
//interrupt enable(1), disable(0)
always @(posedge emuclk) begin
    if(!mrst_n) irq_enabled <= 1'b0; 
    else begin if(cycle_tick) begin
        if(hardi_proc_cyc) irq_enabled <= 1'b0;
        else begin
            if(mc_type == MCTYPE2 && mc_bk_irq_ctrl == 1'b1) irq_enabled <= ~reg_OPCODE[4];
        end
    end end
end

//interrupt routine address
reg     [15:0]  irq_addr;
wire    [15:0]  spurious_irq_addr;
always @(*) begin
    if(softi_proc_cyc) irq_addr = 16'h0060; //SOFTI
    else begin
        case(irq_lv)
            3'd7: irq_addr = 16'h0004; //NMI
            3'd6: irq_addr = 16'h0008; //TIMER
            3'd5: irq_addr = 16'h0010; //INT PIN
            3'd4: irq_addr = 16'h0018; //COUNTER RELATED
            3'd3: irq_addr = 16'h0020; //ADC
            3'd2: irq_addr = 16'h0028; //SERIAL INTERFACE
            3'd1: irq_addr = spurious_irq_addr; //no interrupt
            3'd0: irq_addr = spurious_irq_addr; //no interrupt
        endcase
    end
end

//interrupt flag selector
reg             iflag_muxed;



///////////////////////////////////////////////////////////
//////  SUSPENSION CONTROL
////


//stop pin sync chain
reg     [1:0]       stop_syncchain;
always @(posedge emuclk) if(mcuclk_pcen) begin
    stop_syncchain[0] <= ~i_STOP_n;
    stop_syncchain[1] <= stop_syncchain[0];
end

//force exec nop
reg             force_exec_nop;
always @(posedge emuclk) begin
    if(!mrst_n) force_exec_nop <= 1'b0;
    else begin if(cycle_tick) begin
        if(mc_end_of_instruction) force_exec_nop <= (mc_type == MCTYPE2 && mc_bk_cpu_susp && opcode_page == 3'd1 && reg_OPCODE == 8'h3B);
    end end
end

//hardware stop mode release counter, counts up to 780,000
reg     [19:0]      hstop_osc_wait;
reg                 hstop_osc_unstable;
always @(posedge emuclk) if(mcuclk_pcen) begin
    if(stop_syncchain) begin
        hstop_osc_unstable <= 1'b1;
        hstop_osc_wait <= 20'd0;
    end
    else begin
        if(hstop_osc_wait == 20'd780000) begin
            hstop_osc_unstable <= 1'b0;
        end
        else begin
            hstop_osc_wait <= hstop_osc_unstable + 20'd1;
        end
    end
end

//halt and stop, can be halt insturction without stopping chip's oscillator, this core will stop the timing generator
always @(posedge emuclk) begin
    if(!mrst_n) begin
        soft_halt <= 1'b0;
        soft_stop <= 1'b0;
        hard_stop <= 1'b0;
    end
    else begin
        if(soft_halt) begin
            if(mcuclk_pcen) if((irq_lv != 3'd1) && (irq_lv != irq_lv_z)) soft_halt <= 1'b0;
        end
        else begin
            if(cycle_tick) if(mc_type == MCTYPE2 && mc_bk_cpu_susp && opcode_page == 3'd1 && reg_OPCODE == 8'h3B) soft_halt <= 1'b1;
        end
        
        if(soft_stop) begin
            
        end
        else begin
            if(cycle_tick) if(mc_type == MCTYPE2 && mc_bk_cpu_susp && opcode_page == 3'd1 && reg_OPCODE == 8'hBB) soft_stop <= 1'b1;
        end

        if(hard_stop) begin

        end
        else begin
            if(cycle_tick) if(mc_end_of_instruction) hard_stop <= stop_syncchain;
        end
    end
end



///////////////////////////////////////////////////////////
//////  MICROCODE ENGINE
////

//opcode page indicator
always @(posedge emuclk) begin
    if(!mrst_n) opcode_page <= 3'd0;
    else begin
        if(cycle_tick) if(mc_next_bus_acc == RD4) begin
            if(opcode_page == 3'd0) begin
                     if(reg_OPCODE == 8'h48) opcode_page <= 3'd1;
                else if(reg_OPCODE == 8'h60) opcode_page <= 3'd2;
                else if(reg_OPCODE == 8'h64) opcode_page <= 3'd3;
                else if(reg_OPCODE == 8'h70) opcode_page <= 3'd4;
                else if(reg_OPCODE == 8'h74) opcode_page <= 3'd5;
            end
            else begin
                opcode_page <= 3'd0; //2-byte opcode ended, reset opcode page
            end
        end
    end
end

//microcode address counter
localparam WAIT_FOR_DECODING = 1'b0;
localparam RUNNING = 1'b1;

reg             mseq_state;      //microsequencer state
reg     [3:0]   mseq_susp_timer; //microsequencer suspension timer
reg     [2:0]   mseq_cntr;       //microsequencer counter, registered output
reg     [2:0]   mseq_cntr_next;  //microsequencer counter, "next" combinational output
always @(posedge emuclk) begin
    if(!mrst_n) begin
        mseq_state <= WAIT_FOR_DECODING;
        mseq_susp_timer <= 4'd0;
        mseq_cntr <= 3'd0;
    end
    else begin
        if(mcrom_read_tick) begin
            if(mseq_state == RUNNING) begin
                if(mc_next_bus_acc == RD4) begin
                    mseq_state <= WAIT_FOR_DECODING;
                    mseq_cntr <= mseq_cntr;

                    mseq_susp_timer <= 4'd0;
                end
                else begin
                    mseq_state <= mseq_state;
                    mseq_cntr <= mseq_cntr_next;

                    if(mc_type == MCTYPE3 && mc_s_nop[4]) begin
                        if(mseq_susp_timer == mc_s_nop[3:0]) mseq_susp_timer <= 4'd0;
                        else mseq_susp_timer <= mseq_susp_timer + 4'd1;
                    end
                    else begin
                        mseq_susp_timer <= 4'd0;
                    end
                end
            end
            else begin
                mseq_state <= RUNNING;
                mseq_cntr <= mcrom_sa[2:0];
            end
        end
    end
end

always @(*) begin
    if(mc_type == MCTYPE3 && mc_s_nop[4]) begin
        if(mseq_susp_timer == mc_s_nop[3:0]) mseq_cntr_next = mseq_cntr + 3'd1;
        else mseq_cntr_next = mseq_cntr;
    end
    else if(mc_type == MCTYPE3 && mc_s_bra_on_alu != 3'd0 ) begin
        if(is_arith_eval_op) mseq_cntr_next = mseq_cntr + mc_s_bra_on_alu + 3'd1;
        else mseq_cntr_next = mseq_cntr + 3'd1;
    end
    else begin
        mseq_cntr_next = mseq_cntr + 3'd1;
    end
end

reg     [7:0]   mcrom_addr;
always @(*) begin
    if(mseq_state == WAIT_FOR_DECODING) mcrom_addr = mcrom_sa;
    else mcrom_addr = mc_end_of_instruction ? IRD : {mcrom_sa[7:3], mseq_cntr_next};
end



///////////////////////////////////////////////////////////
//////  MICROCODE ROM
////

IKA87AD_microcode u_microcode (
    .i_CLK                      (emuclk                     ),
    .i_MCROM_READ_TICK          (mcrom_read_tick            ),
    .i_MCROM_ADDR               (mcrom_addr                 ),
    .o_MCROM_DATA               (mcrom_data                 )
);

/*
    MICROCODE TYPE DESCRIPTION

    1. ALU-REGISTER 1
    00_X_X_XXXXX_XXXXX_XX_XX

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
        01011: (w) MD_spare(MDI)
        01100:
        01101:
        01110: (b) A
        01111: (w) EA
        10000: (w) ADDR_IM
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
    D[8:4] source A, destination register type, decoded by the external circuit, :
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
        1X: ALU operation - OPCODE[6:3] / OPCODE[6:4], OPCODE[0] (single byte inst) 
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
    1010: NO_SOURCE
    1011:
    1100: 
    1101: 
    1110:
    1111: (w) *RPA
    D[9:6] source C, destination
    0000: (b) r2
    0001: (b) A
    0010: (w) EA
    0011: (b) MD_low_byte
    0100  (b) MD_high_byte
    0101: (w) MD_word
    0110: (w) MA
    0111: (b) PSW
    1000: (w) BC
    1001:     PC
    1010:     
    1011: 
    1100: 
    1101: 
    1110: 
    1111:     NOWHERE

    D[5:2] ALU operation type:
    0000: bypass
    0001: NEGA(negate)
    0010: DAA(what the fuck is that)
    0011: RLD(rotate left digit)
    0100: MUL
    0101: DIV
    0110:
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
    10_X_X_-_-_X_X_X_XX_XX_XXX_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13:11]: reserved
    D[10]: save special register address
    D[9]: CARRY MOD
    D[8]: INTERRUPT E/D
    D[7:6]: EXCHANGE
        00: NOP
        01: EXX
        10: EXA
        11: EXH
    D[5]: CPU control - suspension
    D[4:2]: SKIP control
        000: NOP
        001: reserved
        010: reserved
        011: BIT
        100: SK
        101: SKN
        110: SKIT
        111: SKNIT
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read

    4. SPECIAL OPERATION
    11_X_X_XXXXX_X_X_XXX_X_?_XX
    D[17:16]: instruction type bit
    D[15]: FLAG bit
    D[14]: SKIP bit
    D[13]: nop
    D[12:9]: nop cycles 0=>1cycle, 15=16cycles
    D[8]: conditional PC decrement(BLOCK)
    D[7]: conditional read(rpa+byte or register)
    D[6:4]: conditional branch on ALU type, branch+ steps
    D[3]: swap MD output order
    D[2]: 1st cycle of 2-byte instruction
    D[1:0] current bus transaction type :
        00: IDLE
        01: 3-state read
        10: 3-state write
        11: 4-state read

    nop = 11_0_0_10000_0_0_000_0_0_XX
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

wire            reg_EAL_wr = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_EA) ||                      //direct designation
                             (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_EA) ||                      //direct designation
                             (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd1) || //rp1 word
                             (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd4) || //rp2 word
                             (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd4) || //rp1 word
                             alu_muldiv_reg_EA_wr;

wire            reg_EAH_wr = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_EA) ||                      //direct designation
                             (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_EA) ||                      //direct designation
                             (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd0) || //r1 byte
                             (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd4) || //rp2 word
                             (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd4) || //rp1 word
                             alu_muldiv_reg_EA_wr;
                             
wire            reg_V_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd0) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd0);   //rp1 word

wire            reg_A_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_A) ||                       //direct designation
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_A) ||                       //direct designation
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd1) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd1) || //r2 byte(mc0)
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd1) || //r2 byte(mc1)
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd0);   //rp1 word

wire            reg_B_wr  = (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_BC) ||                      //direct designation
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd2) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd2) || //r2 byte(mc0)
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd2) || //r2 byte(mc1)
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd2) || //r1 byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd1) || //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd1) || //rp word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd1);   //rp1 word

wire            reg_C_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_C)  ||                      //direct designation
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_BC) ||                      //direct designation
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd3) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R2  && r2_addr  == 2'd3) || //r2 byte(mc0)
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_R2  && r2_addr  == 2'd3) || //r2 byte(mc1)
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd3) || //r1 byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd1) || //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd1) || //rp word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd1);   //rp1 word

wire            reg_D_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd4) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd4) || //r1 byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd2) || //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd2) || //rp word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd2) || //rp1 word
                            (mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd2)  || //rpa auto inc/dec condition
                            (mc_type == MCTYPE1 && mc_sd == SD_DE  && mc_t1_alusel[3:1] == 3'b100); //manual inc/dec

wire            reg_E_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd5) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd5) || //r1 byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd2) || //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd2) || //rp word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd2) || //rp1 word
                            (mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd2)  || //rpa auto inc/dec condition
                            (mc_type == MCTYPE1 && mc_sd == SD_DE  && mc_t1_alusel[3:1] == 3'b100); //manual inc/dec

wire            reg_H_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd6) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd6) || //r1 byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd3) || //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd3) || //rp word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd3) || //rp1 word
                            (mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd3)  || //rpa auto inc/dec condition
                            (mc_type == MCTYPE1 && mc_sd == SD_HL  && mc_t1_alusel[3:1] == 3'b100); //manual inc/dec

wire            reg_L_wr  = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R   && r_addr   == 3'd7) || //r byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_R1  && r1_addr  == 3'd7) || //r1 byte
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd3) || //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd3) || //rp word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1 && rp1_addr == 3'd3) || //rp1 word
                            (mc_type == MCTYPE1 && mc_sd == SD_RPA && rpa_incdec_addr == 2'd3)  || //rpa auto inc/dec condition
                            (mc_type == MCTYPE1 && mc_sd == SD_HL  && mc_t1_alusel[3:1] == 3'b100); //manual inc/dec

wire            data_w_nb = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_EA ) || //direct designation
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2) || 
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP ) || 
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP1) || 
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_MD ) || //direct designation
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_EA ) || //direct designation
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_BC ) || //direct designation
                            (mc_type == MCTYPE1 && mc_t1_alusel == 4'b1000) || //addr dec
                            (mc_type == MCTYPE1 && mc_t1_alusel == 4'b1001) || //addr inc(BLOCK)
                            (mc_type == MCTYPE1 && mc_t1_alusel == 4'b1010) || //rpa auto inc/dec
                            alu_muldiv_reg_EA_wr;

//PC/SP
wire            reg_PC_wr = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_PC) ||
                            (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_PC) ||
                            (mc_type == MCTYPE3 && mc_s_cond_pc_dec);
wire            reg_SP_wr = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_SP ) ||                      //direct designation
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP2 && rp2_addr == 3'd0) ||  //rp2 word
                            (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_RP  && rp_addr  == 2'd0) ||
                            (mc_type == MCTYPE1 && mc_sd == SD_SP  && mc_t1_alusel[3:1] == 3'b100); //manual inc/dec

//Memory IO related registers, MA=Memory Address, MD=Memory Data
wire            reg_MA_dec_mode = mc_type == MCTYPE1 && mc_t1_alusel == 4'h8;
wire            reg_MA_wr     = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_MA) ||
                                (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_MA);

wire            reg_MDL_wr_A  = (mc_type == MCTYPE0 && mc_sb == SB_RPA2 && mc_sa_dst == SA_DST_MA && opcode_page == 3'd0) || //STAX
                                (mc_type == MCTYPE1 && mc_sd == SD_RPA  && mc_sc_dst == SC_DST_MA && opcode_page == 3'd0);   //STAX
wire            reg_MD_wr_EA  = (mc_type == MCTYPE0 && mc_sb == SB_RPA2 && mc_sa_dst == SA_DST_MA && opcode_page == 3'd1) || //STEAX
                                (mc_type == MCTYPE1 && mc_sd == SD_RPA  && mc_sc_dst == SC_DST_MA && opcode_page == 3'd1);   //STEAX
wire            reg_MD_wr_PC  = (mc_type == MCTYPE1 && mc_sd == SD_SP && mc_sc_dst == SC_DST_MA && mc_t1_alusel == 4'b1000); 
wire            reg_MDL_wr    = (mc_type == MCTYPE0 && (mc_sa_dst == SA_DST_MDL || mc_sa_dst == SA_DST_MD)) || 
                                (mc_type == MCTYPE1 && (mc_sc_dst == SC_DST_MDL || mc_sc_dst == SC_DST_MD));
wire            reg_MDH_wr    = (mc_type == MCTYPE0 && mc_sa_dst == SA_DST_MD) || 
                                (mc_type == MCTYPE1 && (mc_sc_dst == SC_DST_MDH || mc_sc_dst == SC_DST_MD));

//swaps MD output order, push to stack or V_wa addressing
wire            reg_MD_output_h2l = (mc_type == MCTYPE3 && mc_s_swap_md_out) || 
                                    (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_MA && mc_t1_alusel == 4'b1000); 
wire            reg_MD_output_l2h = (mc_type == MCTYPE1 && mc_sd == SD_PSW);
wire            reg_MD_input_h2l  = mc_type == MCTYPE3 && mc_s_cond_read && mc_next_bus_acc == RD3;

//status register(flag restoration)
wire            reg_PSW_wr    = (mc_type == MCTYPE1 && mc_sc_dst == SC_DST_PSW);

//ALU control
wire            alu_mul_start = mc_type == MCTYPE1 && mc_t1_alusel == 4'b0100;
wire            alu_div_start = mc_type == MCTYPE1 && mc_t1_alusel == 4'b0101;



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
        if(cycle_tick) begin
            if(mc_type == MCTYPE2) begin
                case(mc_bk_reg_exchg)
                    2'b00: ;
                    2'b01: flag_EXX <= ~flag_EXX;
                    2'b10: flag_EXA <= ~flag_EXA;
                    2'b11: flag_EXH <= ~flag_EXH;
                endcase
            end
        end
    end
end

//register pairs and write control
reg     [7:0]   regpair_EAH[0:1], regpair_EAL[0:1], 
                regpair_V[0:1]  , regpair_A[0:1]  , 
                regpair_B[0:1]  , regpair_C[0:1]  ,
                regpair_D[0:1]  , regpair_E[0:1]  , 
                regpair_H[0:1]  , regpair_L[0:1]  ;
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
        if(cycle_tick) begin
            if(reg_EAH_wr) regpair_EAH[sel_VAEA] <= data_w_nb ? alu_output[15:8] : alu_output[7:0];
            if(reg_EAL_wr) regpair_EAL[sel_VAEA] <= alu_output[7:0];
            if(reg_V_wr)   regpair_V[sel_VAEA]   <= data_w_nb ? alu_output[15:8] : alu_output[7:0];
            if(reg_A_wr)   regpair_A[sel_VAEA]   <= alu_output[7:0];
            if(reg_B_wr)   regpair_B[sel_BCDE]   <= data_w_nb ? alu_output[15:8] : alu_output[7:0];
            if(reg_C_wr)   regpair_C[sel_BCDE]   <= alu_output[7:0];
            if(reg_D_wr)   regpair_D[sel_BCDE]   <= data_w_nb ? alu_output[15:8] : alu_output[7:0];
            if(reg_E_wr)   regpair_E[sel_BCDE]   <= alu_output[7:0];
            if(reg_H_wr)   regpair_H[sel_HL]     <= data_w_nb ? alu_output[15:8] : alu_output[7:0];
            if(reg_L_wr)   regpair_L[sel_HL]     <= alu_output[7:0];
        end
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

reg     [15:0]  reg_MDI; //Memory Data input spare
reg     [7:0]   reg_MDH, reg_MDL; //byte [15:8], word[15:0]
reg     [15:0]  reg_TEMP;


//
//  Special registers
//

reg     [6:0]   reg_MKL; //intrq disable register low ; ncntrcin, cntr1, cntr0, pint1, nint2, timer1, timer0, -
reg     [2:0]   reg_MKH; //intrq disable register high; -, -, -, -, -, empty, full, adc
assign irq_mask_n = ~{reg_MKH, reg_MKL, 1'b0};

always @(posedge emuclk) begin
    if(!mrst_n) begin
        reg_MKL <= 7'b1111110;
        reg_MKH <= 3'b111;
    end
    /*
    else begin
        if(cycle_tick) begin
            
        end
    end
    */
end


//
//  Flags
//

reg             flag_Z, flag_SK, flag_C, flag_HC, flag_L1, flag_L0;
wire    [7:0]   reg_PSW = {1'b0, flag_Z, flag_SK, flag_HC, flag_L1, flag_L0, 1'b0, flag_C};


//
//  PC, SP, MA registers with auto increment/decrement feature
//

reg     [15:0]  reg_PC, reg_SP, reg_MA;
reg     [15:0]  next_pc;
assign  spurious_irq_addr = reg_PC;

//address source selector
localparam PC = 1'b0;
localparam MA = 1'b1;
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
                if(reg_MA_wr) begin
                    address_source_sel <= MA; //select MA
                    reg_PC_inc_stop <= 1'b1;
                end

                if(reg_MA_dec_mode) reg_MA_inc_ndec <= 1'b0;
            end
        end
    end

    //REGISTERS
    if(!mrst_n) begin
        reg_PC <= 16'hFFFF;
        reg_SP <= 16'h0000;
        reg_MA <= 16'h0000;
        reg_TEMP <= 16'h0000;
    end
    else begin
        if(cycle_tick) begin
            //Program Counter load/auto increment conditions
            if(reg_PC_wr) reg_PC <= alu_output;
            else reg_PC <= next_pc;

            //Stack Pointer load condition
            if(reg_SP_wr) reg_SP <= alu_output;

            //Memory Address load/auto inc conditions
            if(reg_MA_wr) reg_MA <= alu_ma_output;
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

            if(reg_TEMP_wr) reg_TEMP <= alu_temp_output;
        end
        else if(opcode_inlatch_tick) begin
            reg_MA <= reg_PC;
        end
    end
end

always @(*) begin
    if(reg_PC_inc_stop) next_pc = reg_PC;
    else begin
        if(force_exec_hardi | force_exec_nop) next_pc = reg_PC;
        else begin
            if(current_bus_acc == RD4 || current_bus_acc == RD3) next_pc = reg_PC == 16'hFFFF ? 16'h0000 : reg_PC + 16'h0001;
            else next_pc = reg_PC;
        end
    end

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
                if(reg_MD_output_h2l) md_out_byte_sel <= 1'b1;
                else if(reg_MD_output_l2h) md_out_byte_sel <= 1'b0;
                else begin
                    if(current_bus_acc == WR3) md_out_byte_sel <= ~md_out_byte_sel;
                end

                if(reg_MD_input_h2l) md_in_byte_sel <= 1'b1; //for rpa2 DE/HL+byte addressing mode
                else begin
                    if(current_bus_acc == RD3) md_in_byte_sel <= ~md_in_byte_sel;
                end
            end
        end
    end
end


//OPCODE/memory data IO
reg             md_dirty;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        reg_MDI <= 16'h0000;
        reg_MDL <= 8'h00;
        reg_MDH <= 8'h00;
        reg_OPCODE <= 8'h00;

        //when microcode writes data prior to bus data read cycle, dirty bit is set
        //this bit prevents MD register from being overwritten by bus data
        //CALL CALF instructions use this feature
        md_dirty <= 1'b0; 
    end
    else begin
        if(mcuclk_pcen) begin
            if(cycle_tick) begin
                if(mc_end_of_instruction) md_dirty <= 1'b0;
                else begin
                    if(reg_MDL_wr || reg_MDH_wr) md_dirty <= 1'b1;
                end
            end
            //Memory Data register load
            if(cycle_tick) begin
                if(mc_end_of_instruction) begin
                    reg_MDI <= 16'h0000;
                    reg_MDH <= 8'h00;
                    reg_MDL <= 8'h00;
                end
                else begin
                    if(reg_MDL_wr_A) begin //save A to MDL(rpa2, stax)
                        reg_MDL <= reg_A;
                    end
                    else if(reg_MD_wr_EA) begin //save EA to MD(rpa2, steax)
                        reg_MDH <= reg_EAH;
                        reg_MDL <= reg_EAL;
                    end
                    else if(reg_MD_wr_PC) begin
                        reg_MDH <= next_pc[15:8];
                        reg_MDL <= next_pc[7:0];
                    end
                    else begin
                        if(reg_MDH_wr) reg_MDH <= data_w_nb ? alu_output[15:8] : alu_output[7:0];
                        if(reg_MDL_wr) reg_MDL <= alu_output[7:0];
                    end
                end
            end
            else if(md_inlatch_tick) begin
                if(md_dirty) begin
                    if(md_in_byte_sel) reg_MDI[15:8] <= i_PD_I;
                    else reg_MDI[7:0] <= i_PD_I;
                end
                else begin
                    if(md_in_byte_sel) reg_MDI[15:8] <= i_PD_I;
                    else reg_MDI[7:0] <= i_PD_I;

                    if(md_in_byte_sel) reg_MDH <= i_PD_I;
                    else reg_MDL <= i_PD_I;
                end
            end

            //Opcode register load
            if(opcode_inlatch_tick) begin
                     if(force_exec_hardi) reg_OPCODE <= 8'h73;
                else if(force_exec_nop)   reg_OPCODE <= 8'h00;
                else                      reg_OPCODE <= i_PD_I;
            end
        
            //Full opcode register for the disassembler
            if(cycle_tick) begin if(mc_end_of_instruction) reg_FULL_OPCODE_cntr <= 2'd0; end
            else if(full_opcode_inlatch_tick_debug) begin
                reg_FULL_OPCODE_debug[reg_FULL_OPCODE_cntr] <= force_exec_hardi ? 8'h73 : i_PD_I;
                reg_FULL_OPCODE_cntr <= reg_FULL_OPCODE_cntr + 2'd1;
            end
        end
    end
end


//address high, multiplexed address low/byte data output
wire    [7:0]   md_out_byte_data = md_out_byte_sel == 1'b1 ? reg_MDH : reg_MDL;
wire    [7:0]   addr_hi_out = memory_access_address[15:8];
wire    [7:0]   addr_lo_data_out = addr_data_sel ? md_out_byte_data : memory_access_address[7:0];

//FOR DEBUGGING
assign  o_FULL_ADDRESS_DEBUG = memory_access_address;
assign  o_OUTPUT_DATA_DEBUG = md_out_byte_data;


//ALE, /RD, /WR
reg             ale_out, rd_out, wr_out, pd_oe;
assign o_ALE = ale_out;
assign o_RD_n = ~rd_out;
assign o_WR_n = ~wr_out;

always @(posedge emuclk) begin
    if(!mrst_n) begin
        ale_out <= 1'b0;
        rd_out <= 1'b0;
        wr_out <= 1'b0;
        pd_oe <= 1'b0;
    end
    else begin
        if(mcuclk_pcen) begin
            if(cycle_tick) begin
                if(mc_end_of_instruction && (irq_detected | (mc_type == MCTYPE2 && mc_bk_cpu_susp && opcode_page == 3'd1 && reg_OPCODE == 8'h3B))) begin
                    ale_out <= 1'b0;
                    pd_oe <= 1'b0;
                end
                else begin
                    if(mc_next_bus_acc != IDLE) begin
                        ale_out <= 1'b1;
                        pd_oe <= 1'b1;
                    end
                end
            end
            else begin
                if(!(force_exec_hardi | force_exec_nop)) begin
                    //ALE off
                    if(timing_sr[1]) ale_out <= 1'b0;

                    //PD data OE off
                    if(current_bus_acc == RD3 || current_bus_acc == RD4) begin
                        if(timing_sr[2]) pd_oe <= 1'b0;
                    end

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
                end

                //WR control
                if(current_bus_acc == WR3) begin
                    if(timing_sr[2]) wr_out <= 1'b1;
                    else if(timing_sr[6]) wr_out <= 1'b0;
                end
                else wr_out <= 1'b0;
            end
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
        3'b100: reg_RP2 = {reg_EAH, reg_EAL};
        3'b101: reg_RP2 = 16'h0000; //not specified on the datasheet
        3'b110: reg_RP2 = 16'h0000;
        3'b111: reg_RP2 = 16'h0000;
    endcase

    case(reg_OPCODE[2:0])
        3'b000: reg_RP1 = {reg_V, reg_A};
        3'b001: reg_RP1 = {reg_B, reg_C};
        3'b010: reg_RP1 = {reg_D, reg_E};
        3'b011: reg_RP1 = {reg_H, reg_L};
        3'b100: reg_RP1 = {reg_EAH, reg_EAL};
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
        3'b011: reg_RPA2_OFFSET = {8'h00, reg_MDH}; //use alu type 0 to select an addend automatically
        3'b100: reg_RPA2_OFFSET = {8'h00, reg_A}; 
        3'b101: reg_RPA2_OFFSET = {8'h00, reg_B};
        3'b110: reg_RPA2_OFFSET = {reg_EAH, reg_EAL};
        3'b111: reg_RPA2_OFFSET = {8'h00, reg_MDH};
    endcase
end

//ALU port A and B
reg     [15:0]  alu_pa, alu_pb; 
always @(*) begin
    alu_pa = 16'h0000; alu_pb = 16'h0000;

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
            SB_MDI        : alu_pb = reg_MDI;
            SB_A          : alu_pb = {8'h00, reg_A};
            SB_EA         : alu_pb = {reg_EAH, reg_EAL};
            SB_ADDR_V_WA  : alu_pb = {reg_V, reg_MDI[7:0]};
            SB_ADDR_TA    : alu_pb = {8'h00, 2'b10, reg_OPCODE[4:0], 1'b0};
            SB_ADDR_FA    : alu_pb = {5'b00001, reg_OPCODE[2:0], reg_MDI[7:0]};
            SB_ADDR_REL_S : alu_pb = {{11{reg_OPCODE[5]}}, reg_OPCODE[4:0]}; //sign extension
            SB_ADDR_REL_L : alu_pb = {{8{reg_OPCODE[0]}}, reg_MDI[7:0]};
            SB_ADDR_INT   : alu_pb = irq_addr; //selected externally
            SB_SUB2       : alu_pb = 16'hFFFE;
            SB_SUB1       : alu_pb = 16'hFFFF;
            SB_ZERO       : alu_pb = 16'h0000;
            SB_ADD1       : alu_pb = 16'h0001;
            SB_ADD2       : alu_pb = 16'h0002;
            SB_TEMP       : alu_pb = reg_TEMP;
            SB_RPA1       : alu_pb = reg_RPA;
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
            SC_DST_MDH    : alu_pa = {8'h00, reg_MDH};
            SC_DST_MD     : alu_pa = {reg_MDH, reg_MDL};
            SC_DST_MA     : alu_pa = reg_MA;
            SC_DST_PSW    : alu_pa = reg_PSW;
            SC_DST_BC     : alu_pa = {reg_B, reg_C};
            SC_DST_PC     : alu_pa = reg_PC;
            default       : alu_pa = 16'h0000;
        endcase

        case(mc_sd)
            SD_A          : alu_pb = reg_A;
            SD_EA         : alu_pb = {reg_EAH, reg_EAL};
            SD_BC         : alu_pb = {reg_B, reg_C};
            SD_DE         : alu_pb = {reg_D, reg_E};
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
        if(mc_s_cond_pc_dec) begin
            alu_pa = reg_PC;
            alu_pb = reg_C == 8'hFF ? 16'h0000 : 16'hFFFF;
        end
        else begin
            alu_pa = 16'h0000;
            alu_pb = 16'h0000;
        end
    end
end




///////////////////////////////////////////////////////////
//////  ALU
////

//
//  ALU: full adder with nibble, byte, word c_comb outputs
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

assign  alu_adder_nibble_lo = alu_adder_op0[3:0] + alu_adder_op1[3:0] + alu_adder_cin;
assign  alu_adder_nibble_hi = alu_adder_op0[7:4] + alu_adder_op1[7:4] + alu_adder_nibble_co;
assign  alu_adder_byte_high = alu_adder_op0[15:8] + alu_adder_op1[15:8] + alu_adder_byte_co;

assign  alu_adder_out[3:0] = alu_adder_nibble_lo[3:0];
assign  alu_adder_out[7:4] = alu_adder_nibble_hi[3:0];
assign  alu_adder_out[15:8] = alu_adder_byte_high[7:0];


//
//  ALU: shifter and rotator
//

reg     [15:0]  alu_shifter;
reg             alu_shifter_co;
always @(*) begin
    alu_shifter = 16'h00;
    alu_shifter_co = 1'b0;

    if(mc_type == MCTYPE1) if(mc_t1_alusel == 4'h7) begin
        case(shift_code)
            4'b0000: begin alu_shifter[7] = 1'b0; 
                           alu_shifter[6:0] = alu_pa[7:1];
                           alu_shifter_co = alu_pa[0]; end //SLRC, skip condition: CARRY
            4'b0001: ; //no instruction specified
            4'b0010: begin alu_shifter[7] = 1'b0; 
                           alu_shifter[6:0] = alu_pa[7:1];
                           alu_shifter_co = alu_pa[0]; end //SLR
            4'b0011: begin alu_shifter[7] = flag_C;
                           alu_shifter[6:0] = alu_pa[7:1];
                           alu_shifter_co = alu_pa[0]; end //RLR
            4'b0100: begin alu_shifter[0] = 1'b0; 
                           alu_shifter[7:1] = alu_pa[6:0];
                           alu_shifter_co = alu_pa[7]; end //SLLC, skip condition: CARRY
            4'b0101: ; //no instruction specified
            4'b0110: begin alu_shifter[0] = 1'b0; 
                           alu_shifter[7:1] = alu_pa[6:0];
                           alu_shifter_co = alu_pa[7]; end //SLL
            4'b0111: begin alu_shifter[0] = flag_C;
                           alu_shifter[7:1] = alu_pa[6:0];
                           alu_shifter_co = alu_pa[7]; end //RLL

            4'b1000: ; //no instruction specified
            4'b1001: ; //no instruction specified
            4'b1010: begin alu_shifter[15] = 1'b0;
                           alu_shifter[14:0] = alu_pa[15:1];
                           alu_shifter_co = alu_pa[0]; end //DSLR
            4'b1011: begin alu_shifter[15] = flag_C;
                           alu_shifter[14:0] = alu_pa[15:1];
                           alu_shifter_co = alu_pa[0]; end //DRLR
            4'b1100: ; //no instruction specified
            4'b1101: ; //no instruction specified
            4'b1110: begin alu_shifter[0] = 1'b0;
                           alu_shifter[15:1] = alu_pa[14:0];
                           alu_shifter_co = alu_pa[15]; end //DSLL
            4'b1111: begin alu_shifter[0] = flag_C;
                           alu_shifter[15:1] = alu_pa[14:0];
                           alu_shifter_co = alu_pa[15]; end //DRLL
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
            if(alu_mul_start) begin
                if(alu_muldiv_cntr == 5'd31) alu_muldiv_cntr <= 5'd16;
            end
            else if(alu_div_start) begin
                if(alu_muldiv_cntr == 5'd31) alu_muldiv_cntr <= 5'd0;
            end
            else begin
                if(alu_muldiv_cntr != 5'd31) alu_muldiv_cntr <= (alu_muldiv_cntr == 5'd23 || alu_muldiv_cntr == 5'd15) ? 5'd31 : alu_muldiv_cntr + 5'd1;
                else alu_muldiv_cntr <= 5'd31;
            end
        end
    end
end

wire    [15:0]  alu_mul_pa = {reg_EAH, reg_EAL};
wire    [15:0]  alu_mul_pb = reg_R2[alu_muldiv_cntr[2:0]] ? ({8'h00, reg_A} << alu_muldiv_cntr[2:0]) : 16'h0000;

wire    [15:0]  alu_div_pa = reg_TEMP;
wire    [15:0]  alu_div_pb = ~{8'h00, reg_R2};

wire    [31:0]  a = {reg_TEMP, {reg_EAH, reg_EAL}};
wire    [31:0]  b = {alu_adder_out, {reg_EAH, reg_EAL}};
wire    [31:0]  alu_div_out = alu_adder_out[15] ? {a[30:0], 1'b0} :
                                                  {b[30:0], 1'b1};


//
//  ALU: operator
//

always @(*) begin
    //maintain current destination register's data, if the port is not altered
    alu_adder_op0 = 16'h0000; alu_adder_op1 = 16'h0000; alu_adder_cin = 1'b0; //FA inputs
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
            alu_ma_output = alu_pb; //out<-pb bypass
            alu_output = alu_pb;
        end
        else if(mc_t0_alusel == 2'd1) begin
            alu_ma_output = alu_adder_out;
            alu_output = alu_adder_out;
            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin = 1'b0;
        end
        else begin
            case(arith_code)
                4'h0: alu_output = alu_pb;                        //MVI(move)
                4'h1: alu_output = alu_pa ^ alu_pb;               //XOR(bitwise XOR)
                4'h2: begin alu_output = alu_adder_out;           //ADDNC(check skip condition: NO CARRY)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin = 1'b0; end
                4'h3: begin alu_output = alu_adder_out;           //SUBNB(check skip condition: NO BORROW; 2's complement)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
                4'h4: begin alu_output = alu_adder_out;           //ADD
                            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin = 1'b0; end
                4'h5: begin alu_output = alu_adder_out;           //ADD with c_comb
                            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin = flag_C; end
                4'h6: begin alu_output = alu_adder_out;           //SUB
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
                4'h7: begin alu_output = alu_adder_out;           //SUB with borrow
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = ~flag_C; 
                            alu_adder_borrow_mode = 1'b1; end
                4'h8: alu_output = alu_pa & alu_pb;               //AND(bitwise AND)
                4'h9: alu_output = alu_pa | alu_pb;               //OR(bitwise OR)
                4'hA: begin alu_temp_output = alu_adder_out;      //SGT(skip if greater than; PA-PB-1, adding the inverted PB has the same effect)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b0; 
                            alu_adder_borrow_mode = 1'b1; end
                4'hB: begin alu_temp_output = alu_adder_out;      //SLT(skip if less than; check skip condition: BORROW; 2's complement)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b1;
                            alu_adder_borrow_mode = 1'b1; end
                4'hC: alu_temp_output = alu_pa & alu_pb;          //AND(check skip condition: NO ZERO)
                4'hD: alu_temp_output = alu_pa | alu_pb;          //OR(check skip condition: ZERO)
                4'hE: begin alu_temp_output = alu_adder_out;      //SNE(skip on not equal; check skip condition: NO ZERO)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
                4'hF: begin alu_temp_output = alu_adder_out;      //SEQ(skip on equal; check skip condition: ZERO)
                            alu_adder_op0 = alu_pa; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b1; 
                            alu_adder_borrow_mode = 1'b1; end
            endcase
        end
    end
    else if(mc_type == MCTYPE1) begin
        if(mc_t1_alusel == 4'h0) begin 
                alu_ma_output = alu_pb; //out<-pb bypass
                alu_output = alu_pb;
        end
        else if(mc_t1_alusel == 4'h1) begin //2's complement
            alu_output = alu_adder_out;
            alu_adder_op0 = 16'h0000; alu_adder_op1 = ~alu_pb; alu_adder_cin = 1'b1;
        end
        else if(mc_t1_alusel == 4'h2) begin //DAA, kinda shit
            alu_output = alu_adder_out;
            alu_adder_op0 = alu_pa; alu_adder_cin = 1'b0;
            if(flag_HC) begin
                if(flag_C == 1'b0 && alu_pa[7:4] <= 4'h9) alu_adder_op1 = 16'h0006;
                else alu_adder_op1 = 16'h0066;
            end
            else begin
                if(alu_pa[3:0] <= 4'h9) begin
                    if(flag_C == 1'b0 && alu_pa[7:4] <= 4'h9) alu_adder_op1 = 16'h0000;
                    else alu_adder_op1 = 16'h0060;
                end
                else begin
                    if(flag_C == 1'b0 && alu_pa[7:4] <= 4'h9) alu_adder_op1 = 16'h0006;
                    else alu_adder_op1 = 16'h0066;
                end
            end
        end
        else if(mc_t1_alusel == 4'h3) begin //RLD(even) RRD(odd) PA=MDin, PB=reg_A
            alu_output = reg_OPCODE[0] ? {alu_pb[3:0], alu_pa[7:4]} : {alu_pa[3:0], alu_pb[3:0]}; //to MD
            alu_temp_output = reg_OPCODE[0] ? {alu_pb[7:4], alu_pa[3:0]} : {alu_pb[7:4], alu_pa[7:4]}; //to TEMP->A

            alu_digrot_TEMP_wr = 1'b1;
        end
        else if(mc_t1_alusel == 4'h4) begin //MUL pa=EA, pb=r2
            alu_output = 16'h0000; //reset EA
        end
        else if(mc_t1_alusel == 4'h5) begin //DIV
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
            alu_adder_op0 = alu_pa; alu_adder_op1 = 16'hFFFF; alu_adder_cin = 1'b0;
            alu_adder_borrow_mode = 1'b1;

            alu_output = alu_adder_out;
        end
    end
    else if(mc_type == MCTYPE3) begin
        if(mc_s_cond_pc_dec) begin
            alu_adder_op0 = alu_pa; alu_adder_op1 = alu_pb; alu_adder_cin = 1'b0;

            alu_output = alu_adder_out;
        end
        else begin
            if(alu_muldiv_cntr != 5'd31) begin
                if(alu_muldiv_cntr[4]) begin //multiply
                    alu_muldiv_reg_EA_wr = 1'b1;

                    alu_adder_op0 = alu_mul_pa; //reg EA
                    alu_adder_op1 = alu_mul_pb; //A * r2, shifted and masked
                    alu_adder_cin = 1'b0;

                    alu_output = alu_adder_out;
                end
                else begin
                    alu_muldiv_reg_TEMP_wr = alu_muldiv_cntr[3:0] == 4'd15 ? 1'b0 : 1'b1;
                    alu_muldiv_reg_EA_wr = 1'b1;
                    
                    alu_adder_op0 = alu_div_pa;  //reg EA
                    alu_adder_op1 = alu_div_pb; //-r2
                    alu_adder_cin = 1'b1;
                    
                    alu_output = alu_div_out[15:0];
                    alu_temp_output = alu_div_out[31:16];
                end
            end
        end
    end
end




///////////////////////////////////////////////////////////
//////  FLAG GENERATOR
////

//Since the flags are generated as a result of the ALU-type microcode operation
//bits are enabled during execution. This can interrupt the current microcode
//flow. Use two-stage DFF to change the flags "after" the current instruction.

//combinational
reg             z_comb, c_comb, hc_comb, sk_comb;

//temporary latch
reg             z_temp, c_temp, hc_temp, sk_temp;

//z_comb flag
always @(*) begin
    z_comb = flag_Z;

    if(mc_type == MCTYPE0) begin
        if(mc_t0_alusel == 2'd2 || mc_t0_alusel == 2'd3) z_comb = data_w_nb ? alu_output == 16'h0000 : alu_output[7:0] == 8'h00;
    end
    else if(mc_type == MCTYPE1) begin
             if(mc_t1_alusel == 4'h2) z_comb = data_w_nb ? alu_output == 16'h0000 : alu_output[7:0] == 8'h00; //DAA
        else if(mc_t1_alusel == 4'hB) z_comb = data_w_nb ? alu_output == 16'h0000 : alu_output[7:0] == 8'h00; //INC
        else if(mc_t1_alusel == 4'hC) z_comb = data_w_nb ? alu_output == 16'h0000 : alu_output[7:0] == 8'h00; //DEC
    end
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        z_temp <= 1'b0;
        flag_Z <= 1'b0; //reset
    end
    else begin
        if(cycle_tick) begin
            if(reg_PSW_wr) begin
                flag_Z <= reg_MDL[6]; z_temp <= reg_MDL[6];
            end
            else begin
                if(mc_alter_flag) begin
                    if(mc_end_of_instruction) begin 
                        flag_Z <= z_comb; z_temp <= z_comb; 
                    end
                    else z_temp <= z_comb;
                end
                else begin
                    if(mc_end_of_instruction) flag_Z <= z_temp;
                end
            end
        end
    end
end

//c_comb flag
always @(*) begin
    c_comb = flag_C;

    if(mc_type == MCTYPE0) begin
        if(mc_t0_alusel == 2'd2 || mc_t0_alusel == 2'd3) begin
            case(arith_code)
                4'h2: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //ADDNC
                4'h3: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SUBNB
                4'h4: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //ADD
                4'h5: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //ADC
                4'h6: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SUB
                4'h7: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SBB
                4'hA: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SGT
                4'hB: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SLT
                4'hE: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SNE
                4'hF: c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode; //SEQ
                default: c_comb = flag_C;
            endcase
        end
    end
    else if(mc_type == MCTYPE1) begin
        if(mc_t1_alusel == 4'h2) begin //DAA
            c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode;
        end
        else if(mc_t1_alusel == 4'h7) begin //shift 
            c_comb = alu_shifter_co;
        end
        else if(mc_t1_alusel == 4'hB) begin //INC
            c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode;
        end
        else if(mc_t1_alusel == 4'hC) begin //DEC
            c_comb = data_w_nb ? alu_adder_word_co ^ alu_adder_borrow_mode : alu_adder_byte_co ^ alu_adder_borrow_mode;
        end
    end
    else if(mc_type == MCTYPE2) begin
        if(mc_bk_carry_ctrl == 1'b1) c_comb = reg_OPCODE[0];
    end
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        c_temp <= 1'b0;
        flag_C <= 1'b0; //reset
    end
    else begin
        if(cycle_tick) begin
            if(reg_PSW_wr) begin
                flag_C <= reg_MDL[0]; c_temp <= reg_MDL[0];
            end
            else begin
                if(mc_alter_flag) begin
                    if(mc_end_of_instruction) begin flag_C <= c_comb; c_temp <= c_comb; end
                    else c_temp <= c_comb;
                end
                else begin
                    if(mc_end_of_instruction) flag_C <= c_temp;
                end
            end
        end
    end
end

//HALF c_comb flag
always @(*) begin
    hc_comb = flag_HC;

    if(mc_type == MCTYPE0) begin
        if(mc_t0_alusel == 2'd2 || mc_t0_alusel == 2'd3) begin
            case(arith_code)
                4'h2: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //ADDNC
                4'h3: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SUBNB
                4'h4: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //ADD
                4'h5: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //ADC
                4'h6: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SUB
                4'h7: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SBB
                4'hA: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SGT
                4'hB: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SLT
                4'hE: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SNE
                4'hF: hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode; //SEQ
                default: hc_comb = flag_HC;
            endcase
        end
    end
    else if(mc_type == MCTYPE1) begin
        if(mc_t1_alusel == 4'h2) begin //DAA
            hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode;
        end
        else if(mc_t1_alusel == 4'hB) begin //INC
            hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode;
        end
        else if(mc_t1_alusel == 4'hC) begin //DEC
            hc_comb = alu_adder_nibble_co ^ alu_adder_borrow_mode;
        end
    end
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        hc_temp <= 1'b0;
        flag_HC <= 1'b0; //reset
    end
    else begin
        if(cycle_tick) begin
            if(reg_PSW_wr) begin
                flag_HC <= reg_MDL[4]; hc_temp <= reg_MDL[4];
            end
            else begin
                if(mc_alter_flag) begin
                    if(mc_end_of_instruction) begin flag_HC <= hc_comb; hc_temp <= hc_comb; end
                    else hc_temp <= hc_comb;
                end
                else begin
                    if(mc_end_of_instruction) flag_HC <= hc_temp;
                end
            end
        end
    end
end

//SKIP flag
reg             skip_flag;
always @(*) begin
    case(reg_OPCODE[2:0])
        3'b010: skip_flag = flag_C;
        3'b011: skip_flag = flag_HC;
        3'b100: skip_flag = flag_Z;
        default: skip_flag = 1'b0;
    endcase
end

always @(*) begin
    sk_comb = 1'b0; //RETS, skip unconditionally

    if(mc_alter_flag) begin
        if(mc_type == MCTYPE0) begin
            if(mc_t0_alusel == 2'd2 || mc_t0_alusel == 2'd3) begin
                case(arith_code)
                    4'h2: sk_comb = ~c_comb; //ADDNC(skip condition: NO CARRY)
                    4'h3: sk_comb = ~c_comb; //SUBNB(skip condition: NO BORROW)
                    4'hA: sk_comb = ~c_comb; //SGT(skip condition: NO BORROW)
                    4'hB: sk_comb = c_comb;  //SLT(skip condition: BORROW)
                    4'hC: sk_comb = ~z_comb; //AND(skip condition: NO ZERO)
                    4'hD: sk_comb = z_comb;  //OR(skip condition: ZERO)
                    4'hE: sk_comb = ~z_comb; //SNE(skip condition: NO ZERO)
                    4'hF: sk_comb = z_comb;  //SEQ(skip condition: ZERO)
                endcase
            end
        end
        else if(mc_type == MCTYPE1) begin
            if(mc_t1_alusel == 4'h7) begin //shift 
                case(shift_code)
                    4'b0000: sk_comb = c_comb; //SLRC, skip condition: CARRY
                    4'b0100: sk_comb = c_comb; //SLLC, skip condition: CARRY
                endcase
            end
            else if(mc_t1_alusel == 4'hB) begin //INC, skip condition: CARRY
                sk_comb = c_comb;
            end
            else if(mc_t1_alusel == 4'hC) begin //DEC, skip condition: BORROW
                sk_comb = c_comb;
            end
        end
        else if(mc_type == MCTYPE2) begin
            case(mc_bk_skip_ctrl)
                3'b011: sk_comb = reg_MDH[reg_OPCODE[2:0]]; //BIT
                3'b100: sk_comb = skip_flag;  //SK
                3'b101: sk_comb = ~skip_flag; //SKN
                3'b110: sk_comb = iflag_muxed; //SKIT
                3'b111: sk_comb = ~iflag_muxed; //SKNIT
            endcase
        end

        if(opcode_page == 3'd0 && reg_OPCODE == 8'hB9) sk_comb = 1'b1;
    end
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        sk_temp <= 1'b0;
        flag_SK <= 1'b0; //reset
    end
    else begin
        if(cycle_tick) begin
            if(reg_PSW_wr) begin
                flag_SK <= reg_MDL[5]; sk_temp <= reg_MDL[5];
            end
            else begin
                if(mc_alter_flag) begin
                    if(mc_end_of_instruction) begin flag_SK <= sk_comb; sk_temp <= sk_comb; end
                    else sk_temp <= sk_comb;
                end
                else begin
                    if(mc_jump_to_next_inst) begin flag_SK <= sk_comb; sk_temp <= sk_comb; end
                    else begin if(mc_end_of_instruction) flag_SK <= sk_temp; end
                end
            end
        end
    end
end

//The L1/0 flag should be enabled at the end of the last microcode step(w/ mc_alter_flag)
//so as not to interfere with the execution of the current microcode.

//L1 flag, MVI A
//L0 flag, MVI L, LXI HL
wire            flag_l1_set_cond = opcode_page == 3'd0 && reg_OPCODE == 8'h69;
wire            flag_l0_set_cond = (opcode_page == 3'd0 && reg_OPCODE == 8'h6F) || (opcode_page == 3'd0 && reg_OPCODE == 8'h34);
always @(posedge emuclk) begin
    if(!mrst_n) begin
        flag_L1 <= 1'b0; //reset
        flag_L0 <= 1'b0;
    end
    else begin
        if(mcrom_read_tick) begin
            if(reg_PSW_wr) begin
                flag_L1 <= reg_MDL[3];
            end
            else begin
                if(!flag_l1_set_cond) flag_L1 <= 1'b0;
                else begin
                    if(mc_alter_flag) flag_L1 <= 1'b1;
                end
            end

            if(reg_PSW_wr) begin
                flag_L0 <= reg_MDL[2];
            end
            else begin
                if(!flag_l0_set_cond) flag_L0 <= 1'b0;
                else begin
                    if(mc_alter_flag) flag_L0 <= 1'b1;
                end
            end
        end
    end
end



///////////////////////////////////////////////////////////
//////  SKIP HANDLER
////

//Mask the microcode output as NOP when the skip condition is met.
//At the end of the opcode/data fetch cycle, read the successive instruction 
//by forcing the next_bus_acc as "RD4"
always @(*) begin
    if(|{flag_SK, flag_L1, flag_L0} && !(softi_proc_cyc | hardi_proc_cyc)) begin
        if(mc_jump_to_next_inst) mc_ctrl_output = {16'b11_0_0_10000_0_0_000_0_0, RD4};
        else mc_ctrl_output = {16'b11_0_0_10000_0_0_000_0_0, mcrom_data[1:0]};
    end
    else begin
        mc_ctrl_output[17:2] = mcrom_data[17:2];

        //next bus access; assert RD3 when the operation mode is RPA2/3, DE/HL+byte
        if(mc_type == MCTYPE3 && mc_s_cond_read) mc_ctrl_output[1:0] = (reg_OPCODE[0] & reg_OPCODE[1]) ? RD3 : mcrom_data[1:0];
        else mc_ctrl_output[1:0] = mcrom_data[1:0];
    end
end






///////////////////////////////////////////////////////////
//////  I/O PORTS
////







endmodule