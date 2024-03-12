module IKA87AD(
    //clock
    input   wire                i_EMUCLK,
    input   wire                i_MCUCLK_PCEN,

    //system control
    input   wire                i_RESET_n,
    input   wire                i_STOP_n,

    //M1/IO cycle output(mode0/1 open drain, negative logic output)
    output  wire                o_M1_n,
    output  wire                o_IO_n,

    //R/W control and output enables
    output  wire                o_ALE,
    output  wire                o_RD_n,
    output  wire                o_WR_n,
    output  wire                o_ALE_OE,   //ALE output driver control(for a CMOS variant, an NMOS one doesn't require this)
    output  wire                o_RD_n_OE,  //RD_n output driver control(")
    output  wire                o_WR_n_OE,  //WR_n output driver control(")

    //full address and data input/output port
    output  wire    [15:0]      o_A,
    input   wire    [7:0]       i_DI,
    output  wire    [7:0]       o_DO,
    output  wire                o_PD_DO_OE,
    output  wire                o_DO_OE,

    //memory structure config register
    output  wire    [7:0]       o_REG_MM, //MM register

    //interrupt control
    input   wire                i_NMI_n,
    input   wire                i_INT1,
    input   wire                i_INT2_n, //PC3

    //timer control
    input   wire                i_TI, //PC3
    output  wire                o_TO, //PC4

    //event counter control
    input   wire                i_CI, //PC5

    //port A I/O and output enables
    input   wire    [7:0]       i_PA_I,
    output  wire    [7:0]       o_PA_O,
    output  wire    [7:0]       o_PA_OE, //bitwise direction control

    //port B I/O and output enables
    input   wire    [7:0]       i_PB_I,
    output  wire    [7:0]       o_PB_O,
    output  wire    [7:0]       o_PB_OE,

    //port C I/O and output enables
    input   wire    [7:0]       i_PC_I,
    output  wire    [7:0]       o_PC_O,
    output  wire    [7:0]       o_PC_OE,

    //port D I/O and output enables
    input   wire    [7:0]       i_PD_I,
    output  wire    [7:0]       o_PD_O,
    output  wire                o_PD_OE, //can set only a bytewise direction

    //port F I/O and output enables
    input   wire    [7:0]       i_PF_I,
    output  wire    [7:0]       o_PF_O,
    output  wire    [7:0]       o_PF_OE,

    //AN4-7 digital edge detector
    input   wire    [3:0]       i_ANx_DIGITAL,

    //ADC interface
    output  wire    [2:0]       o_ANx_ANALOG_CH,   //adc channel select, from 0 to 7
    input   wire    [7:0]       i_ANx_ANALOG_DATA, //adc data input
    output  wire                o_ANx_ANALOG_RD_n  //conversion data read strobe
);

//include mnemonic list
`include "IKA87AD_mnemonics.sv"

//hardware stop mode release wait time
localparam HARD_STOP_RELEASE_WAIT = 20'd78;

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

reg             halt_flag, soft_stop_flag, hard_stop_flag;
wire            sr_stop = halt_flag | soft_stop_flag | hard_stop_flag;

reg     [11:0]  timing_sr;
reg     [1:0]   current_bus_acc;

wire    opcode_tick = timing_sr[11] & current_bus_acc == RD4 & mcuclk_pcen;
wire    rw_tick = timing_sr[8] & current_bus_acc != RD4 & mcuclk_pcen;
wire    cycle_tick = opcode_tick | rw_tick;

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
                    if(!sr_stop) begin
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
wire            is_NMI, is_TIMER0, is_TIMER1, is_pINT1, is_nINT2, is_CNTR0, is_CNTR1, is_nCNTRCIN, is_ADC; //implemented
wire            is_BUFFULL, is_BUFEMPTY; //not yet implemented

assign is_BUFFULL = 1'b0;
assign is_BUFEMPTY = 1'b0;

wire    [10:0]  is = {is_BUFEMPTY, is_BUFFULL, is_ADC, is_nCNTRCIN, is_CNTR1, is_CNTR0, is_nINT2, is_pINT1, is_TIMER1, is_TIMER0, is_NMI};

//interrupt sampler, note that interrupt sampler uses an independent divided clock
IKA87AD_irqsampler u_nmi_sampler   (mrst_n, emuclk, mcuclk_pcen, ~i_NMI_n, is_NMI);
IKA87AD_irqsampler u_pint1_sampler (mrst_n, emuclk, mcuclk_pcen, i_INT1, is_pINT1);
IKA87AD_irqsampler u_nint2_sampler (mrst_n, emuclk, mcuclk_pcen, ~i_INT2_n, is_nINT2);

//interrupt flags
wire    [10:0]  iflag; //interrupt flag
wire    [6:0]   eflag; //exception flag

assign iflag[10:9] = 2'b00; //not implemented

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

//NMI interrupt flag set/reset
IKA87AD_flag u_nmi          (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[0], 1'b1, 5'd0, reg_OPCODE[4:0], 
                            1'b0, 1'b0, iflag_auto_ack && irq_lv == 3'd7, iflag[0]);

//Timer0/1 interrupt flag set/reset
IKA87AD_flag u_timer0       (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[1], irq_mask_n[1], 5'd1, reg_OPCODE[4:0], 
                            &{irq_mask_n[2:1]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd6, iflag[1]);
IKA87AD_flag u_timer1       (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[2], irq_mask_n[2], 5'd2, reg_OPCODE[4:0], 
                            &{irq_mask_n[2:1]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd6, iflag[2]);

//Pin interrupt flag set/reset
IKA87AD_flag u_int1         (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[3], irq_mask_n[3], 5'd3, reg_OPCODE[4:0], 
                            &{irq_mask_n[4:3]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd5, iflag[3]);
IKA87AD_flag u_int2         (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[4], irq_mask_n[4], 5'd4, reg_OPCODE[4:0], 
                            &{irq_mask_n[4:3]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd5, iflag[4]);

//Event counter
IKA87AD_flag u_cntr0        (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[5], irq_mask_n[5], 5'd5, reg_OPCODE[4:0], 
                            &{irq_mask_n[6:5]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd4, iflag[5]);
IKA87AD_flag u_cntr1        (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[6], irq_mask_n[6], 5'd6, reg_OPCODE[4:0], 
                            &{irq_mask_n[6:5]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd4, iflag[6]);

//CI input/ADC
IKA87AD_flag u_ci           (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[7], irq_mask_n[7], 5'd7, reg_OPCODE[4:0], 
                            &{irq_mask_n[8:7]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd3, iflag[7]);
IKA87AD_flag u_adc          (mrst_n, emuclk, mcuclk_pcen, cycle_tick, is[8], irq_mask_n[8], 5'd8, reg_OPCODE[4:0], 
                            &{irq_mask_n[8:7]}, iflag_manual_ack, iflag_auto_ack && irq_lv == 3'd3, iflag[8]);

//interrupt generation
reg     [2:0]   irq_lv_z;
reg             irq_pending;
wire            irq_detected = (irq_pending & irq_lv < 3'd7 & irq_enabled) | (irq_pending & irq_lv == 3'd7);
always @(posedge emuclk) if(mcuclk_pcen) begin
    irq_lv_z <= irq_lv;
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        irq_pending <= 1'b0;
    end
    else begin
        if(irq_pending) begin
            if(mcuclk_pcen) if(iflag_auto_ack) irq_pending <= 1'b0; 
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
reg             nmi_n_z;
always @(posedge emuclk) if(mcuclk_pcen) begin
    nmi_n_z <= i_NMI_n;
end

reg             iflag_muxed;
always @(*) begin
    case(reg_OPCODE[4:0])
        5'h00: iflag_muxed = nmi_n_z;
        5'h01: iflag_muxed = iflag[1];
        5'h02: iflag_muxed = iflag[2];
        5'h03: iflag_muxed = iflag[3];
        5'h04: iflag_muxed = iflag[4];
        5'h05: iflag_muxed = iflag[5];
        5'h06: iflag_muxed = iflag[6];
        5'h07: iflag_muxed = iflag[7];
        5'h08: iflag_muxed = iflag[8];
        5'h09: iflag_muxed = iflag[9];
        5'h0A: iflag_muxed = iflag[10];
        5'h0B: iflag_muxed = eflag[0];
        5'h0C: iflag_muxed = eflag[1];
        5'h10: iflag_muxed = eflag[2];
        5'h11: iflag_muxed = eflag[3];
        5'h12: iflag_muxed = eflag[4];
        5'h13: iflag_muxed = eflag[5];
        5'h14: iflag_muxed = eflag[6];
        default: iflag_muxed = 1'b0;
    endcase
end




///////////////////////////////////////////////////////////
//////  SUSPENSION CONTROL
////

//stop pin sync chain
reg     [1:0]       stop_syncchain;
always @(posedge emuclk) if(mcuclk_pcen) begin
    stop_syncchain[0] <= ~i_STOP_n;
    stop_syncchain[1] <= stop_syncchain[0];
end

//halt/stop detection
wire                soft_halt_detected = mc_type == MCTYPE2 && mc_bk_cpu_susp && opcode_page == 3'd1 && reg_OPCODE == 8'h3B;
wire                soft_stop_detected = mc_type == MCTYPE2 && mc_bk_cpu_susp && opcode_page == 3'd1 && reg_OPCODE == 8'hBB;
wire                hard_stop_detected = mc_end_of_instruction && stop_syncchain[1];
wire                susp_detected = soft_halt_detected | soft_stop_detected | hard_stop_detected;

//force exec nop
reg             force_exec_nop;
always @(posedge emuclk) begin
    if(!mrst_n) force_exec_nop <= 1'b0;
    else begin if(cycle_tick) begin
        if(mc_end_of_instruction) force_exec_nop <= susp_detected;
    end end
end

//hardware stop mode release counter, counts up to 780,000
reg     [19:0]      hstop_osc_wait;
reg                 hstop_osc_unstable, hstop_osc_unstable_z;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        hstop_osc_unstable <= 1'b0;
        hstop_osc_unstable_z <= 1'b0;
    end
    else begin
        if(mcuclk_pcen) begin
            hstop_osc_unstable_z <= hstop_osc_unstable;

            if(stop_syncchain[1]) begin
                hstop_osc_unstable <= 1'b1;
            end
            else begin
                if(hstop_osc_wait == HARD_STOP_RELEASE_WAIT) begin
                    hstop_osc_unstable <= 1'b0;
                end
            end
        end
    end

    if(mcuclk_pcen) begin
        if(stop_syncchain) begin
            hstop_osc_wait <= 20'd0;
        end
        else begin
            if(hstop_osc_wait != HARD_STOP_RELEASE_WAIT) begin
                hstop_osc_wait <= hstop_osc_wait + 20'd1;
            end
        end
    end
end

//halt and stop, can be halt insturction without stopping chip's oscillator, this core will stop the timing generator
wire            release_soft_stop;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        halt_flag <= 1'b0;
        soft_stop_flag <= 1'b0;
        hard_stop_flag <= 1'b0;
    end
    else begin
        if(halt_flag) begin
            if(mcuclk_pcen) if((irq_lv != 3'd1) && (irq_lv != irq_lv_z)) halt_flag <= 1'b0;
        end
        else begin
            if(cycle_tick) if(soft_halt_detected) halt_flag <= 1'b1;
        end
        
        if(soft_stop_flag) begin
            if(mcuclk_pcen) if(release_soft_stop) soft_stop_flag <= 1'b0;
        end
        else begin
            if(cycle_tick) if(soft_stop_detected) soft_stop_flag <= 1'b1;
        end

        //According to the datasheet on page 178, if RST is asserted before the oscillator has stabilized,
        //the CPU core will start the program from 0x0000 without waiting for stabilization. I emulated that.
        if(hard_stop_flag) begin
            if(mcuclk_pcen) hard_stop_flag <= hstop_osc_unstable == 1'b0 && hstop_osc_unstable_z == 1'b1 ? 1'b0 : 1'b1; //release stop
        end
        else begin
            if(cycle_tick) if(mc_end_of_instruction) hard_stop_flag <= stop_syncchain[1];
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

//microsequencer
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
        if(is_arith_eval_op || arith_code == 4'h0) mseq_cntr_next = mseq_cntr + mc_s_bra_on_alu + 3'd1; //eval op + 00(move)
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
        00110: (w) RPA*
        00111: (w) RPA2*
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
        11101: (b) sr/sr1, OPCODE[]
        11110: () sr2, OPCODE[]
        11111: (w) *RPA_OFFSET, rpa2/rpa3 A, B, EA, byte addend select
    D[8:4] source A, destination register type, decoded by the external circuit, :
        00000: (b) r, OPCODE[]         
        00001: (b) r2, OPCODE[]
        00010: (b) r1, OPCODE[]
        00011: (w) rp2, OPCODE[]
        00100: (w) rp, OPCODE[]
        00101: (w) rp1, OPCODE[]
        00110: (b) C 
        00111: (w) SP
        01000: () sr3, OPCODE[0]
        01001: (b) MD_low_byte
        01010: (w) MD_word
        01011: (w) MA
        01100: (w) PC
        01101:     sr/sr1, OPCODE[]
        01110:     sr2, OPCODE[]
        01111: (b) A
        10000: (w) EA 
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
    0000: BC
    0001: DE
    0010: HL
    0011: SP
    0100: RPA
    0101:
    0110:
    0111:
    1000: A
    1001: EA
    1010: MD_high_byte
    1011: MD_word
    1100: PC
    1101: PSW
    1110: 
    1111: NO_SOURCE
    D[9:6] source C, destination
    0000: (w) BC
    0001: (b) r2
    0010:     
    0011: 
    0100: 
    0101: 
    0110: 
    0111:     NOWHERE
    1000: (b) A
    1001: (w) EA
    1010: (b) MD_low_byte
    1011  (b) MD_high_byte
    1100: (w) MD_word
    1101: (w) MA
    1110: (b) PSW
    1111:     PC

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
    D[13:10]: reserved
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
        //GPRs remain in undefined status after reset
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
        reg_SP <= 16'h0000; //undefined after reset
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
//////  SPECIAL REGISTERS
////

/*
    SPECIAL REGISTER LIST

    rw 0x00 PA - port A rw data
    rw 0x01 PB - port B rw data
    rw 0x02 PC - port C rw data
    rw 0x03 PD - port D rw data
    rw 0x05 PF - port F rw data
     w 0x06 MKH - Mask High(D[7:1])
     w 0x07 MKL - Mask Low(D[1:0])
    rw 0x08 ANM - ADC Mode(D[4:0], 0x00 after reset)
    rw 0x09 SMH - Serial Mode High(0x00 after reset) 
     w 0x0A SML - Serial Mode Low(0x48 after reset)
    rw 0x0B EOM - Timer/event counter output mode
     w 0x0C ETMM - Timer/event counter mode
    rw 0x0D TMM - Timer mode
     w 0x10 MM - Memory mapping(piggyback model only)
     w 0x11 MCC - Mode control C register
     w 0x12 MA - port A direction
     w 0x13 MB - port B direction
     w 0x14 MC - port C direction
     w 0x17 MF - port F direction
     w 0x18 TXB - tx buffer
    r  0x19 RXB - rx buffer
     w 0x1A TM0 - timer A register
     w 0x1B TM1 - timer B register
    r  0x20 CR0 - conversion result 0
    r  0x21 CR1 - conversion result 1
    r  0x22 CR2 - conversion result 2
    r  0x23 CR3 - conversion result 3
     w 0x28 ZCM - zero crossing detector mode
    
    <----register here can't be accessed by sr/sr1/sr2 fields---->
     w 0x30 ETM0 - event counter register 0
     w 0x31 ETM1 - event counter register 1
    r  0x32 ECNT - event counter
    r  0x33 ECPT - event counter capture register
*/

//special register address
reg     [5:0]   sr_wr_addr, sr_rd_addr;
always @(*) begin
         if(mc_type == MCTYPE0 && mc_sa_dst == SA_DST_SR_SR1) sr_wr_addr = reg_MDI[5:0];
    else if(mc_type == MCTYPE0 && mc_sa_dst == SA_DST_SR2)    sr_wr_addr = {2'b00, reg_OPCODE[7], reg_OPCODE[2:0]};
    else if(mc_type == MCTYPE0 && mc_sa_dst == SA_DST_SR3)    sr_wr_addr = {5'b11000, reg_OPCODE[0]};
    else   sr_wr_addr = 6'h3F;

         if(mc_type == MCTYPE0 && mc_sb == SB_SR_SR1)         sr_rd_addr = reg_MDI[5:0];
    else if(mc_type == MCTYPE0 && mc_sb == SB_SR2)            sr_rd_addr = {2'b00, reg_OPCODE[7], reg_OPCODE[2:0]};
    else if(mc_type == MCTYPE0 && mc_sb == SB_SR4)            sr_rd_addr = {5'b11001, reg_OPCODE[0]};
    else   sr_rd_addr = 6'h3F;
end

//port related register
reg     [7:0]   sreg_PAO, sreg_PBO, sreg_PCO, sreg_PDO, sreg_PFO; //undefined after reset, see page 180
reg     [7:0]   sreg_MA, sreg_MB, sreg_MC, sreg_MF, sreg_MM, sreg_MCC;

reg     [6:0]   sreg_MKL; //intrq disable register low ; ncntrcin, cntr1, cntr0, pint1, nint2, timer1, timer0, -
reg     [2:0]   sreg_MKH; //intrq disable register high; -, -, -, -, -, empty, full, adc

reg     [4:0]   sreg_ANM; //ADC settings

reg     [7:0]   sreg_SMH, sreg_SML; //serial interface settings

reg     [7:0]   sreg_EOM;
reg     [7:0]   sreg_ETMM;
reg     [7:0]   sreg_TMM;
reg     [7:0]   sreg_TM0, sreg_TM1; //undefined after reset, see page 180

reg     [1:0]   sreg_ZCM;

reg     [15:0]  sreg_ETM0, sreg_ETM1; //undefined after reset, see page 180

reg     [7:0]   sreg_CR[0:3];

assign irq_mask_n = ~{sreg_MKH, sreg_MKL, 1'b0};
assign o_REG_MM = sreg_MM;

reg     [7:0]   sreg_RDBUS;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        sreg_PAO <= 8'h00; sreg_PBO <= 8'h00; sreg_PCO <= 8'h00; sreg_PDO <= 8'h00; sreg_PFO <= 8'h00;
        sreg_MA  <= 8'hFF; sreg_MB  <= 8'hFF; sreg_MC  <= 8'hFF; sreg_MF  <= 8'hFF; sreg_MM  <= 8'h00;
        sreg_MCC <= 8'h00;
        sreg_MKL <= 7'b1111111; sreg_MKH <= 3'b111;
        sreg_ANM <= 5'h00;
        sreg_SMH <= 8'h00; sreg_SML <= 8'h48;
        sreg_EOM <= 8'h00;
        sreg_ETMM <= 8'h00;
        sreg_TMM <= 8'hFF;
        sreg_TM0 <= 8'h00; sreg_TM1 <= 8'h00; //see page 79
        sreg_ZCM <= 2'b11; //see page 59
        sreg_ETM0 <= 16'h0000; sreg_ETM1 <= 16'h0000;
    end
    else begin 
        if(cycle_tick) begin
            case(sr_wr_addr) 
                6'h00: sreg_PAO <= alu_output[7:0];
                6'h01: sreg_PBO <= alu_output[7:0];
                6'h02: sreg_PCO <= alu_output[7:0];
                6'h03: sreg_PDO <= alu_output[7:0];
                6'h05: sreg_PFO <= alu_output[7:0];
                6'h06: sreg_MKH <= alu_output[2:0];
                6'h07: sreg_MKL <= alu_output[7:1];
                6'h08: sreg_ANM <= alu_output[4:0];
                6'h09: sreg_SMH <= alu_output[7:0];
                6'h0A: sreg_SML <= alu_output[7:0];
                6'h0B: sreg_EOM <= alu_output[7:0];
                6'h0C: sreg_ETMM <= alu_output[7:0];
                6'h0D: sreg_TMM <= alu_output[7:0];
                6'h10: sreg_MM <= alu_output[7:0];
                6'h11: sreg_MCC <= alu_output[7:0];
                6'h12: sreg_MA <= alu_output[7:0];
                6'h13: sreg_MB <= alu_output[7:0];
                6'h14: sreg_MC <= alu_output[7:0];
                6'h17: sreg_MF <= alu_output[7:0];
                6'h18: ; //TxB, not implemented
                6'h1A: sreg_TM0 <= alu_output[7:0];
                6'h1B: sreg_TM1 <= alu_output[7:0];
                6'h28: sreg_ZCM <= alu_output[2:1];
                6'h30: sreg_ETM0 <= alu_output[15:0];
                6'h31: sreg_ETM1 <= alu_output[15:0];
                default: ;
            endcase 
        end 
        else if(mcuclk_pcen) begin
            if(release_soft_stop) sreg_TMM <= 8'hFF;
        end
    end
end

always @(*) begin
    case(sr_rd_addr) 
            6'h00: sreg_RDBUS = i_PA_I;
            6'h01: sreg_RDBUS = i_PB_I;
            6'h02: sreg_RDBUS = i_PC_I;
            6'h03: sreg_RDBUS = i_PD_I; 
            6'h05: sreg_RDBUS = i_PF_I;
            6'h08: sreg_RDBUS = {3'b000, sreg_ANM};
            6'h09: sreg_RDBUS = sreg_SMH;
            6'h0B: sreg_RDBUS = sreg_EOM;
            6'h0D: sreg_RDBUS = sreg_TMM;
            6'h19: sreg_RDBUS = 8'h00; //RxB, not implemented
            6'h20: sreg_RDBUS = sreg_CR[0];
            6'h21: sreg_RDBUS = sreg_CR[1];
            6'h22: sreg_RDBUS = sreg_CR[2];
            6'h23: sreg_RDBUS = sreg_CR[3];
            6'h32: sreg_RDBUS = 8'h00; //ECNT, not yet implemented
            6'h33: sreg_RDBUS = 8'h00; //ECPT, not yet implemented
            default: sreg_RDBUS = 8'h00;
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
                    if(md_in_byte_sel) reg_MDI[15:8] <= i_DI;
                    else reg_MDI[7:0] <= i_DI;
                end
                else begin
                    if(md_in_byte_sel) reg_MDI[15:8] <= i_DI;
                    else reg_MDI[7:0] <= i_DI;

                    if(md_in_byte_sel) reg_MDH <= i_DI;
                    else reg_MDL <= i_DI;
                end
            end

            //Opcode register load
            if(opcode_inlatch_tick) begin
                     if(force_exec_hardi) reg_OPCODE <= 8'h73;
                else if(force_exec_nop)   reg_OPCODE <= 8'h00;
                else                      reg_OPCODE <= i_DI;
            end
        
            //Full opcode register for the disassembler
            if(cycle_tick) begin if(mc_end_of_instruction) reg_FULL_OPCODE_cntr <= 2'd0; end
            else if(full_opcode_inlatch_tick_debug) begin
                reg_FULL_OPCODE_debug[reg_FULL_OPCODE_cntr] <= force_exec_hardi ? 8'h73 : i_DI;
                reg_FULL_OPCODE_cntr <= reg_FULL_OPCODE_cntr + 2'd1;
            end
        end
    end
end


//address high, multiplexed address low/byte data output
wire    [7:0]   md_out_byte_data = md_out_byte_sel == 1'b1 ? reg_MDH : reg_MDL;
//wire    [7:0]   addr_hi_out = memory_access_address[15:8];
//wire    [7:0]   addr_lo_data_out = addr_data_sel ? md_out_byte_data : memory_access_address[7:0];

//address/data output
assign  o_A = memory_access_address;
assign  o_DO = md_out_byte_data;


//ALE, /RD, /WR
reg             ale_out, rd_out, wr_out, pd_do_oe, do_oe, m1, io;
assign o_ALE = ale_out;
assign o_RD_n = ~rd_out;
assign o_WR_n = ~wr_out;
assign o_PD_DO_OE = pd_do_oe;
assign o_DO_OE = do_oe;
assign o_M1_n = ~m1;
assign o_IO_n = ~io;

always @(posedge emuclk) begin
    if(!mrst_n) begin
        ale_out <= 1'b0;
        rd_out <= 1'b0;
        wr_out <= 1'b0;
        pd_do_oe <= 1'b0;
        do_oe <= 1'b0;
        m1 <= 1'b0;
        io <= 1'b0;
    end
    else begin
        if(mcuclk_pcen) begin
            if(cycle_tick) begin
                if(mc_end_of_instruction && (irq_detected | susp_detected)) begin
                    ale_out <= 1'b0;
                    pd_do_oe <= 1'b0;
                end
                else begin
                    if(mc_next_bus_acc != IDLE) begin
                        ale_out <= 1'b1;
                        pd_do_oe <= 1'b1;
                        
                        if(mc_next_bus_acc == RD4) m1 <= 1'b1;
                        //if(mc_end_of_instruction && mc_next_bus_acc == RD4) m1 <= 1'b1;
                        if(mc_next_bus_acc == RD3 || mc_next_bus_acc == WR3) io <= 1'b1;
                    end
                end
            end
            else begin
                if(!(force_exec_hardi | force_exec_nop)) begin
                    //ALE off
                    if(timing_sr[1]) ale_out <= 1'b0;

                    //PD data OE off
                    if(current_bus_acc == RD3 || current_bus_acc == RD4) begin
                        if(timing_sr[2]) pd_do_oe <= 1'b0;
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

                    //M1/IO control
                    if(timing_sr[2]) m1 <= 1'b0;
                    if(timing_sr[2]) io <= 1'b0;
                end

                //WR control
                if(current_bus_acc == WR3) begin
                    if(timing_sr[2]) begin
                        wr_out <= 1'b1;
                        do_oe <= 1'b1;
                        io <= 1'b0;
                    end
                    else if(timing_sr[6]) wr_out <= 1'b0;
                    else if(timing_sr[8]) do_oe <= 1'b0;
                end
                else wr_out <= 1'b0;
            end
        end
    end
end



///////////////////////////////////////////////////////////
//////  ALU READ PORT MULTIPLEXERS
////

/*
    COMMON READ BUS
    4'h0: {8'h00, reg_V} - r
    4'h1: {8'h00, reg_A} - r
    4'h2: {8'h00, reg_EAH} - r1
    4'h3: {8'h00, reg_EAL} - r1
    4'h4: {8'h00, reg_B} - common
    4'h5: {8'h00, reg_C} - common
    4'h6: {8'h00, reg_D} - common
    4'h7: {8'h00, reg_E} - common
    4'h8: {8'h00, reg_H} - common
    4'h9: {8'h00, reg_L} - common
    4'hA: reg_SP - rp, rp2
    4'hB: {reg_V, reg_A} - rp1
    4'hC: {reg_B, reg_C} - common
    4'hD: {reg_D, reg_E} - common
    4'hE: {reg_H, reg_L} - common
    4'hF: {reg_EAH, reg_EAL} - common

    A READ BUS
    {8'h00, reg_A}

    EA READ BUS
    {reg_EAH, reg_EAL}
*/

//determines which fields in the microcode are used to read the register file
reg     [2:0]   mc_regfile_acc_mode;
always @(*) begin
    mc_regfile_acc_mode = 3'd4;
    
    if(mc_type == MCTYPE0) begin
             if(mc_sa_dst < 5'd8 && mc_sb >= 5'd8) mc_regfile_acc_mode = 3'd0; //source A uses regfile
        else if(mc_sa_dst >= 5'd8 && mc_sb < 5'd8) mc_regfile_acc_mode = 3'd1; //source B uses regfile
    end
    else if(mc_type == MCTYPE1) begin
             if(mc_sc_dst < 4'd8 && mc_sd >= 4'd8) mc_regfile_acc_mode = 3'd2; //source C uses regfile
        else if(mc_sc_dst >= 4'd8 && mc_sd < 4'd8) mc_regfile_acc_mode = 3'd3; //source D uses regfile
    end
end

//register file address: which register we should read?
reg     [4:0]   regfile_addr;
always @(*) begin
    regfile_addr = 5'h10; //ZERO

    if(mc_regfile_acc_mode[2] != 1'b1) begin
        if((mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_R) || 
            (mc_regfile_acc_mode == 3'd1 && mc_sb == SB_R)           ) begin //R type decoder
            case(reg_OPCODE[2:0])
                3'b000: regfile_addr = 5'h0;
                3'b001: regfile_addr = 5'h1;
                3'b010: regfile_addr = 5'h4;
                3'b011: regfile_addr = 5'h5;
                3'b100: regfile_addr = 5'h6;
                3'b101: regfile_addr = 5'h7;
                3'b110: regfile_addr = 5'h8;
                3'b111: regfile_addr = 5'h9;
            endcase
        end
        else if((mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_R1) || 
                (mc_regfile_acc_mode == 3'd1 && mc_sb == SB_R1)           ) begin //R1 type decoder
            case(reg_OPCODE[2:0])
                3'b000: regfile_addr = 5'h2;
                3'b001: regfile_addr = 5'h3;
                3'b010: regfile_addr = 5'h4;
                3'b011: regfile_addr = 5'h5;
                3'b100: regfile_addr = 5'h6;
                3'b101: regfile_addr = 5'h7;
                3'b110: regfile_addr = 5'h8;
                3'b111: regfile_addr = 5'h9;
            endcase
        end
        else if((mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_R2) || 
                (mc_regfile_acc_mode == 3'd1 && mc_sb == SB_R2)         ||
                (mc_regfile_acc_mode == 3'd2 && mc_sc_dst == SC_DST_R2) ||
                (mc_regfile_acc_mode == 3'd3 && mc_sd == SD_R2)           ) begin //R2 type decoder
            case(reg_OPCODE[1:0])
                2'b00: regfile_addr = 5'h0;
                2'b01: regfile_addr = 5'h1;
                2'b10: regfile_addr = 5'h4;
                2'b11: regfile_addr = 5'h5;
            endcase
        end
        else if((mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_RP2) || 
                (mc_regfile_acc_mode == 3'd1 && mc_sb == SB_RP2)           ) begin //RP2 type decoder
            case(reg_OPCODE[6:4])
                3'b000: regfile_addr = 5'hA;
                3'b001: regfile_addr = 5'hC;
                3'b010: regfile_addr = 5'hD;
                3'b011: regfile_addr = 5'hE;
                3'b100: regfile_addr = 5'hF;
                default: regfile_addr = 5'h10;
            endcase
        end
        else if((mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_RP1) || 
                (mc_regfile_acc_mode == 3'd1 && mc_sb == SB_RP1)           ) begin //RP1 type decoder
            case(reg_OPCODE[2:0])
                3'b000: regfile_addr = 5'hB;
                3'b001: regfile_addr = 5'hC;
                3'b010: regfile_addr = 5'hD;
                3'b011: regfile_addr = 5'hE;
                3'b100: regfile_addr = 5'hF;
                default: regfile_addr = 5'h10;
            endcase
        end
        else if((mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_RP) || 
                (mc_regfile_acc_mode == 3'd1 && mc_sb == SB_RP)           ) begin //RP type decoder
            case(reg_OPCODE[1:0])
                2'b00: regfile_addr = 5'hA;
                2'b01: regfile_addr = 5'hC;
                2'b10: regfile_addr = 5'hD;
                2'b11: regfile_addr = 5'hE;
            endcase
        end
        else if((mc_regfile_acc_mode == 3'd1 && mc_sb == SB_RPA) ||
                (mc_regfile_acc_mode == 3'd3 && mc_sd == SD_RPA)   ) begin //RPA type decoder
            case(reg_OPCODE[2:0])
                3'b001: regfile_addr = 5'hC;
                3'b010: regfile_addr = 5'hD;
                3'b011: regfile_addr = 5'hE;
                3'b100: regfile_addr = 5'hD; //A+, use alu type 1 for auto inc/dec
                3'b101: regfile_addr = 5'hE; //A+, use alu type 1 for auto inc/dec
                3'b110: regfile_addr = 5'hD; //-A, use alu type 1 for auto inc/dec
                3'b111: regfile_addr = 5'hE; //-A, use alu type 1 for auto inc/dec
                default: regfile_addr = 5'h10;
            endcase
        end
        else if(mc_regfile_acc_mode == 3'd1 && mc_sb == SB_RPA2) begin //RPA2 type decoder
            case(reg_OPCODE[2:0])
                3'b011: regfile_addr = 5'hD;
                3'b100: regfile_addr = 5'hE;
                3'b101: regfile_addr = 5'hE;
                3'b110: regfile_addr = 5'hE;
                3'b111: regfile_addr = 5'hE;
                default: regfile_addr = 5'h10;
            endcase
        end
        else if(mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_C) 
            regfile_addr = 5'h5;
        else if(mc_regfile_acc_mode == 3'd0 && mc_sa_dst == SA_DST_SP) 
            regfile_addr = 5'hA;
        else if((mc_regfile_acc_mode == 3'd2 && mc_sc_dst == SC_DST_BC) || 
                (mc_regfile_acc_mode == 3'd3 && mc_sd == SD_BC)) 
            regfile_addr = 5'hC;
        else if(mc_regfile_acc_mode == 3'd3 && mc_sd == SD_DE) 
            regfile_addr = 5'hD;
        else if(mc_regfile_acc_mode == 3'd3 && mc_sd == SD_HL) 
            regfile_addr = 5'hE;
        else if(mc_regfile_acc_mode == 3'd3 && mc_sd == SD_SP) 
            regfile_addr = 5'hA;
        else if(mc_regfile_acc_mode == 3'd4) begin
            case(reg_OPCODE[1:0])
                2'b00: regfile_addr = 5'h0;
                2'b01: regfile_addr = 5'h1;
                2'b10: regfile_addr = 5'h4;
                2'b11: regfile_addr = 5'h5;
            endcase
        end
    end
end

reg     [15:0]  reg_RDBUS;
always @(*) begin
    if(regfile_addr[4]) reg_RDBUS = 16'h0000;
    else begin
        case(regfile_addr[3:0])
            4'h0: reg_RDBUS = {8'h00, reg_V};
            4'h1: reg_RDBUS = {8'h00, reg_A};
            4'h2: reg_RDBUS = {8'h00, reg_EAH};
            4'h3: reg_RDBUS = {8'h00, reg_EAL};
            4'h4: reg_RDBUS = {8'h00, reg_B};
            4'h5: reg_RDBUS = {8'h00, reg_C};
            4'h6: reg_RDBUS = {8'h00, reg_D};
            4'h7: reg_RDBUS = {8'h00, reg_E};
            4'h8: reg_RDBUS = {8'h00, reg_H};
            4'h9: reg_RDBUS = {8'h00, reg_L};
            4'hA: reg_RDBUS = reg_SP;
            4'hB: reg_RDBUS = {reg_V, reg_A};
            4'hC: reg_RDBUS = {reg_B, reg_C};
            4'hD: reg_RDBUS = {reg_D, reg_E};
            4'hE: reg_RDBUS = {reg_H, reg_L};
            4'hF: reg_RDBUS = {reg_EAH, reg_EAL};
        endcase
    end
end

/*
OLDER IMPLEMENTATION
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
reg     [15:0]   reg_RPA, reg_RPA2;
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
end
*/

reg     [15:0]   reg_RPA2_OFFSET;
always @(*) begin
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
            SA_DST_R      : alu_pa = reg_RDBUS; //{8'h00, reg_R};
            SA_DST_R2     : alu_pa = reg_RDBUS; //{8'h00, reg_R2};
            SA_DST_R1     : alu_pa = reg_RDBUS; //{8'h00, reg_R1};
            SA_DST_RP2    : alu_pa = reg_RDBUS; //reg_RP2;
            SA_DST_C      : alu_pa = reg_RDBUS; //{8'h00, reg_C};
            SA_DST_SP     : alu_pa = reg_RDBUS; //reg_SP;
            SA_DST_RP     : alu_pa = reg_RDBUS; //reg_RP;
            SA_DST_RP1    : alu_pa = reg_RDBUS; //reg_RP1;
            SA_DST_SR_SR1 : alu_pa = sreg_RDBUS;
            SA_DST_SR2    : alu_pa = sreg_RDBUS;
            SA_DST_SR3    : alu_pa = sreg_RDBUS;
            SA_DST_MDL    : alu_pa = {8'h00, reg_MDL};
            SA_DST_MD     : alu_pa = {reg_MDH, reg_MDL};
            SA_DST_MA     : alu_pa = reg_MA;
            SA_DST_PC     : alu_pa = reg_PC;
            SA_DST_A      : alu_pa = {8'h00, reg_A};
            SA_DST_EA     : alu_pa = {reg_EAH, reg_EAL};
            default       : alu_pa = 16'h0000;
        endcase

        case(mc_sb)
            SB_R          : alu_pb = reg_RDBUS; //{8'h00, reg_R};
            SB_R2         : alu_pb = reg_RDBUS; //{8'h00, reg_R2};
            SB_R1         : alu_pb = reg_RDBUS; //{8'h00, reg_R1};
            SB_RP2        : alu_pb = reg_RDBUS; //reg_RP2;
            SB_RP         : alu_pb = reg_RDBUS; //reg_RP;
            SB_RP1        : alu_pb = reg_RDBUS; //reg_RP1;
            SB_RPA        : alu_pb = reg_RDBUS; //reg_RPA;
            SB_RPA2       : alu_pb = reg_RDBUS; //reg_RPA2;
            SB_SR_SR1     : alu_pb = sreg_RDBUS;
            SB_SR2        : alu_pb = sreg_RDBUS;
            SB_SR4        : alu_pb = sreg_RDBUS;
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
            SB_OFFSET     : alu_pb = reg_RPA2_OFFSET;
            default       : alu_pb = 16'h0000;
        endcase
    end
    else if(mc_type == MCTYPE1) begin
        case(mc_sc_dst)
            SC_DST_R2     : alu_pa = reg_RDBUS; //{8'h00, reg_R2};
            SC_DST_BC     : alu_pa = reg_RDBUS; //{reg_B, reg_C};
            SC_DST_A      : alu_pa = reg_A;
            SC_DST_EA     : alu_pa = {reg_EAH, reg_EAL};
            SC_DST_MDL    : alu_pa = {8'h00, reg_MDL};
            SC_DST_MDH    : alu_pa = {8'h00, reg_MDH};
            SC_DST_MD     : alu_pa = {reg_MDH, reg_MDL};
            SC_DST_MA     : alu_pa = reg_MA;
            SC_DST_PSW    : alu_pa = reg_PSW;
            SC_DST_PC     : alu_pa = reg_PC;
            default       : alu_pa = 16'h0000;
        endcase

        case(mc_sd)
            SD_BC         : alu_pb = reg_RDBUS; //{reg_B, reg_C};
            SD_DE         : alu_pb = reg_RDBUS; //{reg_D, reg_E};
            SD_HL         : alu_pb = reg_RDBUS; //{reg_H, reg_L};
            SD_SP         : alu_pb = reg_RDBUS; //reg_SP;
            SD_RPA        : alu_pb = reg_RDBUS; //reg_RPA;
            SD_A          : alu_pb = reg_A;
            SD_EA         : alu_pb = {reg_EAH, reg_EAL};
            SD_MDH        : alu_pb = {8'h00, reg_MDH};
            SD_MD         : alu_pb = {reg_MDH, reg_MDL};
            SD_PC         : alu_pb = reg_PC;
            SD_PSW        : alu_pb = reg_PSW;
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

reg     [7:0]   alu_muldiv_r2_temp;
always @(posedge emuclk) if(cycle_tick) begin
    if(mc_type == MCTYPE1 && (mc_t1_alusel == 4'h4 || mc_t1_alusel == 4'h5)) alu_muldiv_r2_temp <= reg_RDBUS[7:0];
end

wire    [15:0]  alu_mul_pa = {reg_EAH, reg_EAL};
wire    [15:0]  alu_mul_pb = alu_muldiv_r2_temp[alu_muldiv_cntr[2:0]] ? ({8'h00, reg_A} << alu_muldiv_cntr[2:0]) : 16'h0000;

wire    [15:0]  alu_div_pa = reg_TEMP;
wire    [15:0]  alu_div_pb = ~{8'h00, alu_muldiv_r2_temp};

wire    [31:0]  a = {reg_TEMP, {reg_EAH, reg_EAL}};
wire    [31:0]  b = {alu_adder_out, {reg_EAH, reg_EAL}};
wire    [31:0]  alu_div_out = alu_adder_out[15] ? {a[30:0], 1'b0} :
                                                  {b[30:0], 1'b1};
wire    [7:0]   alu_div_remainder = alu_adder_out[15] ? reg_TEMP[7:0] : alu_adder_out[7:0];


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

            alu_muldiv_reg_TEMP_wr = 1'b1;
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
                    alu_muldiv_reg_TEMP_wr = 1'b1; //alu_muldiv_cntr[3:0] == 4'd15 ? 1'b0 : 1'b1;
                    alu_muldiv_reg_EA_wr = 1'b1;
                    
                    alu_adder_op0 = alu_div_pa;  //reg EA
                    alu_adder_op1 = alu_div_pb; //-r2
                    alu_adder_cin = 1'b1;
                    
                    alu_output = alu_div_out[15:0];
                    alu_temp_output = alu_muldiv_cntr[3:0] == 4'd15 ? {8'h00, alu_div_remainder} : alu_div_out[31:16];
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
        if(mc_t0_alusel == 2'd2 || mc_t0_alusel == 2'd3) begin
            if(is_arith_eval_op) z_comb = data_w_nb ? alu_temp_output == 16'h0000 : alu_temp_output[7:0] == 8'h00;
            else                 z_comb = data_w_nb ? alu_output == 16'h0000 : alu_output[7:0] == 8'h00;
        end
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
                    default: sk_comb = 1'b0;
                endcase
            end
        end
        else if(mc_type == MCTYPE1) begin
            if(mc_t1_alusel == 4'h7) begin //shift 
                case(shift_code)
                    4'b0000: sk_comb = c_comb; //SLRC, skip condition: CARRY
                    4'b0100: sk_comb = c_comb; //SLLC, skip condition: CARRY
                    default: sk_comb = 1'b0;
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
                default: sk_comb = 1'b0;
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

//port output enables
assign o_PA_OE = ~sreg_MA;
assign o_PB_OE = ~sreg_MB;
assign o_PC_OE = ~sreg_MC;
assign o_PD_OE = sreg_MM[0];
assign o_PF_OE = ~sreg_MF;

//port data
assign o_PA_O = sreg_PAO;
assign o_PB_O = sreg_PBO;
assign o_PC_O = sreg_PCO;
assign o_PD_O = sreg_PDO;
assign o_PF_O = sreg_PFO;



///////////////////////////////////////////////////////////
//////  DIV3 TICK GENERATOR
////

reg     [1:0]   div3_tick_cntr;
wire            div3_tick = div3_tick_cntr == 2'd2 && mcuclk_pcen;
always @(posedge emuclk) begin
    if(!mrst_n) div3_tick_cntr <= 2'd0;
    else begin if(mcuclk_pcen) begin
        div3_tick_cntr <= div3_tick_cntr == 2'd2 ? 2'd0 : div3_tick_cntr + 2'd1;
    end end
end



///////////////////////////////////////////////////////////
//////  ADC DATA ACQUISITION CONTROL
////

reg     [4:0]   current_adc_mode;
reg     [7:0]   adc_state_cntr;
reg     [2:0]   adc_ch;
reg             adc_strobe_n;

assign o_ANx_ANALOG_CH = current_adc_mode[0] ? current_adc_mode[3:1] : adc_ch;
assign o_ANx_ANALOG_RD_n = adc_strobe_n;
assign is_ADC = current_adc_mode[4] ? div3_tick_cntr == 2'd2 && adc_state_cntr == 8'd127 && adc_ch[1:0] == 2'b11: 
                                      div3_tick_cntr == 2'd2 && adc_state_cntr == 8'd175 && adc_ch[1:0] == 2'b11;

always @(posedge emuclk) begin
    if(!mrst_n) begin
        current_adc_mode <= 5'b00000;
        adc_state_cntr <= 8'd0;
        adc_ch <= 3'd0;
    end
    else begin if(div3_tick) begin
        if(( current_adc_mode[4] && adc_state_cntr == 8'd143) || 
           (~current_adc_mode[4] && adc_state_cntr == 8'd191)) begin

            //make a copy of the current ANM reg value
            current_adc_mode <= sreg_ANM;

            //scan mode/single ch mode select
            if(sreg_ANM[0] != current_adc_mode[0]) adc_ch <= {sreg_ANM[3], 2'b00}; //if the mode has been changed, initialize the counter
            else adc_ch[1:0] <= adc_ch[1:0] == 2'b11 ? 2'b00 : adc_ch[1:0] + 2'b01; //count up

            //reset the adc state counter
            adc_state_cntr <= 8'd0;
        end
        else begin
            adc_state_cntr <= adc_state_cntr + 8'd1;
        end

        //data acquisition control
        if(adc_state_cntr == 8'd0) adc_strobe_n <= 1'b0;
        else if(adc_state_cntr == 8'd127) begin
            adc_strobe_n <= current_adc_mode[4] ? 1'b1 : 1'b0;
            if(current_adc_mode[4]) sreg_CR[adc_ch[1:0]] <= i_ANx_ANALOG_DATA;
        end
        else if(adc_state_cntr == 8'd175) begin
            adc_strobe_n <= current_adc_mode[4] ? 1'b0 : 1'b1;
            if(~current_adc_mode[4]) sreg_CR[adc_ch[1:0]] <= i_ANx_ANALOG_DATA;
        end
    end end
end



///////////////////////////////////////////////////////////
//////  TIMER
////

//tick from an external source
wire            ti_nedet;
IKA87AD_nedet nedet_ti (mrst_n, emuclk, div3_tick, i_TI, ti_nedet);

//make timer ticks
reg     [1:0]   timer_prescaler;
reg     [6:0]   timer_div_cntr;
wire            timer_tick   = timer_prescaler == 2'd2 & mcuclk_pcen; //fs/3 tick
wire            timer_div12  = timer_div_cntr[1:0] == 2'd3;
wire            timer_div384 = timer_div_cntr == 7'd127;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        timer_prescaler <= 2'd0;
        timer_div_cntr <= 7'd0;
    end
    else begin if(mcuclk_pcen) begin
        if(hard_stop_flag || (soft_stop_flag && !iflag[0])) begin //stop timer
            timer_prescaler <= timer_prescaler;
            timer_div_cntr <= timer_div_cntr;
        end
        else begin
            timer_prescaler <= timer_prescaler == 2'd2 ? 2'd0 : timer_prescaler + 2'd1;
            if(timer_prescaler == 2'd2) timer_div_cntr <= timer_div_cntr == 7'd127 ? 7'd0 : timer_div_cntr + 7'd1;
        end
    end end
end

//timer 0/1
reg     [7:0]   timer0, timer1; //timer registersx`
reg             tmff; 
reg             timer0_cnt, timer1_cnt, tmff_toggle; //timer0/1 and tmff ticks
wire            timer0_match = timer0_cnt & (timer0 == sreg_TM0) & (sr_wr_addr != 6'h1A); //this tick pokes the next DFFs
wire            timer1_match = timer1_cnt & (timer1 == sreg_TM1) & (sr_wr_addr != 6'h1B);
wire            tmff_pcen = tmff_toggle & (tmff == 1'b0) & timer_tick; //TMFF postive edge clock enable
wire            tmff_ncen = (tmff_toggle & (tmff == 1'b1) & timer_tick) | 
                            ((sreg_TMM[1:0] == 2'b11) & (tmff == 1'b1) & timer_tick); //TMFF negative edge clock enable
assign is_TIMER0 = ~soft_stop_flag & timer0_match & timer_tick;
assign is_TIMER1 = ~soft_stop_flag & timer1_match & timer_tick;
assign release_soft_stop = soft_stop_flag & timer1_match & timer_tick; //release soft stop
assign o_TO = tmff;
always @(*) begin
    case(sreg_TMM[3:2])
        2'b00: timer0_cnt = timer_div12;
        2'b01: timer0_cnt = timer_div384;
        2'b10: timer0_cnt = ti_nedet;
        2'b11: timer0_cnt = 1'b0;
    endcase

    case(sreg_TMM[6:5])
        2'b00: timer1_cnt = timer_div12;
        2'b01: timer1_cnt = timer_div384;
        2'b10: timer1_cnt = ti_nedet;
        2'b11: timer1_cnt = timer0_match;
    endcase

    case(sreg_TMM[1:0])
        2'b00: tmff_toggle = timer0_match;
        2'b01: tmff_toggle = timer1_match;
        2'b10: tmff_toggle = 1'b1; //toggle at every timer_tick(fs/3)
        2'b11: tmff_toggle = 1'b0; //reset
    endcase
end

always @(posedge emuclk) begin
    if(!mrst_n) begin
        timer0 <= 8'h01;
        timer1 <= 8'h01;
        tmff <= 1'b0;
    end
    else if(soft_stop_flag && !iflag[0]) begin //timer 0 and 1 can be used as a soft stop release wait timer, NMI triggered
        timer0 <= 8'h01;
        timer1 <= 8'h01;
    end
    else begin if(timer_tick) begin //use timer tick(fs/3)
        //define timer 0 behavior
        if(sreg_TMM[4]) timer0 <= 8'h01;
        else begin
            if(timer0_cnt) begin
                if(timer0 == sreg_TM0 && sr_wr_addr != 6'h1A) timer0 <= 8'h01; //no comparison is performed while updating TM0, see datasheet p79
                else timer0 <= timer0 == 8'hFF ? 8'h00 : timer0 + 8'h01;
            end
        end

        //define timer 1 behavior
        if(sreg_TMM[7]) timer1 <= 8'h01;
        else begin
            if(timer1_cnt) begin
                if(timer1 == sreg_TM1 && sr_wr_addr != 6'h1B) timer1 <= 8'h01; //no comparison is performed while updating TM1
                else timer1 <= timer1 == 8'hFF ? 8'h00 : timer1 + 8'h01;
            end
        end

        //define tmff behavior
        if(sreg_TMM[1:0] == 2'b11) tmff <= 1'b0;
        else begin
            if(tmff_toggle) tmff <= ~tmff;
        end
    end end
end



///////////////////////////////////////////////////////////
//////  EVENT COUNTER
////

//tick from an external source
reg     [1:0]   ci_sampler;
reg             ci_nedet, ci_state;
always @(posedge emuclk) begin
    if(!mrst_n) begin 
        ci_sampler <= 2'b00;
        ci_nedet <= 1'b0;
        ci_state <= 1'b0;
    end
    else begin if(div3_tick) begin
        ci_sampler[0] <= i_CI;
        ci_sampler[1] <= ci_sampler[0];

        if(ci_sampler == 2'b10 && i_CI == 1'b0) ci_nedet <= 1'b1;
        else ci_nedet <= 1'b0;

        if(ci_sampler == 2'b10 && i_CI == 1'b0) ci_state <= 1'b0;
        else if(ci_sampler == 2'b01 && i_CI == 1'b1) ci_state <= 1'b1;
        else ci_state <= ci_state;
    end end
end

//event counter
reg             event_cntr_cnt;
always @(*) begin
    case(sreg_ETMM[1:0])
        2'b00: event_cntr_cnt = timer_div12;
        2'b01: event_cntr_cnt = timer_div12 & ci_state;
        2'b10: event_cntr_cnt = ci_nedet;
        2'b11: event_cntr_cnt = ci_nedet & tmff;
    endcase
end

reg     [15:0]  event_cntr;
wire    [16:0]  event_cntr_next = event_cntr + 16'd1;
wire            cntr0_match = event_cntr_cnt & (event_cntr_next[15:0] == sreg_ETM0) & (sr_wr_addr != 6'h30);
wire            cntr1_match = event_cntr_cnt & (event_cntr_next[15:0] == sreg_ETM1) & (sr_wr_addr != 6'h31);
assign is_CNTR0 = cntr0_match & timer_tick;
assign is_CNTR1 = cntr1_match & timer_tick;
assign is_nCNTRCIN = ci_nedet;
wire            fs_OV = event_cntr_next[16] & event_cntr_cnt & timer_tick;
always @(posedge emuclk) begin
    if(!mrst_n) begin
        event_cntr <= 16'd0;
    end
    else begin if(timer_tick) begin 
        case(sreg_ETMM[3:2])
            2'b00: event_cntr <= 16'd0;
            2'b01: event_cntr <= event_cntr_cnt ? event_cntr_next[15:0] : event_cntr;
            2'b10: begin
                if(sreg_ETMM[1]) begin
                    if(tmff_ncen) event_cntr <= 16'd0;
                    else event_cntr <= event_cntr_cnt ? event_cntr_next[15:0] : event_cntr;
                end
                else begin
                    if(ci_nedet) event_cntr <= 16'd0;
                    else event_cntr <= event_cntr_cnt ? event_cntr_next[15:0] : event_cntr;
                end
            end
            2'b11: begin
                if(event_cntr_cnt) begin
                    if(event_cntr_next[15:0] == sreg_ETM1) event_cntr <= 16'd0;
                    else event_cntr <= event_cntr_next[15:0];
                end
            end 
        endcase
    end end
end



///////////////////////////////////////////////////////////
//////  MISC FLAGS
////

//ER(serial IO error)
assign eflag[0] = 1'b0; //not implemented

//OV(event counter overflow)
IKA87AD_flag u_nedet_ov     (mrst_n, emuclk, mcuclk_pcen, cycle_tick, fs_OV, 1'b1, 5'd12, reg_OPCODE[4:0], 
                            1'b1, iflag_manual_ack, 1'b0, eflag[1]);

//AN7-4(negative edge)
wire    [3:0]   fs_ANx;
IKA87AD_nedet u_nedet_an7 (mrst_n, emuclk, div3_tick, i_ANx_DIGITAL[3], fs_ANx[3]);
IKA87AD_nedet u_nedet_an6 (mrst_n, emuclk, div3_tick, i_ANx_DIGITAL[2], fs_ANx[2]);
IKA87AD_nedet u_nedet_an5 (mrst_n, emuclk, div3_tick, i_ANx_DIGITAL[1], fs_ANx[1]);
IKA87AD_nedet u_nedet_an4 (mrst_n, emuclk, div3_tick, i_ANx_DIGITAL[0], fs_ANx[0]);

IKA87AD_flag u_eflag_an7    (mrst_n, emuclk, mcuclk_pcen, cycle_tick, fs_ANx[3], 1'b1, 5'd16, reg_OPCODE[4:0], 
                            1'b1, iflag_manual_ack, 1'b0, eflag[2]);
IKA87AD_flag u_eflag_an6    (mrst_n, emuclk, mcuclk_pcen, cycle_tick, fs_ANx[2], 1'b1, 5'd17, reg_OPCODE[4:0], 
                            1'b1, iflag_manual_ack, 1'b0, eflag[3]);
IKA87AD_flag u_eflag_an5    (mrst_n, emuclk, mcuclk_pcen, cycle_tick, fs_ANx[1], 1'b1, 5'd18, reg_OPCODE[4:0], 
                            1'b1, iflag_manual_ack, 1'b0, eflag[4]);
IKA87AD_flag u_eflag_an4    (mrst_n, emuclk, mcuclk_pcen, cycle_tick, fs_ANx[0], 1'b1, 5'd19, reg_OPCODE[4:0], 
                            1'b1, iflag_manual_ack, 1'b0, eflag[5]);

//SB(first boot flag)
assign eflag[6] = 1'b1; //not implemented

endmodule