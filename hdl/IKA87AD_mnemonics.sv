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
localparam SA_DST_A      = 5'b01110;
localparam SA_DST_EA     = 5'b01111;
localparam SA_DST_C      = 5'b10000;

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

//MICROCODE ROUTINE ENTRANCE ADDRESS
localparam MVI_R_BYTE       = 8'd000;
localparam STAX_RPA         = 8'd002;
localparam LDAX_RPA         = 8'd004;
localparam LXI              = 8'd006;
localparam NOP              = 8'd255;