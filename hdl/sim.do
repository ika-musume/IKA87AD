onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /IKA87AD_tb/EMUCLK
add wave -noupdate /IKA87AD_tb/PCEN
add wave -noupdate /IKA87AD_tb/RST_n
add wave -noupdate -radix unsigned /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -radix hexadecimal /IKA87AD_tb/cpu_addr
add wave -noupdate /IKA87AD_tb/cpu_rd_n
add wave -noupdate /IKA87AD_tb/cpu_wr_n
add wave -noupdate -radix hexadecimal /IKA87AD_tb/dbus
add wave -noupdate -radix hexadecimal /IKA87AD_tb/u_dut/o_REG_MM
add wave -noupdate -radix hexadecimal -childformat {{{/IKA87AD_tb/ram[0]} -radix hexadecimal} {{/IKA87AD_tb/ram[1]} -radix hexadecimal} {{/IKA87AD_tb/ram[2]} -radix hexadecimal} {{/IKA87AD_tb/ram[3]} -radix hexadecimal} {{/IKA87AD_tb/ram[4]} -radix hexadecimal} {{/IKA87AD_tb/ram[5]} -radix hexadecimal} {{/IKA87AD_tb/ram[6]} -radix hexadecimal} {{/IKA87AD_tb/ram[7]} -radix hexadecimal} {{/IKA87AD_tb/ram[8]} -radix hexadecimal} {{/IKA87AD_tb/ram[9]} -radix hexadecimal} {{/IKA87AD_tb/ram[10]} -radix hexadecimal} {{/IKA87AD_tb/ram[11]} -radix hexadecimal} {{/IKA87AD_tb/ram[12]} -radix hexadecimal} {{/IKA87AD_tb/ram[13]} -radix hexadecimal} {{/IKA87AD_tb/ram[14]} -radix hexadecimal} {{/IKA87AD_tb/ram[15]} -radix hexadecimal} {{/IKA87AD_tb/ram[16]} -radix hexadecimal} {{/IKA87AD_tb/ram[17]} -radix hexadecimal} {{/IKA87AD_tb/ram[18]} -radix hexadecimal} {{/IKA87AD_tb/ram[19]} -radix hexadecimal} {{/IKA87AD_tb/ram[20]} -radix hexadecimal} {{/IKA87AD_tb/ram[21]} -radix hexadecimal} {{/IKA87AD_tb/ram[22]} -radix hexadecimal} {{/IKA87AD_tb/ram[23]} -radix hexadecimal} {{/IKA87AD_tb/ram[24]} -radix hexadecimal} {{/IKA87AD_tb/ram[25]} -radix hexadecimal} {{/IKA87AD_tb/ram[26]} -radix hexadecimal} {{/IKA87AD_tb/ram[27]} -radix hexadecimal} {{/IKA87AD_tb/ram[28]} -radix hexadecimal} {{/IKA87AD_tb/ram[29]} -radix hexadecimal} {{/IKA87AD_tb/ram[30]} -radix hexadecimal} {{/IKA87AD_tb/ram[31]} -radix hexadecimal} {{/IKA87AD_tb/ram[32]} -radix hexadecimal} {{/IKA87AD_tb/ram[33]} -radix hexadecimal} {{/IKA87AD_tb/ram[34]} -radix hexadecimal} {{/IKA87AD_tb/ram[35]} -radix hexadecimal} {{/IKA87AD_tb/ram[36]} -radix hexadecimal} {{/IKA87AD_tb/ram[37]} -radix hexadecimal} {{/IKA87AD_tb/ram[38]} -radix hexadecimal} {{/IKA87AD_tb/ram[39]} -radix hexadecimal} {{/IKA87AD_tb/ram[40]} -radix hexadecimal} {{/IKA87AD_tb/ram[41]} -radix hexadecimal} {{/IKA87AD_tb/ram[42]} -radix hexadecimal} {{/IKA87AD_tb/ram[43]} -radix hexadecimal} {{/IKA87AD_tb/ram[44]} -radix hexadecimal} {{/IKA87AD_tb/ram[45]} -radix hexadecimal} {{/IKA87AD_tb/ram[46]} -radix hexadecimal} {{/IKA87AD_tb/ram[47]} -radix hexadecimal} {{/IKA87AD_tb/ram[48]} -radix hexadecimal} {{/IKA87AD_tb/ram[49]} -radix hexadecimal} {{/IKA87AD_tb/ram[50]} -radix hexadecimal} {{/IKA87AD_tb/ram[51]} -radix hexadecimal} {{/IKA87AD_tb/ram[52]} -radix hexadecimal} {{/IKA87AD_tb/ram[53]} -radix hexadecimal} {{/IKA87AD_tb/ram[54]} -radix hexadecimal} {{/IKA87AD_tb/ram[55]} -radix hexadecimal} {{/IKA87AD_tb/ram[56]} -radix hexadecimal} {{/IKA87AD_tb/ram[57]} -radix hexadecimal} {{/IKA87AD_tb/ram[58]} -radix hexadecimal} {{/IKA87AD_tb/ram[59]} -radix hexadecimal} {{/IKA87AD_tb/ram[60]} -radix hexadecimal} {{/IKA87AD_tb/ram[61]} -radix hexadecimal} {{/IKA87AD_tb/ram[62]} -radix hexadecimal} {{/IKA87AD_tb/ram[63]} -radix hexadecimal} {{/IKA87AD_tb/ram[64]} -radix hexadecimal} {{/IKA87AD_tb/ram[65]} -radix hexadecimal} {{/IKA87AD_tb/ram[66]} -radix hexadecimal} {{/IKA87AD_tb/ram[67]} -radix hexadecimal} {{/IKA87AD_tb/ram[68]} -radix hexadecimal} {{/IKA87AD_tb/ram[69]} -radix hexadecimal} {{/IKA87AD_tb/ram[70]} -radix hexadecimal} {{/IKA87AD_tb/ram[71]} -radix hexadecimal} {{/IKA87AD_tb/ram[72]} -radix hexadecimal} {{/IKA87AD_tb/ram[73]} -radix hexadecimal} {{/IKA87AD_tb/ram[74]} -radix hexadecimal} {{/IKA87AD_tb/ram[75]} -radix hexadecimal} {{/IKA87AD_tb/ram[76]} -radix hexadecimal} {{/IKA87AD_tb/ram[77]} -radix hexadecimal} {{/IKA87AD_tb/ram[78]} -radix hexadecimal} {{/IKA87AD_tb/ram[79]} -radix hexadecimal} {{/IKA87AD_tb/ram[80]} -radix hexadecimal} {{/IKA87AD_tb/ram[81]} -radix hexadecimal} {{/IKA87AD_tb/ram[82]} -radix hexadecimal} {{/IKA87AD_tb/ram[83]} -radix hexadecimal} {{/IKA87AD_tb/ram[84]} -radix hexadecimal} {{/IKA87AD_tb/ram[85]} -radix hexadecimal} {{/IKA87AD_tb/ram[86]} -radix hexadecimal} {{/IKA87AD_tb/ram[87]} -radix hexadecimal} {{/IKA87AD_tb/ram[88]} -radix hexadecimal} {{/IKA87AD_tb/ram[89]} -radix hexadecimal} {{/IKA87AD_tb/ram[90]} -radix hexadecimal} {{/IKA87AD_tb/ram[91]} -radix hexadecimal} {{/IKA87AD_tb/ram[92]} -radix hexadecimal} {{/IKA87AD_tb/ram[93]} -radix hexadecimal} {{/IKA87AD_tb/ram[94]} -radix hexadecimal} {{/IKA87AD_tb/ram[95]} -radix hexadecimal} {{/IKA87AD_tb/ram[96]} -radix hexadecimal} {{/IKA87AD_tb/ram[97]} -radix hexadecimal} {{/IKA87AD_tb/ram[98]} -radix hexadecimal} {{/IKA87AD_tb/ram[99]} -radix hexadecimal} {{/IKA87AD_tb/ram[100]} -radix hexadecimal} {{/IKA87AD_tb/ram[101]} -radix hexadecimal} {{/IKA87AD_tb/ram[102]} -radix hexadecimal} {{/IKA87AD_tb/ram[103]} -radix hexadecimal} {{/IKA87AD_tb/ram[104]} -radix hexadecimal} {{/IKA87AD_tb/ram[105]} -radix hexadecimal} {{/IKA87AD_tb/ram[106]} -radix hexadecimal} {{/IKA87AD_tb/ram[107]} -radix hexadecimal} {{/IKA87AD_tb/ram[108]} -radix hexadecimal} {{/IKA87AD_tb/ram[109]} -radix hexadecimal} {{/IKA87AD_tb/ram[110]} -radix hexadecimal} {{/IKA87AD_tb/ram[111]} -radix hexadecimal} {{/IKA87AD_tb/ram[112]} -radix hexadecimal} {{/IKA87AD_tb/ram[113]} -radix hexadecimal} {{/IKA87AD_tb/ram[114]} -radix hexadecimal} {{/IKA87AD_tb/ram[115]} -radix hexadecimal} {{/IKA87AD_tb/ram[116]} -radix hexadecimal} {{/IKA87AD_tb/ram[117]} -radix hexadecimal} {{/IKA87AD_tb/ram[118]} -radix hexadecimal} {{/IKA87AD_tb/ram[119]} -radix hexadecimal} {{/IKA87AD_tb/ram[120]} -radix hexadecimal} {{/IKA87AD_tb/ram[121]} -radix hexadecimal} {{/IKA87AD_tb/ram[122]} -radix hexadecimal} {{/IKA87AD_tb/ram[123]} -radix hexadecimal} {{/IKA87AD_tb/ram[124]} -radix hexadecimal} {{/IKA87AD_tb/ram[125]} -radix hexadecimal} {{/IKA87AD_tb/ram[126]} -radix hexadecimal} {{/IKA87AD_tb/ram[127]} -radix hexadecimal} {{/IKA87AD_tb/ram[128]} -radix hexadecimal} {{/IKA87AD_tb/ram[129]} -radix hexadecimal} {{/IKA87AD_tb/ram[130]} -radix hexadecimal} {{/IKA87AD_tb/ram[131]} -radix hexadecimal} {{/IKA87AD_tb/ram[132]} -radix hexadecimal} {{/IKA87AD_tb/ram[133]} -radix hexadecimal} {{/IKA87AD_tb/ram[134]} -radix hexadecimal} {{/IKA87AD_tb/ram[135]} -radix hexadecimal} {{/IKA87AD_tb/ram[136]} -radix hexadecimal} {{/IKA87AD_tb/ram[137]} -radix hexadecimal} {{/IKA87AD_tb/ram[138]} -radix hexadecimal} {{/IKA87AD_tb/ram[139]} -radix hexadecimal} {{/IKA87AD_tb/ram[140]} -radix hexadecimal} {{/IKA87AD_tb/ram[141]} -radix hexadecimal} {{/IKA87AD_tb/ram[142]} -radix hexadecimal} {{/IKA87AD_tb/ram[143]} -radix hexadecimal} {{/IKA87AD_tb/ram[144]} -radix hexadecimal} {{/IKA87AD_tb/ram[145]} -radix hexadecimal} {{/IKA87AD_tb/ram[146]} -radix hexadecimal} {{/IKA87AD_tb/ram[147]} -radix hexadecimal} {{/IKA87AD_tb/ram[148]} -radix hexadecimal} {{/IKA87AD_tb/ram[149]} -radix hexadecimal} {{/IKA87AD_tb/ram[150]} -radix hexadecimal} {{/IKA87AD_tb/ram[151]} -radix hexadecimal} {{/IKA87AD_tb/ram[152]} -radix hexadecimal} {{/IKA87AD_tb/ram[153]} -radix hexadecimal} {{/IKA87AD_tb/ram[154]} -radix hexadecimal} {{/IKA87AD_tb/ram[155]} -radix hexadecimal} {{/IKA87AD_tb/ram[156]} -radix hexadecimal} {{/IKA87AD_tb/ram[157]} -radix hexadecimal} {{/IKA87AD_tb/ram[158]} -radix hexadecimal} {{/IKA87AD_tb/ram[159]} -radix hexadecimal} {{/IKA87AD_tb/ram[160]} -radix hexadecimal} {{/IKA87AD_tb/ram[161]} -radix hexadecimal} {{/IKA87AD_tb/ram[162]} -radix hexadecimal} {{/IKA87AD_tb/ram[163]} -radix hexadecimal} {{/IKA87AD_tb/ram[164]} -radix hexadecimal} {{/IKA87AD_tb/ram[165]} -radix hexadecimal} {{/IKA87AD_tb/ram[166]} -radix hexadecimal} {{/IKA87AD_tb/ram[167]} -radix hexadecimal} {{/IKA87AD_tb/ram[168]} -radix hexadecimal} {{/IKA87AD_tb/ram[169]} -radix hexadecimal} {{/IKA87AD_tb/ram[170]} -radix hexadecimal} {{/IKA87AD_tb/ram[171]} -radix hexadecimal} {{/IKA87AD_tb/ram[172]} -radix hexadecimal} {{/IKA87AD_tb/ram[173]} -radix hexadecimal} {{/IKA87AD_tb/ram[174]} -radix hexadecimal} {{/IKA87AD_tb/ram[175]} -radix hexadecimal} {{/IKA87AD_tb/ram[176]} -radix hexadecimal} {{/IKA87AD_tb/ram[177]} -radix hexadecimal} {{/IKA87AD_tb/ram[178]} -radix hexadecimal} {{/IKA87AD_tb/ram[179]} -radix hexadecimal} {{/IKA87AD_tb/ram[180]} -radix hexadecimal} {{/IKA87AD_tb/ram[181]} -radix hexadecimal} {{/IKA87AD_tb/ram[182]} -radix hexadecimal} {{/IKA87AD_tb/ram[183]} -radix hexadecimal} {{/IKA87AD_tb/ram[184]} -radix hexadecimal} {{/IKA87AD_tb/ram[185]} -radix hexadecimal} {{/IKA87AD_tb/ram[186]} -radix hexadecimal} {{/IKA87AD_tb/ram[187]} -radix hexadecimal} {{/IKA87AD_tb/ram[188]} -radix hexadecimal} {{/IKA87AD_tb/ram[189]} -radix hexadecimal} {{/IKA87AD_tb/ram[190]} -radix hexadecimal} {{/IKA87AD_tb/ram[191]} -radix hexadecimal} {{/IKA87AD_tb/ram[192]} -radix hexadecimal} {{/IKA87AD_tb/ram[193]} -radix hexadecimal} {{/IKA87AD_tb/ram[194]} -radix hexadecimal} {{/IKA87AD_tb/ram[195]} -radix hexadecimal} {{/IKA87AD_tb/ram[196]} -radix hexadecimal} {{/IKA87AD_tb/ram[197]} -radix hexadecimal} {{/IKA87AD_tb/ram[198]} -radix hexadecimal} {{/IKA87AD_tb/ram[199]} -radix hexadecimal} {{/IKA87AD_tb/ram[200]} -radix hexadecimal} {{/IKA87AD_tb/ram[201]} -radix hexadecimal} {{/IKA87AD_tb/ram[202]} -radix hexadecimal} {{/IKA87AD_tb/ram[203]} -radix hexadecimal} {{/IKA87AD_tb/ram[204]} -radix hexadecimal} {{/IKA87AD_tb/ram[205]} -radix hexadecimal} {{/IKA87AD_tb/ram[206]} -radix hexadecimal} {{/IKA87AD_tb/ram[207]} -radix hexadecimal} {{/IKA87AD_tb/ram[208]} -radix hexadecimal} {{/IKA87AD_tb/ram[209]} -radix hexadecimal} {{/IKA87AD_tb/ram[210]} -radix hexadecimal} {{/IKA87AD_tb/ram[211]} -radix hexadecimal} {{/IKA87AD_tb/ram[212]} -radix hexadecimal} {{/IKA87AD_tb/ram[213]} -radix hexadecimal} {{/IKA87AD_tb/ram[214]} -radix hexadecimal} {{/IKA87AD_tb/ram[215]} -radix hexadecimal} {{/IKA87AD_tb/ram[216]} -radix hexadecimal} {{/IKA87AD_tb/ram[217]} -radix hexadecimal} {{/IKA87AD_tb/ram[218]} -radix hexadecimal} {{/IKA87AD_tb/ram[219]} -radix hexadecimal} {{/IKA87AD_tb/ram[220]} -radix hexadecimal} {{/IKA87AD_tb/ram[221]} -radix hexadecimal} {{/IKA87AD_tb/ram[222]} -radix hexadecimal} {{/IKA87AD_tb/ram[223]} -radix hexadecimal} {{/IKA87AD_tb/ram[224]} -radix hexadecimal} {{/IKA87AD_tb/ram[225]} -radix hexadecimal} {{/IKA87AD_tb/ram[226]} -radix hexadecimal} {{/IKA87AD_tb/ram[227]} -radix hexadecimal} {{/IKA87AD_tb/ram[228]} -radix hexadecimal} {{/IKA87AD_tb/ram[229]} -radix hexadecimal} {{/IKA87AD_tb/ram[230]} -radix hexadecimal} {{/IKA87AD_tb/ram[231]} -radix hexadecimal} {{/IKA87AD_tb/ram[232]} -radix hexadecimal} {{/IKA87AD_tb/ram[233]} -radix hexadecimal} {{/IKA87AD_tb/ram[234]} -radix hexadecimal} {{/IKA87AD_tb/ram[235]} -radix hexadecimal} {{/IKA87AD_tb/ram[236]} -radix hexadecimal} {{/IKA87AD_tb/ram[237]} -radix hexadecimal} {{/IKA87AD_tb/ram[238]} -radix hexadecimal} {{/IKA87AD_tb/ram[239]} -radix hexadecimal} {{/IKA87AD_tb/ram[240]} -radix hexadecimal} {{/IKA87AD_tb/ram[241]} -radix hexadecimal} {{/IKA87AD_tb/ram[242]} -radix hexadecimal} {{/IKA87AD_tb/ram[243]} -radix hexadecimal} {{/IKA87AD_tb/ram[244]} -radix hexadecimal} {{/IKA87AD_tb/ram[245]} -radix hexadecimal} {{/IKA87AD_tb/ram[246]} -radix hexadecimal} {{/IKA87AD_tb/ram[247]} -radix hexadecimal} {{/IKA87AD_tb/ram[248]} -radix hexadecimal} {{/IKA87AD_tb/ram[249]} -radix hexadecimal} {{/IKA87AD_tb/ram[250]} -radix hexadecimal} {{/IKA87AD_tb/ram[251]} -radix hexadecimal} {{/IKA87AD_tb/ram[252]} -radix hexadecimal} {{/IKA87AD_tb/ram[253]} -radix hexadecimal} {{/IKA87AD_tb/ram[254]} -radix hexadecimal} {{/IKA87AD_tb/ram[255]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/ram[0]} {-radix hexadecimal} {/IKA87AD_tb/ram[1]} {-radix hexadecimal} {/IKA87AD_tb/ram[2]} {-radix hexadecimal} {/IKA87AD_tb/ram[3]} {-radix hexadecimal} {/IKA87AD_tb/ram[4]} {-radix hexadecimal} {/IKA87AD_tb/ram[5]} {-radix hexadecimal} {/IKA87AD_tb/ram[6]} {-radix hexadecimal} {/IKA87AD_tb/ram[7]} {-radix hexadecimal} {/IKA87AD_tb/ram[8]} {-radix hexadecimal} {/IKA87AD_tb/ram[9]} {-radix hexadecimal} {/IKA87AD_tb/ram[10]} {-radix hexadecimal} {/IKA87AD_tb/ram[11]} {-radix hexadecimal} {/IKA87AD_tb/ram[12]} {-radix hexadecimal} {/IKA87AD_tb/ram[13]} {-radix hexadecimal} {/IKA87AD_tb/ram[14]} {-radix hexadecimal} {/IKA87AD_tb/ram[15]} {-radix hexadecimal} {/IKA87AD_tb/ram[16]} {-radix hexadecimal} {/IKA87AD_tb/ram[17]} {-radix hexadecimal} {/IKA87AD_tb/ram[18]} {-radix hexadecimal} {/IKA87AD_tb/ram[19]} {-radix hexadecimal} {/IKA87AD_tb/ram[20]} {-radix hexadecimal} {/IKA87AD_tb/ram[21]} {-radix hexadecimal} {/IKA87AD_tb/ram[22]} {-radix hexadecimal} {/IKA87AD_tb/ram[23]} {-radix hexadecimal} {/IKA87AD_tb/ram[24]} {-radix hexadecimal} {/IKA87AD_tb/ram[25]} {-radix hexadecimal} {/IKA87AD_tb/ram[26]} {-radix hexadecimal} {/IKA87AD_tb/ram[27]} {-radix hexadecimal} {/IKA87AD_tb/ram[28]} {-radix hexadecimal} {/IKA87AD_tb/ram[29]} {-radix hexadecimal} {/IKA87AD_tb/ram[30]} {-radix hexadecimal} {/IKA87AD_tb/ram[31]} {-radix hexadecimal} {/IKA87AD_tb/ram[32]} {-radix hexadecimal} {/IKA87AD_tb/ram[33]} {-radix hexadecimal} {/IKA87AD_tb/ram[34]} {-radix hexadecimal} {/IKA87AD_tb/ram[35]} {-radix hexadecimal} {/IKA87AD_tb/ram[36]} {-radix hexadecimal} {/IKA87AD_tb/ram[37]} {-radix hexadecimal} {/IKA87AD_tb/ram[38]} {-radix hexadecimal} {/IKA87AD_tb/ram[39]} {-radix hexadecimal} {/IKA87AD_tb/ram[40]} {-radix hexadecimal} {/IKA87AD_tb/ram[41]} {-radix hexadecimal} {/IKA87AD_tb/ram[42]} {-radix hexadecimal} {/IKA87AD_tb/ram[43]} {-radix hexadecimal} {/IKA87AD_tb/ram[44]} {-radix hexadecimal} {/IKA87AD_tb/ram[45]} {-radix hexadecimal} {/IKA87AD_tb/ram[46]} {-radix hexadecimal} {/IKA87AD_tb/ram[47]} {-radix hexadecimal} {/IKA87AD_tb/ram[48]} {-radix hexadecimal} {/IKA87AD_tb/ram[49]} {-radix hexadecimal} {/IKA87AD_tb/ram[50]} {-radix hexadecimal} {/IKA87AD_tb/ram[51]} {-radix hexadecimal} {/IKA87AD_tb/ram[52]} {-radix hexadecimal} {/IKA87AD_tb/ram[53]} {-radix hexadecimal} {/IKA87AD_tb/ram[54]} {-radix hexadecimal} {/IKA87AD_tb/ram[55]} {-radix hexadecimal} {/IKA87AD_tb/ram[56]} {-radix hexadecimal} {/IKA87AD_tb/ram[57]} {-radix hexadecimal} {/IKA87AD_tb/ram[58]} {-radix hexadecimal} {/IKA87AD_tb/ram[59]} {-radix hexadecimal} {/IKA87AD_tb/ram[60]} {-radix hexadecimal} {/IKA87AD_tb/ram[61]} {-radix hexadecimal} {/IKA87AD_tb/ram[62]} {-radix hexadecimal} {/IKA87AD_tb/ram[63]} {-radix hexadecimal} {/IKA87AD_tb/ram[64]} {-radix hexadecimal} {/IKA87AD_tb/ram[65]} {-radix hexadecimal} {/IKA87AD_tb/ram[66]} {-radix hexadecimal} {/IKA87AD_tb/ram[67]} {-radix hexadecimal} {/IKA87AD_tb/ram[68]} {-radix hexadecimal} {/IKA87AD_tb/ram[69]} {-radix hexadecimal} {/IKA87AD_tb/ram[70]} {-radix hexadecimal} {/IKA87AD_tb/ram[71]} {-radix hexadecimal} {/IKA87AD_tb/ram[72]} {-radix hexadecimal} {/IKA87AD_tb/ram[73]} {-radix hexadecimal} {/IKA87AD_tb/ram[74]} {-radix hexadecimal} {/IKA87AD_tb/ram[75]} {-radix hexadecimal} {/IKA87AD_tb/ram[76]} {-radix hexadecimal} {/IKA87AD_tb/ram[77]} {-radix hexadecimal} {/IKA87AD_tb/ram[78]} {-radix hexadecimal} {/IKA87AD_tb/ram[79]} {-radix hexadecimal} {/IKA87AD_tb/ram[80]} {-radix hexadecimal} {/IKA87AD_tb/ram[81]} {-radix hexadecimal} {/IKA87AD_tb/ram[82]} {-radix hexadecimal} {/IKA87AD_tb/ram[83]} {-radix hexadecimal} {/IKA87AD_tb/ram[84]} {-radix hexadecimal} {/IKA87AD_tb/ram[85]} {-radix hexadecimal} {/IKA87AD_tb/ram[86]} {-radix hexadecimal} {/IKA87AD_tb/ram[87]} {-radix hexadecimal} {/IKA87AD_tb/ram[88]} {-radix hexadecimal} {/IKA87AD_tb/ram[89]} {-radix hexadecimal} {/IKA87AD_tb/ram[90]} {-radix hexadecimal} {/IKA87AD_tb/ram[91]} {-radix hexadecimal} {/IKA87AD_tb/ram[92]} {-radix hexadecimal} {/IKA87AD_tb/ram[93]} {-radix hexadecimal} {/IKA87AD_tb/ram[94]} {-radix hexadecimal} {/IKA87AD_tb/ram[95]} {-radix hexadecimal} {/IKA87AD_tb/ram[96]} {-radix hexadecimal} {/IKA87AD_tb/ram[97]} {-radix hexadecimal} {/IKA87AD_tb/ram[98]} {-radix hexadecimal} {/IKA87AD_tb/ram[99]} {-radix hexadecimal} {/IKA87AD_tb/ram[100]} {-radix hexadecimal} {/IKA87AD_tb/ram[101]} {-radix hexadecimal} {/IKA87AD_tb/ram[102]} {-radix hexadecimal} {/IKA87AD_tb/ram[103]} {-radix hexadecimal} {/IKA87AD_tb/ram[104]} {-radix hexadecimal} {/IKA87AD_tb/ram[105]} {-radix hexadecimal} {/IKA87AD_tb/ram[106]} {-radix hexadecimal} {/IKA87AD_tb/ram[107]} {-radix hexadecimal} {/IKA87AD_tb/ram[108]} {-radix hexadecimal} {/IKA87AD_tb/ram[109]} {-radix hexadecimal} {/IKA87AD_tb/ram[110]} {-radix hexadecimal} {/IKA87AD_tb/ram[111]} {-radix hexadecimal} {/IKA87AD_tb/ram[112]} {-radix hexadecimal} {/IKA87AD_tb/ram[113]} {-radix hexadecimal} {/IKA87AD_tb/ram[114]} {-radix hexadecimal} {/IKA87AD_tb/ram[115]} {-radix hexadecimal} {/IKA87AD_tb/ram[116]} {-radix hexadecimal} {/IKA87AD_tb/ram[117]} {-radix hexadecimal} {/IKA87AD_tb/ram[118]} {-radix hexadecimal} {/IKA87AD_tb/ram[119]} {-radix hexadecimal} {/IKA87AD_tb/ram[120]} {-radix hexadecimal} {/IKA87AD_tb/ram[121]} {-radix hexadecimal} {/IKA87AD_tb/ram[122]} {-radix hexadecimal} {/IKA87AD_tb/ram[123]} {-radix hexadecimal} {/IKA87AD_tb/ram[124]} {-radix hexadecimal} {/IKA87AD_tb/ram[125]} {-radix hexadecimal} {/IKA87AD_tb/ram[126]} {-radix hexadecimal} {/IKA87AD_tb/ram[127]} {-radix hexadecimal} {/IKA87AD_tb/ram[128]} {-radix hexadecimal} {/IKA87AD_tb/ram[129]} {-radix hexadecimal} {/IKA87AD_tb/ram[130]} {-radix hexadecimal} {/IKA87AD_tb/ram[131]} {-radix hexadecimal} {/IKA87AD_tb/ram[132]} {-radix hexadecimal} {/IKA87AD_tb/ram[133]} {-radix hexadecimal} {/IKA87AD_tb/ram[134]} {-radix hexadecimal} {/IKA87AD_tb/ram[135]} {-radix hexadecimal} {/IKA87AD_tb/ram[136]} {-radix hexadecimal} {/IKA87AD_tb/ram[137]} {-radix hexadecimal} {/IKA87AD_tb/ram[138]} {-radix hexadecimal} {/IKA87AD_tb/ram[139]} {-radix hexadecimal} {/IKA87AD_tb/ram[140]} {-radix hexadecimal} {/IKA87AD_tb/ram[141]} {-radix hexadecimal} {/IKA87AD_tb/ram[142]} {-radix hexadecimal} {/IKA87AD_tb/ram[143]} {-radix hexadecimal} {/IKA87AD_tb/ram[144]} {-radix hexadecimal} {/IKA87AD_tb/ram[145]} {-radix hexadecimal} {/IKA87AD_tb/ram[146]} {-radix hexadecimal} {/IKA87AD_tb/ram[147]} {-radix hexadecimal} {/IKA87AD_tb/ram[148]} {-radix hexadecimal} {/IKA87AD_tb/ram[149]} {-radix hexadecimal} {/IKA87AD_tb/ram[150]} {-radix hexadecimal} {/IKA87AD_tb/ram[151]} {-radix hexadecimal} {/IKA87AD_tb/ram[152]} {-radix hexadecimal} {/IKA87AD_tb/ram[153]} {-radix hexadecimal} {/IKA87AD_tb/ram[154]} {-radix hexadecimal} {/IKA87AD_tb/ram[155]} {-radix hexadecimal} {/IKA87AD_tb/ram[156]} {-radix hexadecimal} {/IKA87AD_tb/ram[157]} {-radix hexadecimal} {/IKA87AD_tb/ram[158]} {-radix hexadecimal} {/IKA87AD_tb/ram[159]} {-radix hexadecimal} {/IKA87AD_tb/ram[160]} {-radix hexadecimal} {/IKA87AD_tb/ram[161]} {-radix hexadecimal} {/IKA87AD_tb/ram[162]} {-radix hexadecimal} {/IKA87AD_tb/ram[163]} {-radix hexadecimal} {/IKA87AD_tb/ram[164]} {-radix hexadecimal} {/IKA87AD_tb/ram[165]} {-radix hexadecimal} {/IKA87AD_tb/ram[166]} {-radix hexadecimal} {/IKA87AD_tb/ram[167]} {-radix hexadecimal} {/IKA87AD_tb/ram[168]} {-radix hexadecimal} {/IKA87AD_tb/ram[169]} {-radix hexadecimal} {/IKA87AD_tb/ram[170]} {-radix hexadecimal} {/IKA87AD_tb/ram[171]} {-radix hexadecimal} {/IKA87AD_tb/ram[172]} {-radix hexadecimal} {/IKA87AD_tb/ram[173]} {-radix hexadecimal} {/IKA87AD_tb/ram[174]} {-radix hexadecimal} {/IKA87AD_tb/ram[175]} {-radix hexadecimal} {/IKA87AD_tb/ram[176]} {-radix hexadecimal} {/IKA87AD_tb/ram[177]} {-radix hexadecimal} {/IKA87AD_tb/ram[178]} {-radix hexadecimal} {/IKA87AD_tb/ram[179]} {-radix hexadecimal} {/IKA87AD_tb/ram[180]} {-radix hexadecimal} {/IKA87AD_tb/ram[181]} {-radix hexadecimal} {/IKA87AD_tb/ram[182]} {-radix hexadecimal} {/IKA87AD_tb/ram[183]} {-radix hexadecimal} {/IKA87AD_tb/ram[184]} {-radix hexadecimal} {/IKA87AD_tb/ram[185]} {-radix hexadecimal} {/IKA87AD_tb/ram[186]} {-radix hexadecimal} {/IKA87AD_tb/ram[187]} {-radix hexadecimal} {/IKA87AD_tb/ram[188]} {-radix hexadecimal} {/IKA87AD_tb/ram[189]} {-radix hexadecimal} {/IKA87AD_tb/ram[190]} {-radix hexadecimal} {/IKA87AD_tb/ram[191]} {-radix hexadecimal} {/IKA87AD_tb/ram[192]} {-radix hexadecimal} {/IKA87AD_tb/ram[193]} {-radix hexadecimal} {/IKA87AD_tb/ram[194]} {-radix hexadecimal} {/IKA87AD_tb/ram[195]} {-radix hexadecimal} {/IKA87AD_tb/ram[196]} {-radix hexadecimal} {/IKA87AD_tb/ram[197]} {-radix hexadecimal} {/IKA87AD_tb/ram[198]} {-radix hexadecimal} {/IKA87AD_tb/ram[199]} {-radix hexadecimal} {/IKA87AD_tb/ram[200]} {-radix hexadecimal} {/IKA87AD_tb/ram[201]} {-radix hexadecimal} {/IKA87AD_tb/ram[202]} {-radix hexadecimal} {/IKA87AD_tb/ram[203]} {-radix hexadecimal} {/IKA87AD_tb/ram[204]} {-radix hexadecimal} {/IKA87AD_tb/ram[205]} {-radix hexadecimal} {/IKA87AD_tb/ram[206]} {-radix hexadecimal} {/IKA87AD_tb/ram[207]} {-radix hexadecimal} {/IKA87AD_tb/ram[208]} {-radix hexadecimal} {/IKA87AD_tb/ram[209]} {-radix hexadecimal} {/IKA87AD_tb/ram[210]} {-radix hexadecimal} {/IKA87AD_tb/ram[211]} {-radix hexadecimal} {/IKA87AD_tb/ram[212]} {-radix hexadecimal} {/IKA87AD_tb/ram[213]} {-radix hexadecimal} {/IKA87AD_tb/ram[214]} {-radix hexadecimal} {/IKA87AD_tb/ram[215]} {-radix hexadecimal} {/IKA87AD_tb/ram[216]} {-radix hexadecimal} {/IKA87AD_tb/ram[217]} {-radix hexadecimal} {/IKA87AD_tb/ram[218]} {-radix hexadecimal} {/IKA87AD_tb/ram[219]} {-radix hexadecimal} {/IKA87AD_tb/ram[220]} {-radix hexadecimal} {/IKA87AD_tb/ram[221]} {-radix hexadecimal} {/IKA87AD_tb/ram[222]} {-radix hexadecimal} {/IKA87AD_tb/ram[223]} {-radix hexadecimal} {/IKA87AD_tb/ram[224]} {-radix hexadecimal} {/IKA87AD_tb/ram[225]} {-radix hexadecimal} {/IKA87AD_tb/ram[226]} {-radix hexadecimal} {/IKA87AD_tb/ram[227]} {-radix hexadecimal} {/IKA87AD_tb/ram[228]} {-radix hexadecimal} {/IKA87AD_tb/ram[229]} {-radix hexadecimal} {/IKA87AD_tb/ram[230]} {-radix hexadecimal} {/IKA87AD_tb/ram[231]} {-radix hexadecimal} {/IKA87AD_tb/ram[232]} {-radix hexadecimal} {/IKA87AD_tb/ram[233]} {-radix hexadecimal} {/IKA87AD_tb/ram[234]} {-radix hexadecimal} {/IKA87AD_tb/ram[235]} {-radix hexadecimal} {/IKA87AD_tb/ram[236]} {-radix hexadecimal} {/IKA87AD_tb/ram[237]} {-radix hexadecimal} {/IKA87AD_tb/ram[238]} {-radix hexadecimal} {/IKA87AD_tb/ram[239]} {-radix hexadecimal} {/IKA87AD_tb/ram[240]} {-radix hexadecimal} {/IKA87AD_tb/ram[241]} {-radix hexadecimal} {/IKA87AD_tb/ram[242]} {-radix hexadecimal} {/IKA87AD_tb/ram[243]} {-radix hexadecimal} {/IKA87AD_tb/ram[244]} {-radix hexadecimal} {/IKA87AD_tb/ram[245]} {-radix hexadecimal} {/IKA87AD_tb/ram[246]} {-radix hexadecimal} {/IKA87AD_tb/ram[247]} {-radix hexadecimal} {/IKA87AD_tb/ram[248]} {-radix hexadecimal} {/IKA87AD_tb/ram[249]} {-radix hexadecimal} {/IKA87AD_tb/ram[250]} {-radix hexadecimal} {/IKA87AD_tb/ram[251]} {-radix hexadecimal} {/IKA87AD_tb/ram[252]} {-radix hexadecimal} {/IKA87AD_tb/ram[253]} {-radix hexadecimal} {/IKA87AD_tb/ram[254]} {-radix hexadecimal} {/IKA87AD_tb/ram[255]} {-radix hexadecimal}} /IKA87AD_tb/ram
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/timing_sr
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/cycle_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/opcode_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/rw_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/md_inlatch_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/i_STOP_n
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_halt_detected
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_stop_detected
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_halt_flag
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_stop_flag
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/hard_stop_flag
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/sr_stop
add wave -noupdate -group SUSP {/IKA87AD_tb/u_dut/stop_syncchain[1]}
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/force_exec_nop
add wave -noupdate -group SUSP -radix decimal /IKA87AD_tb/u_dut/hstop_osc_wait
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/hstop_osc_unstable
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_Z
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_C
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_SK
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_HC
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_L1
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_L0
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/i_NMI_n
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/i_INT1
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/i_INT2_n
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_mask_n
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/iflag
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_enabled
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_pending
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_detected
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/force_exec_hardi
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/hardi_proc_cyc
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/softi_proc_cyc
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/iflag_manual_ack
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/iflag_auto_ack
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_lv
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_lv_z
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_M1_n
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_IO_n
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/mc_next_bus_acc
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_A
add wave -noupdate -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/i_DI
add wave -noupdate -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_DO
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_PD_DO_OE
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_DO_OE
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_ALE
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_RD_n
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/o_WR_n
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/mc_s_cond_read
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/md_in_byte_sel
add wave -noupdate -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDI
add wave -noupdate -group BUS /IKA87AD_tb/u_dut/md_dirty
add wave -noupdate -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDH
add wave -noupdate -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDL
add wave -noupdate -group BUS -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mseq_state
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/is_arith_eval_op
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_s_bra_on_alu
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_jump_to_next_inst
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/opcode_page
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/opcode_inlatch_tick
add wave -noupdate -group {MC ENGINE} -radix hexadecimal /IKA87AD_tb/u_dut/reg_OPCODE
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_sa
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mseq_cntr
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_addr
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_data
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_mul_start
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_div_start
add wave -noupdate -group {ALU MULDIV} -radix unsigned /IKA87AD_tb/u_dut/alu_muldiv_cntr
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_mul_pa
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_mul_pb
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_div_pa
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_div_pb
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_div_out
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_muldiv_reg_TEMP_wr
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_muldiv_reg_EA_wr
add wave -noupdate -radix hexadecimal /IKA87AD_tb/u_dut/reg_TEMP
add wave -noupdate -group ALU /IKA87AD_tb/u_dut/sreg_RDBUS
add wave -noupdate -group ALU /IKA87AD_tb/u_dut/mc_type
add wave -noupdate -group ALU /IKA87AD_tb/u_dut/mc_sa_dst
add wave -noupdate -group ALU /IKA87AD_tb/u_dut/mc_sc_dst
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_pa
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_pb
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_op0
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_op1
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_out
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_output
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_ma_output
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_bk_carry_ctrl
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_ctrl_output
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_type
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_jump_to_next_inst
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_next_bus_acc
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_PC_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_SP_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_MA_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_MDL_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_MDH_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_EAL_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_EAH_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_V_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_A_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_B_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_C_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_D_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_E_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_H_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_L_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_TEMP_wr
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_PC
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_SP
add wave -noupdate -group {REGFILE PCSP} /IKA87AD_tb/u_dut/reg_MA_inc_ndec
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_MA
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_TEMP
add wave -noupdate -expand -group {REGFILE VAEA} /IKA87AD_tb/u_dut/sel_VAEA
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAH[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAL[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_V[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_A[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAH[1]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAL[1]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_V[1]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_A[1]}
add wave -noupdate -expand -group {REGFILE BCDE} /IKA87AD_tb/u_dut/sel_BCDE
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_B[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_C[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_D[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_E[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_B[1]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_C[1]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_D[1]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_E[1]}
add wave -noupdate -expand -group {REGFILE HL} /IKA87AD_tb/u_dut/sel_VAEA
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_H[0]}
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_L[0]}
add wave -noupdate -expand -group {REGFILE HL} /IKA87AD_tb/u_dut/reg_TEMP_wr
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_H[1]}
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_L[1]}
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PA_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PA_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PA_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PB_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PB_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PB_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PC_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PC_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PC_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PD_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PD_O
add wave -noupdate -group PORTS /IKA87AD_tb/u_dut/o_PD_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PF_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PF_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PF_OE
add wave -noupdate -group ADC /IKA87AD_tb/u_dut/sreg_ANM
add wave -noupdate -group ADC /IKA87AD_tb/u_dut/current_adc_mode
add wave -noupdate -group ADC -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/sreg_CR[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/sreg_CR[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/sreg_CR[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/sreg_CR[3]} -radix hexadecimal}} -expand -subitemconfig {{/IKA87AD_tb/u_dut/sreg_CR[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/sreg_CR[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/sreg_CR[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/sreg_CR[3]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/sreg_CR
add wave -noupdate -group ADC /IKA87AD_tb/u_dut/adc_tick
add wave -noupdate -group ADC -radix unsigned /IKA87AD_tb/u_dut/adc_state_cntr
add wave -noupdate -group ADC -radix unsigned /IKA87AD_tb/u_dut/o_ADC_CH
add wave -noupdate -group ADC /IKA87AD_tb/u_dut/o_ADC_RD_n
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sr_wr_addr
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sr_rd_addr
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_PAO
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_PBO
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_PCO
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_PDO
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_PFO
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MA
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MB
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MC
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MF
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MM
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MCC
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MKL
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_MKH
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_ANM
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_SMH
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_SML
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_EOM
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_ETMM
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_TMM
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_TM0
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_TM1
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_ZCM
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_ETM0
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_ETM1
add wave -noupdate -group SR /IKA87AD_tb/u_dut/sreg_CR
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {1527330 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 166
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {973120 ps} {1843520 ps}
