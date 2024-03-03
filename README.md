# IKA87AD
NEC uCOM87AD microcontroller Verilog core for FPGA implementation. It was created using information from a datasheet, **so its behavior may differ from the actual chip.** © 2024 Sehyeon Kim(Raki)

## Features
* A near cycle-accurate, BSD2 licensed core.
* Emulates the CMOS version of the CPU.

## Module instantiation
The steps below show how to instantiate the IKAOPM module in Verilog:

1. Download this repository or add it as a submodule to your project.
2. You can use the Verilog snippet below to instantiate the module.

```verilog
//Verilog module instantiation example
IKA87AD u_upd78c11 (
    .i_EMUCLK                       (                           ),
    .i_MCUCLK_PCEN                  (                           ),

    .i_RESET_n                      (                           ),
    .i_STOP_n                       (                           ),

    .o_M1_n                         (                           ),
    .o_IO_n                         (                           ),

    .o_ALE                          (                           ),
    .o_RD_n                         (                           ),
    .o_WR_n                         (                           ),
    .o_ALE_OE                       (                           ),
    .o_RD_n_OE                      (                           ),
    .o_WR_n_OE                      (                           ),

    .o_A                            (                           ),
    .i_DI                           (                           ),
    .o_DO                           (                           ),
    .o_PD_DO_OE                     (                           ),
    .o_DO_OE                        (                           ),

    .o_MEMSTRUCT                    (                           ),

    .i_NMI_n                        (                           ),
    .i_INT1                         (                           ),
    .i_INT2_n                       (                           ),

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
```
3. Attach your signals to the port. The direction and the polarity of the signals are described in the port names. The section below explains what the signals mean.


* `i_EMUCLK` is your system clock.
* `i_MCUCLK_PCEN` is the clock enable(positive logic) for positive edge of the microcontroller master clock.
* `i_RESET_n` is the synchronous reset.
* `i_STOP_n` is the input for stopping the CPU core. See the datasheet for more information. To change the time it takes for the CPU to resume opcode fetch after being released from a stop state, change a parameter value `HARD_STOP_RELEASE_WAIT` inside the module.
* `o_M1_n` indicates that the CPU is fetching an opcode. This is originally achieved by pulling the MODE1 pin to GND.
* `o_IO_n` indicates that the CPU is reading/writing data. This is originally achieved by pulling the MODE0 pin to GND.