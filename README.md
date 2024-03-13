# IKA87AD
NEC uCOM87AD microcontroller Verilog core for FPGA implementation. It was created using information from a datasheet, **so its behavior may differ from the actual chip.** © 2024 Sehyeon Kim(Raki)

## Features
* A near cycle-accurate, BSD2 licensed core.
* Emulates the CMOS version of the CPU.

## Module instantiation
The steps below show how to instantiate the IKA87AD module in Verilog:

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
    .o_D_nA_SEL                     (                           ),
    .o_DO_OE                        (                           ),

    .o_REG_MM                       (                           ),

    .i_NMI_n                        (                           ),
    .i_INT1                         (                           ),
    .i_INT2_n                       (                           ),

    .i_TI                           (                           ),
    .o_TO                           (                           ),
    .o_TO_PCEN                      (                           ),
    .o_TO_NCEN                      (                           ),

    .i_CI                           (                           ),

    .i_PA_I                         (                           ),
    .o_PA_O                         (                           ),
    .o_PA_OE                        (                           ),

    .i_PB_I                         (                           ),
    .o_PB_O                         (                           ),
    .o_PB_OE                        (                           ),

    .i_PC_I                         (                           ),
    .o_PC_O                         (                           ),
    .o_PC_OE                        (                           ),
    .o_REG_MCC                      (                           )

    .i_PD_I                         (                           ),
    .o_PD_O                         (                           ),
    .o_PD_OE                        (                           ),

    .i_PF_I                         (                           ),
    .o_PF_O                         (                           ),
    .o_PF_OE                        (                           ),

    .i_ANx_DIGITAL                  (                           ),

    .o_ANx_ANALOG_CH                (                           ),
    .i_ANx_ANALOG_DATA              (                           ),
    .o_ANx_ANALOG_RD_n              (                           )
);
```
3. Attach your signals to the port. The direction and the polarity of the signals are described in the port names. The section below explains what the signals mean.

### System ports
* `i_EMUCLK` is your system clock.
* `i_MCUCLK_PCEN` is the clock enable(positive logic) for positive edge of the microcontroller master clock.
* `i_RESET_n` is the synchronous reset.
* `i_STOP_n` is the input for stopping the CPU core. See the datasheet for more information. To change the time it takes for the CPU to resume opcode fetch after being released from a stop state, change a parameter value `HARD_STOP_RELEASE_WAIT` inside the module.

### Bus controller
* `o_M1_n` indicates that the CPU is fetching an opcode. This is originally achieved by pulling the MODE1 pin to GND.
* `o_IO_n` indicates that the CPU is reading/writing data. This is originally achieved by pulling the MODE0 pin to GND.
* `o_ALE`, `o_RD_n`, `o_WR_n` are the asynchronous bus control signals.
* `o_A` is the address output sourced by ther program counter.
* `i_DI` and `o_DO` are the data input/output port, respectively.
* The original chip uses the port D for address low and for data IO. `o_PD_DO_OE` is the output enable of the multiplexed bus.
* `o_D_nA_SEL` selects the output mode of the multiplexed bus.
* You can still use the dedicated data buses instead of the multiplexed bus. In this case, you can use `o_DO_OE` to control output drivers.
* `o_REG_MM` is the MM register output.

### Interrupt inputs
* `i_NMI_n`, `i_INT1`, `i_INT2_n` are the interrupt inputs.

### Timer and event counter
* `i_TI`, `o_TO`, `i_CI` are ther timer/event counter related signals. See the datasheet for the details.
* `o_TO_PCEN` and `o_TO_NCEN` is positive/negative clock enable of the TO output. These output make synchronized circuits easy.
* `i_Px_I`, `o_Px_O`, `o_Px_OE` are the GPIO ports. Note that the port D can only control bytewise direction.
* `o_REG_MCC` is the MCC register output.

### ADC interface
* `i_ANx_DIGITAL` is the digital input port for the port AN7-4. They can perform negative edge detection.
* `o_ANx_ANALOG_CH` selects external ADCs.
* `i_ANx_ANALOG_DATA` grabs data from the external ADCs.
* `o_ANx_ANALOG_RD_n` is the ADC read strobe. It samples data at the rising edge.

## Implementation note
The original chip can change the memory map with the MODE1/0 pin and the MM register. Instead of implementing the original hardware, I added the `o_A` to output the current address of the program counter directly, to give the user flexibility. The entire 16-bit address space that the CPU core can access goes to the `o_A` port. It's up to you to decide how you want to arrange storage components.