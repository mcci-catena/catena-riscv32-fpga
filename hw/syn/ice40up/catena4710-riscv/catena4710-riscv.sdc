# ##############################################################################

# iCEcube SDC

# Version:            2017.01.27914

# File Generated:     Dec 23 2018 14:53:40

# ##############################################################################

####---- CreateClock list ----1
create_clock  -period 83.33 -name {clk_hf} [get_pins {OSCInst0/CLKHF}]

####---- CreateGenClock list ----2
create_generated_clock  [get_pins {clk_i_divider/Q}]  -source [get_pins {OSCInst0/CLKHF}]  -divide_by 2 -name {clk_i}
create_generated_clock  [get_pins {pdm_audio_inst.pdm_audio_clk_inst.pdm_clk_buf/GLOBAL_BUFFER_OUTPUT}]  -source [get_pins {OSCInst0/CLKHF}]  -divide_by 4 -name {pdm_clk}
