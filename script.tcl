set cycle 4       ;	#clock period defined by designer(ns)
set DESIGN Top
set_host_options -max_cores 16
#Read Design File
read_file -format verilog -autoread -top $DESIGN -recursive {./src}
current_design [get_designs $DESIGN]
link


#Setting Clock Constraints
create_clock -name clk -period $cycle [get_ports clk]  

set_dont_touch_network              [get_clocks clk]
set_fix_hold                        [get_clocks clk]
set_clock_uncertainty       0.1     [get_clocks clk]
set_clock_latency   -source 0       [get_clocks clk]
set_clock_latency           0.5     [get_clocks clk]  
set_input_transition        0.5     [all_inputs]
set_clock_transition        0.1     [all_clocks]


#Chip-level synthesis
#set_dont_touch [get_cells ipad_*]
#set_dont_touch [get_cells opad_*]

#Setting Design Environment
set_operating_conditions -min_library fast -min fast  -max_library slow -max slow
#set_wire_load_model -name tsmc090_wl10 -library slow



set_input_delay   -max 1    -clock clk   [all_inputs]
set_input_delay   -min 1  -clock clk   [all_inputs]
set_output_delay  -max 1   -clock clk   [all_outputs]
set_output_delay  -min 1  -clock clk   [all_outputs]


#Setting DRC Constraint
set_max_area        0
set_max_fanout      30    [all_inputs]
set_max_transition  0.5  [all_inputs]


set_ideal_network   -no_propagate    [get_nets clk]
set_ideal_network   -no_propagate    [get_nets rst_n] 
set high_fanout_net_threshold 30
set high_fanout_net_pin_capacitance 0.01

#Solve Multiple Instance
uniquify
set_fix_multiple_port_nets -all -buffer_constants
set_fix_hold [all_clocks]
set case_analysis_with_logic_constants true
#Synthesis all design
#compile_ultra -timing_high_effort_script
compile -map_effort high
set hdlin_translate_off_skip_text "TRUE"
set edifout_netlist_only "TRUE"
set verilogout_no_tri true

set hdlin_enable_presto_for_vhdl "TRUE"
set sh_enable_line_editing true
set sh_line_editing_mode emacs
history keep 100

alias h history
set bus_inference_style {%s[%d]}
set bus_naming_style {%s[%d]}
set hdlout_internal_busses true
change_names -hierarchy -rule verilog
define_name_rules name_rule -allowed {a-z A-Z 0-9 _} -max_length 255 -type cell
define_name_rules name_rule -allowed {a-z A-Z 0-9 _[]} -max_length 255 -type net
define_name_rules name_rule -map {{"\\*cell\\*" "cell"}}
define_name_rules name_rule -case_insensitive
change_names -hierarchy -rules name_rule

#=============================
report_design > Report/$DESIGN\.design
report_port > Report/$DESIGN\.port
report_net > Report/$DESIGN\.net
report_timing_requirement > Report/$DESIGN\.timing_requiremen
report_constraint > Report/$DESIGN\.constraint
report_timing  > Report/$DESIGN\.timing
report_area  > Report/$DESIGN\.area
report_resource  > Report/$DESIGN\.resource
report_power >  Report/$DESIGN\.power



#=============================
set verilogout_higher_design_first true
write -format verilog -output Netlist/$DESIGN\_syn.v -hierarchy
write_sdf -version 2.1 -context verilog -load_delay cell Netlist/$DESIGN\_syn.sdf
write_sdc Netlist/$DESIGN\_syn.sdc
write_file -format ddc -hierarchy -output Netlist/$DESIGN\_syn.ddc
report_timing



