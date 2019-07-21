create_clock -name clk_i -period 100 [get_nets clk_i]
#set_false_path -through hera_wrap/hdcp20_trng_u0/u_rng/sync_ringosc_data
#set_false_path -through hera_wrap/hdcp20_trng_u0/u_rng/sync_ringosc_rdy
#set_false_path -through [find / -net hera_wrap/hdcp20_trng_u0/u_rng/ringosc_rdy*]
#set_false_path -through [find / -net hera_wrap/hdcp20_trng_u0/u_rng/ringosc_data*]