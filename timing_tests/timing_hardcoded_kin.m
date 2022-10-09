N_trials = 100;

runtimes_ur5 = run_timing_test(@IK_ur5_setup.setup, {@IK_ur5_setup.run_mex}, N_trials, 100)
%%
runtimes_IRB_6640 = run_timing_test(@hardcoded_IK_setups.IRB_6640.setup, {@hardcoded_IK_setups.IRB_6640.run_mex}, N_trials, 100)
%%
runtimes_three_parallel_bot = run_timing_test(@hardcoded_IK_setups.three_parallel_bot.setup, {@hardcoded_IK_setups.three_parallel_bot.run_mex}, N_trials, 80)