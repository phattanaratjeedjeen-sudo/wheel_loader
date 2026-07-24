when /load_estimate srv is called, do these step
1. check if acc _g and vel_g have magnitude less than theshold. 
2. if 1 false: response warn that the static conditions arent satisfy. 
2. if 1 true: 
2.1 set pbi and pri equal value of pb_filter and pr_filter and initialize timecounter, store theta_gi  
2.2 begi using median and EMA filter for pb and pr and return as pb_filter, pr_filter 
2.3 calculate pb, pr settle via pressure_estimator function for duration (1sec). this step we get multiple pb, pr settle 
2.4 use median for 2.3 to get single value as pbf, prf
2.5 after get pbf and prf., calculate 
2.5.1 empty bucket offset
2.5.2 geometry
2.5.3 cylinder force
2.5.4 simple_lever
2.5.5 compensator
and return estimated mass 
note that after this srv returns response reset all variables. prepare for next time service call. 
