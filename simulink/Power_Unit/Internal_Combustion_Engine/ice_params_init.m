% =========================================================================
% ice_params_init.m - ICE 发动机与变速箱专属参数
% =========================================================================

% ===== ICE / Turbocharger / Fuel =====
ICE_Max_Torque_Nm        = Simulink.Parameter(320);
ICE_Max_Power_W          = Simulink.Parameter(400000);
Turbo_Lag_TimeConstant_s = Simulink.Parameter(0.15);
Fuel_Flow_Max_kg_s       = Simulink.Parameter(0.028);
Race_Fuel_Limit_kg       = Simulink.Parameter(110);
Engine_ThermalMass_JperC = Simulink.Parameter(50000);
Engine_HeatLossCoeff     = Simulink.Parameter(0.1);
BSFC_kg_per_kWh          = Simulink.Parameter(0.24);
Engine_Temp_Init_C       = Simulink.Parameter(40);
Fuel_LHV_J_per_kg        = Simulink.Parameter(44e6);

% ===== 状态机与保护阈值 =====
Idle_RPM_Threshold            = Simulink.Parameter(4000);
Fuel_Cut_RPM_Threshold        = Simulink.Parameter(5000);
Fuel_Cut_Throttle_Threshold   = Simulink.Parameter(5);
Overheat_Threshold_C          = Simulink.Parameter(105);
Critical_Overheat_Threshold_C = Simulink.Parameter(135);
Max_Safe_Boost_kPa            = Simulink.Parameter(250);
Turbo_Overheat_Derate_Slope   = Simulink.Parameter(10);

% ===== 传动系统 =====
Final_Drive_Ratio    = Simulink.Parameter(3.5);
Gear_Ratios          = Simulink.Parameter([2.9 2.4 2.0 1.7 1.45 1.25 1.1 1.0]);
Driveline_Efficiency = Simulink.Parameter(0.95);
Shift_Up_RPM         = Simulink.Parameter(14500);
Shift_Down_RPM       = Simulink.Parameter(9000);

% ===== 涡轮 PID 与前馈控制 =====
Turbo_FeedForward_Gain_Kff         = Simulink.Parameter(1.0);
Turbo_FeedForward_Gain_Kff_Boosted = Simulink.Parameter(1.2);
Turbo_PID_Kp                       = Simulink.Parameter(0.5);
Turbo_PID_Ki                       = Simulink.Parameter(0.1);

% ===== 细化控制逻辑参数 =====
Throttle_Actuator_TimeConstant_s = Simulink.Parameter(0.05);
Heat_Fraction_Coeff              = Simulink.Parameter(0.3);
Fuel_Mode_Multiplier_Vec         = Simulink.Parameter([1.0 0.75 0.55]);
