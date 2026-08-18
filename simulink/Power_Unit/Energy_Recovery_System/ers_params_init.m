% ===== ERS 系统参数定义 =====

% 电池 / 能量相关
Battery_Capacity_J   = Simulink.Parameter(4000000);   % 电池容量，单位 J（对应 -100/4000000 里的4000000）
Battery_Temp_Init_C  = Simulink.Parameter(40);        % 电池初始温度，对应 Constant4
Battery_Temp_HighLim = Simulink.Parameter(65);         % 过热恢复阈值（Stateflow条件里的65）
Battery_Temp_FaultLim= Simulink.Parameter(55);        % 故障清除阈值（Stateflow条件里的55）

% SOC 相关
SOC_Init_Pct         = Simulink.Parameter(50);        % SOC初始值，对应 Energy_Store 里 Integrator 的50
SOC_Max_Pct          = Simulink.Parameter(100);        % SOC上限
SOC_Min_Pct          = Simulink.Parameter(0);          % SOC下限
SOC_Boost_Cutoff     = Simulink.Parameter(5);          % Boost退出阈值（soc<=5）

% MGU-K 功率/扭矩相关
MGUK_Speed_Boost_RPM = Simulink.Parameter(400);       % 对应 Constant1(42) = 400
MGUK_Power_Regen_W   = Simulink.Parameter(250000);    % 对应 Constant2(45) = 250000
MGUK_Speed_Regen_RPM = Simulink.Parameter(400);       % 对应 Constant3(48) = 400
MGUK_Torque_SatLimit = Simulink.Parameter(400);       % 对应 Saturation 上下限
Regen_Sign_Gain      = Simulink.Parameter(-1);        % 对应 Control_Electronics 里 Gain(SID50)=-1，Regen扭矩符号翻转

% 油门相关
Throttle_Boost_Exit  = Simulink.Parameter(75);  % Boost退出油门阈值(滞回下限,原来跟进入阈值共用80)

% 转速/惯量相关
Vehicle_Speed_Init   = Simulink.Parameter(100);       % 对应 Integrator(53) InitialCondition
Inertia_Gain         = Simulink.Parameter(0.5);       % 对应顶层 Gain=0.5
Throttle_Gain         = Simulink.Parameter(0.01);      % 对应 Control_Electronics 里 Gain1=0.01

% 模式扭矩指令 / 输入默认值
Torque_Cmd_Standby_Nm = Simulink.Parameter(0);        % Off_Standby状态扭矩指令，对应 Off_standby(SID24)=0
Torque_Cmd_FailSafe_Nm= Simulink.Parameter(0);        % Fail_Safe状态扭矩指令，对应 Fail_safe(SID27)=0
Clear_Fault_Default  = Simulink.Parameter(0);         % clear_fault输入默认值，对应顶层 Constant5(SID37)=0

% ===== 电池温度动态模型相关=====
Battery_Mass_kg          = Simulink.Parameter(80);     % 电池组等效质量，单位 kg（F1电池组量级）
Battery_SpecificHeat_J   = Simulink.Parameter(1000);   % 比热容，单位 J/(kg·K)，锂电池常用量级
Battery_ThermalMass_JperC= Simulink.Parameter(80000);  % 等效热容 m*c，单位 J/°C（= Mass * SpecificHeat，方便直接用一个Gain）
Battery_HeatLossCoeff    = Simulink.Parameter(0.05);   % 发热系数k：Q_gen = k * |elec_power|，即5%功率转化为热
Battery_ThermalResistance= Simulink.Parameter(0.02);   % 等效热阻 R_thermal，单位 °C/W，越小散热越强（冷却系统能力）

% ===== 车手按键默认参数配置 =====
Boost_Button_Default    = Simulink.Parameter(0);   % Boost按键默认状态: 0-未按下, 1-按下
Recharge_Button_Default = Simulink.Parameter(0);   % Recharge按键默认状态: 0-未按下, 1-按下

% ===== ERS 策略模式（Race/Push/Charge）分档阈值 =====
Strategy_Cmd_Default = Simulink.Parameter(0);   % 0=Race, 1=Push, 2=Charge
Throttle_Boost_Threshold_Race  = Simulink.Parameter(80);
SOC_Boost_Enter_Race           = Simulink.Parameter(7);
MGUK_Power_Boost_W_Race        = Simulink.Parameter(300000);
SOC_Regen_Cutoff_Race          = Simulink.Parameter(95);

Throttle_Boost_Threshold_Push  = Simulink.Parameter(50);
SOC_Boost_Enter_Push           = Simulink.Parameter(3);
MGUK_Power_Boost_W_Push        = Simulink.Parameter(350000);
SOC_Regen_Cutoff_Push          = Simulink.Parameter(85);

Throttle_Boost_Threshold_Charge = Simulink.Parameter(101);
SOC_Boost_Enter_Charge          = Simulink.Parameter(40);
MGUK_Power_Boost_W_Charge       = Simulink.Parameter(0);
SOC_Regen_Cutoff_Charge         = Simulink.Parameter(100);

% ===== 2026 Overtake / Manual Override 简化模型 =====
Overtake_Button_Default      = Simulink.Parameter(0);
Overtake_Enabled_Default     = Simulink.Parameter(1);
Detection_Gap_Valid_Default  = Simulink.Parameter(0);

ERS_K_Max_Power_W            = Simulink.Parameter(350000);  % 350 kW
Overtake_Energy_Bonus_J      = Simulink.Parameter(500000);  % 0.5 MJ

NonOvertake_Taper_Start_KPH  = Simulink.Parameter(290);
Overtake_Full_Power_KPH      = Simulink.Parameter(337);
Overtake_Max_Speed_KPH       = Simulink.Parameter(355);

% ===== 刹车制动力参数（用于 Vehicle_Accel_Calc 里的 F_brake 项） =====
Brake_Force_N = Simulink.Parameter(40000);  % 刹车踩下时的等效纵向制动力, N
                                             % （约等于 798kg * 9.81 * 5g 的量级，可按需调节）

% ===== 单圈能量回收(Harvest)上限管理 (2026新增) =====
% 数值相对 Battery_Capacity_J = 4,000,000 J 设定：
% Race 留出热/寿命余量，Push 单圈冲刺不考虑长期损耗，可以回收更多支撑下一圈部署。
Lap_Recharge_Limit_J_Race   = Simulink.Parameter(1200000);  % 正赛：单圈回收上限 1.2 MJ（约30%电池容量）
Lap_Recharge_Limit_J_Push   = Simulink.Parameter(2000000);  % 排位hotlap：单圈回收上限 2.0 MJ（约50%电池容量，尽量多回收）
Lap_Recharge_Limit_J_Charge = Simulink.Parameter(4000000);  % Charge策略：约等于电池总容量，相当于圈内不设额外限制，只受电池物理容量本身约束

% ===== 单圈能量部署(Deployment)上限管理 (2026新增) =====
% 和 Harvest 侧一样，Push 策略只需撑1-2圈，可以比 Race 更激进地放电；
% Charge 策略本来 MGUK_Power_Boost_W_Charge 就已经是0，这个上限基本只是兜底、不会真正起作用。
Lap_Deploy_Limit_J_Race   = Simulink.Parameter(8500000);   % 正赛：单圈部署上限 8.5 MJ
Lap_Deploy_Limit_J_Push   = Simulink.Parameter(12000000);  % 排位hotlap：单圈部署上限 12 MJ，尽量多放
Lap_Deploy_Limit_J_Charge = Simulink.Parameter(4000000);   % Charge策略：≈电池总容量，等于不额外限制
