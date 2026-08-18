% =========================================================================
% common_params_init.m - 整车公共物理参数与环境定义
% =========================================================================

% ===== 1. 环境与基础物理常数 =====
Ambient_Temp_C       = Simulink.Parameter(25);      % 环境温度 (°C)
Air_Density_kgm3     = Simulink.Parameter(1.225);   % 空气密度 (kg/m^3)
Gravity_ms2          = Simulink.Parameter(9.81);    % 重力加速度 (m/s^2)

% ===== 2. 车辆与底盘动力学参数 =====
Vehicle_Mass_kg      = Simulink.Parameter(798);     % 整车+车手+燃油等效质量 (kg)
Wheel_Radius_m       = Simulink.Parameter(0.33);    % 车轮滚动半径 (m)
Aero_Cd_A_m2         = Simulink.Parameter(1.2);     % 阻力系数 * 迎风面积 (m^2)
Rolling_Resist_Coeff = Simulink.Parameter(0.015);   % 滚动阻力系数
Brake_Force_N        = Simulink.Parameter(40000);   % 踩刹车时的最大纵向制动力 (N)

% ===== 3. 整车策略与输入信号公共默认值 =====
Strategy_Cmd_Default         = Simulink.Parameter(0);  % 0=Race, 1=Push, 2=Charge
Overtake_Button_Default     = Simulink.Parameter(0);  % 0=未按下, 1=按下
Overtake_Enabled_Default    = Simulink.Parameter(1);  % 0=禁用, 1=允许
Detection_Gap_Valid_Default = Simulink.Parameter(0);  % 0=无效, 1=有效

% ===== 4. 动力总成协调器配置 =====
Enable_Bidirectional_Interlock = Simulink.Parameter(false); % 协调器双向互锁开关，默认 false
