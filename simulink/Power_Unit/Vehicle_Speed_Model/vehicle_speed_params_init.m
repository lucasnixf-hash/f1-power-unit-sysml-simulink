% =========================================================================
% vehicle_speed_params_init.m
% 独立车速计算模型 Vehicle_Speed_sim.slx 专用参数初始化脚本
%
% 说明：
%   本文件从原 common_params_init.m / ers_params_init.m 中，抽取出
%   "车辆纵向动力学（车速计算）" 环节实际用到的参数，汇总成一份独立文件，
%   使新模型 Vehicle_Speed_sim.slx 不再依赖 ERS 的参数文件即可单独打开/仿真。
%
%   建议用法：
%   在 Vehicle_Speed_sim.slx 的
%   Modeling -> Model Settings -> Model Properties -> Callbacks -> InitFcn
%   中填入：
%       vehicle_speed_params_init
%   这样每次打开/仿真该模型时会自动加载以下参数到 base workspace。
%
%   注意：
%   - 下面参数的数值目前与 common_params_init.m / ers_params_init.m 中保持一致，
%     后续如果整车质量、气动、轮胎等参数变化，请【只在本文件中修改一处】，
%     避免多个模型的参数文件互相不同步。
%   - 如果你希望三个模型（ERS / ICE / Vehicle_Speed）继续共用一份参数，
%     也可以不新建本文件，而是把 Vehicle_Speed_Init 这一行从
%     ers_params_init.m 挪到 common_params_init.m 里，
%     然后 Vehicle_Speed_sim.slx 的 InitFcn 改为调用 common_params_init。
%     两种方式二选一即可，本文件给出的是"完全独立"的版本。
% =========================================================================

% ===== 1. 环境与基础物理常数 =====
Air_Density_kgm3     = Simulink.Parameter(1.225);   % 空气密度 (kg/m^3)
Gravity_ms2          = Simulink.Parameter(9.81);    % 重力加速度 (m/s^2)

% ===== 2. 车辆与底盘动力学参数 =====
Vehicle_Mass_kg      = Simulink.Parameter(798);     % 整车+车手+燃油等效质量 (kg)
Wheel_Radius_m       = Simulink.Parameter(0.33);    % 车轮滚动半径 (m)
Aero_Cd_A_m2         = Simulink.Parameter(1.2);     % 阻力系数 * 迎风面积 (m^2)
Rolling_Resist_Coeff = Simulink.Parameter(0.015);   % 滚动阻力系数
Brake_Force_N        = Simulink.Parameter(40000);   % 踩刹车时的最大纵向制动力 (N)

% ===== 3. 积分器初始状态 =====
% 原定义于 ers_params_init.m 中的 Vehicle_Speed_Init，
% 本质上是"车辆初始车速"，属于车辆纵向动力学模型的状态初值，
% 现随速度计算逻辑一并迁移到本文件中。
Vehicle_Speed_Init   = Simulink.Parameter(100);     % 车速积分器初始值 (km/h)
