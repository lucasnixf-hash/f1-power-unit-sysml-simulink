# ICE_sim.slx — 内燃机（ICE）动力单元模型说明

> 本文档基于对 `ICE_sim.slx` 内部 XML（blockdiagram / systems / stateflow）结构的静态解析生成，
> 未在 MATLAB 中实际打开/运行模型。查表（Lookup Table）中的具体数值断点未导出，
> 如需精确标定值请在 Simulink 中直接双击对应模块查看。

## 1. 模型定位

`ICE_sim.slx` 是赛车动力单元（Power Unit）中**内燃机子系统**的独立可仿真模型，
可单独打开调试，也作为 Model Reference 被顶层 `Power_Unit_sim.slx` 中的 `ICE` 模块引用。

依赖的外部文件：

| 文件 | 作用 |
|---|---|
| `common_params_init.m` | 整车公共参数（质量、气动、环境常数等），在模型 `InitFcn` 中调用 |
| `ice_params_init.m` | ICE 专属参数（扭矩/功率上限、涡轮、燃油、传动、状态机阈值等），在模型 `InitFcn` 中调用 |
| `ICE_State.m` | `Simulink.IntEnumType` 枚举，定义 ICE 状态机的 7 个状态 |
| `ICE_Fuel_Mode.m` | `Simulink.IntEnumType` 枚举，定义 3 种燃油模式 |

模型 InitFcn：
```matlab
common_params_init;
ice_params_init;
```

求解器：`VariableStepAuto`（ode3）。**注意**：这是仿真用的变步长求解器，若要做代码生成，
必须先切换为**定步长离散求解器**（见文末"代码生成前置改动"）。

## 2. 顶层输入 / 输出接口

### 输入（Inport）

| 信号名 | 说明 |
|---|---|
| `overtake_button` | 超车按钮 |
| `overtake_enabled` | 超车功能使能 |
| `detection_gap_valid` | 前车雷达/间距检测有效标志 |
| `ignition_on` | 点火开关 |
| `ice_throttle_cmd_pct` | 节气门指令（%） |
| `ice_clear_fault` | 故障清除指令 |
| `ice_strategy_cmd` | 整车策略指令（0=Race, 1=Push, 2=Charge, 3=Lift&Coast） |
| `ers_in_failsafe` | ERS 是否处于 Fail-Safe（用于 ICE 状态机的联锁保护） |
| `car_speed_kph` | 车速反馈（用于变速箱换挡逻辑） |

### 输出（Outport）

| 信号名 | 说明 |
|---|---|
| `ice_fail_safe` | ICE 是否处于失效保护状态 |
| `ice_torque_out_Nm` | ICE 输出到传动系统的扭矩（Nm） |

## 3. 顶层架构

```
overtake_* ──────────────► ICE_Overtake_Mapper ──► turbo_aggressive_mode ──┐
ice_strategy_cmd ────────► ICE_Strategy_Mapper ───► ice_fuel_mode ─────────┤
ignition_on, throttle_cmd,                                                ▼
car_speed_kph, clear_fault, ─────────────────► Internal_Combustion_Engine ──► ice_torque_out_Nm
ers_in_failsafe                                                          └──► ice_fail_safe
```

### 3.1 `ICE_Overtake_Mapper`（子系统）
内部为 MATLAB Function `Overtake_Aggressiveness_Logic`：
根据超车按钮、超车使能、间距有效性，并结合当前冷却液温度/增压压力是否留有安全裕度
（阈值取 `Overheat_Threshold_C`、`Max_Safe_Boost_kPa` 的 95%），
综合判定是否允许开启**涡轮激进模式** `turbo_aggressive_mode`。

### 3.2 `ICE_Strategy_Mapper`（子系统）
内部为 MATLAB Function `Strategy_To_Fuel_Mode`，将整车策略指令映射为燃油模式：

| `ice_strategy_cmd` | 含义 | 映射结果 `ice_fuel_mode` |
|---|---|---|
| 0 | Race（标准正赛） | `Normal` |
| 1 | Push（排位/激进） | `Normal` |
| 2 | Charge（巡航充能/保油） | `Fuel_Saving` |
| 3 | Lift & Coast（抬油门滑行） | `Lift_and_Coast` |
| 其他 | — | `Normal`（默认兜底） |

### 3.3 `Internal_Combustion_Engine`（主体子系统）
包含 7 个子模块，信号相互耦合形成闭环（转速反馈进节流阀限制、变速箱、涡轮等）：

| 子模块 | 功能 |
|---|---|
| `Throttle_Actuator` | 节气门执行器：对指令做限幅（`MinMax` 与状态机给出的 `throttle_limit_pct` 取小），再经一阶传递函数（`Throttle_Actuator_TimeConstant_s`）模拟执行器动态，最终饱和输出 `throttle_actual_pct` |
| `Turbocharger` | 增压压力闭环控制：前馈增益（正常/激进两档，由 `turbo_aggressive_mode` 切换）+ PID，经涡轮迟滞一阶惯性环节（`Turbo_Lag_TimeConstant_s`）输出 `boost_pressure_kPa`；当冷却液温度超过 `Overheat_Threshold_C` 时按 `Turbo_Overheat_Derate_Slope` 对增压上限降额，并整体饱和于 `Max_Safe_Boost_kPa` |
| `Combustion_Torque_Model` | 由节气门/转速查表（`Combustion_Torque_LUT`）得到基础扭矩，叠加增压带来的扭矩增益，再经 MATLAB Function `Torque_Power_Limiter` 做扭矩上限（`ICE_Max_Torque_Nm`）与功率上限（`ICE_Max_Power_W`）双重限制，得到 `total_crank_torque_Nm` |
| `Fuel_System` | 按 BSFC（`BSFC_kg_per_kWh`）与当前功率（`Power_Calc` 子系统：扭矩×角速度）估算燃油流量，乘以燃油模式对应的节油系数（`Fuel_Mode_Multiplier_LUT`：Normal/Lift&Coast/Fuel_Saving 三档），经 `Fuel_Cut_Switch` 在断油条件下强制置零，饱和于 `Fuel_Flow_Max_kg_s`，并积分得到累计耗油量，与 `Race_Fuel_Limit_kg` 比较输出 `fuel_low_warning` |
| `Engine_Thermal_Model` | 一阶热容模型：燃油放热（`Fuel_LHV_J_per_kg` × 燃油流量 × `Heat_Fraction_Coeff`）减去对环境散热（`Engine_HeatLossCoeff` × 温差），除以热容 `Engine_ThermalMass_JperC` 积分得到 `coolant_temp_C`（初值 `Engine_Temp_Init_C`） |
| `Gearbox` | 由车速换算轮速，结合当前档位速比（`Gear_Ratio_LUT` × `Final_Drive_Ratio`）反算发动机转速 `engine_speed_rpm`（经一步延迟 `UnitDelay` 反馈用于换挡判断，避免代数环），扭矩经 `Driveline_Efficiency` 折损后输出到轮端；换挡逻辑见下 |
| `ICE_STM` | 发动机主状态机，见第 4 节 |

其中 `Gearbox/Gear_Shift_Logic`（MATLAB Function，含 `persistent` 档位变量）为**滞回换挡逻辑**：

```matlab
if engine_speed_rpm_prev >= Shift_Up_RPM && gear < 8
    gear = gear + 1;
elseif engine_speed_rpm_prev <= Shift_Down_RPM && gear > 1
    gear = gear - 1;
end
```
即转速上探 `Shift_Up_RPM`（14500）升档、下探 `Shift_Down_RPM`（9000）降档，1~8 档限幅，
使用**上一步**转速避免代数环。

## 4. ICE 状态机（`ICE_STM`，Stateflow）

7 个状态（对应 `ICE_State` 枚举）：

| 状态 | `ice_throttle_limit_pct` | `fuel_cut_enable` | 说明 |
|---|---|---|---|
| `Off_Standby` | 0 | false | 默认初始状态，点火未开启 |
| `Cranking` | 30 | false | 点火开启后的起动阶段（限制节气门至 30%） |
| `Idle` | 100 | false | 怠速（低转速+低油门） |
| `Running` | 100 | false | 正常运行 |
| `Fuel_Cut` | 100 | **true** | 高转速+低油门时断油（滑行减速） |
| `Limp_Home` | 50 | false | 过热保护（限功率跛行） |
| `Fail_Safe` | 0 | **true** | 严重过热 / 增压超限 / ERS 失效联锁触发的完全失效保护 |

主要跳转条件（节选）：
- `Off_Standby → Cranking`：`ignition_on == true`
- `Cranking → Idle`：`engine_speed_rpm > Idle_RPM_Threshold * 0.3`
- `Idle ↔ Running`：由油门/转速是否超过 `Fuel_Cut_Throttle_Threshold` / `Idle_RPM_Threshold` 决定
- `Running → Fuel_Cut`：`engine_speed_rpm > Fuel_Cut_RPM_Threshold && throttle_actual_pct < Fuel_Cut_Throttle_Threshold`
- `Fuel_Cut → Running`：`throttle_actual_pct > Fuel_Cut_Throttle_Threshold`
- `Active_Modes → Limp_Home`：`coolant_temp_C > Overheat_Threshold_C`
- `Limp_Home → Active_Modes`：`coolant_temp_C < Overheat_Threshold_C - 5`（回滞 5°C 防抖）
- `→ Fail_Safe`（任意状态）：`coolant_temp_C > Critical_Overheat_Threshold_C || boost_pressure_kPa > Max_Safe_Boost_kPa || ers_in_failsafe == true`
- `Fail_Safe → Off_Standby`：`ice_clear_fault == true`

## 5. 关键参数一览（详见 `ice_params_init.m`）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `ICE_Max_Torque_Nm` | 320 | 扭矩上限 |
| `ICE_Max_Power_W` | 400000 | 功率上限 |
| `Turbo_Lag_TimeConstant_s` | 0.15 | 涡轮迟滞时间常数 |
| `Fuel_Flow_Max_kg_s` | 0.028 | 最大燃油流量 |
| `Race_Fuel_Limit_kg` | 110 | 比赛燃油配额 |
| `BSFC_kg_per_kWh` | 0.24 | 燃油消耗率 |
| `Overheat_Threshold_C` / `Critical_Overheat_Threshold_C` | 105 / 135 | 过热/严重过热阈值 |
| `Max_Safe_Boost_kPa` | 250 | 增压安全上限 |
| `Gear_Ratios` | [2.9…1.0]（8 档） | 各档速比 |
| `Shift_Up_RPM` / `Shift_Down_RPM` | 14500 / 9000 | 换挡转速阈值 |

## 6. 代码生成前置改动（后续做 C/C++ 代码生成时需注意）

1. **求解器**：`Configuration Parameters → Solver` 改为 `Fixed-step`（如 `discrete (no continuous states)` 或 `ode4`，需先确认模型内 `Integrator`/`TransferFcn` 等连续状态块是否允许离散化或改用离散等效模块）。
2. **代数环**：`Gearbox` 中转速反馈已用 `UnitDelay` 断开，其余回路（涡轮 PID）需确认无新增代数环。
3. **数据类型**：当前多为 `double`，嵌入式目标通常需要定点或 `single` 类型，需结合 Fixed-Point Designer 评估。
4. **Simulink.Parameter**：当前参数以 `Simulink.Parameter` 定义在 base workspace，量产代码生成建议改为 `Simulink.Parameter` + 显式 `StorageClass`（如 `ExportedGlobal`）或迁移到数据字典，便于标定。
5. 建议先用 **Simulink Coder** 生成代码跑通闭环再切换 **Embedded Coder** 做嵌入式优化（详见聊天正文的学习路线）。
