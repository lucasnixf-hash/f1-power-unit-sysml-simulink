# F1 ERS（能量回收系统）SysML + Simulink 仿真项目

本项目用 SysML（drawio 绘制的 BDD / IBD / STM）描述了一套 F1 风格的能量回收系统（Energy Recovery System, ERS）架构，并在 Simulink/Stateflow 中做了对应的可执行仿真模型（`ERS_sim.slx`），参数集中定义在 `ers_params_init.m` 中。整体设计参考了 F1 2026 赛季动力单元规则的思路：MGU-K 最大电功率 350 kW、按 Race / Push / Charge 三种策略动态调整能量部署阈值、按圈管理能量回收/部署预算，以及 2026 新增的 Overtake（超车按钮 / Manual Override）逻辑。

## 一、文件清单

| 文件 | 类型 | 内容 |
|---|---|---|
| `ERS_overall_structure_drawio.xml` | SysML BDD | ERS 顶层 Block 分解（组成关系） |
| `ERS_IBD_drawio.xml` | SysML IBD | ERS 顶层部件之间的接口/信号流 |
| `Control_Electronics_IBD_drawio.xml` | SysML IBD | Control Electronics 内部子部件与接口 |
| `ERS_STM_drawio.xml` | SysML STM | ERS 工作模式状态机（Off/Boost/Regen/Fail_Safe + 策略并行状态） |
| `ERS_sim.slx` | Simulink/Stateflow | 可执行仿真模型，与上面四张图基本一一对应 |
| `ers_params_init.m` | MATLAB 脚本 | 所有 `Simulink.Parameter` 定义，运行后进入 base workspace 供模型引用 |

## 二、SysML 架构模型

### 1. BDD — `ERS_overall_structure_drawio.xml`
顶层 Block **Energy Recovery System** 分解为：
- **Kinetic Recovery**（`krs`）→ 进一步分解为 **MGU-K** 和 **Inverter**
- **Control Electronics**（`ce`）→ 内部包含 **ERS Controller**、**Strategy Parameter Selector**、**Overtake Control**、**Lap Deploy Energy Manager**、**Lap Recharge Energy Manager**
- **Energy Store / Battery Management System**（`es`）→ **Li-ion Battery Module** + **Battery SOC Model** + **Battery Thermal Model**
- **Driver Controls**（`dc`）：Boost Button、Recharge Button、Overtake Button
- 枚举类型 `ERS_Strategy`（Race / Push / Charge）作为策略选择的数据类型

每个 Block 都以 SysML `<<block>>` 形式标注了 input / output / values 三个分区，Lap Deploy / Recharge Energy Manager 两个 Block 的 values 区直接写出了三种策略下的能量上限数值，是"参数—架构图—代码"三者对齐的一个好例子。

### 2. IBD — `ERS_IBD_drawio.xml`
展示了 `ce`（Control Electronics）、`dc`（Driver Controls）、`es`（Energy Store）、`krs`（Kinetic Recovery）、`rc`（Race Control）之间的接口，主要流：
- `[Driver_Inputs]`：Boost_Req / Recharge_Req / Overtake_Req（均为 Boolean）+ Clear_Fault_Cmd
- `[BMS_DataBus]`：电池 SOC、温度等状态数据
- `[Elec_Power]`：Kinetic Recovery ↔ Energy Store 之间的电功率交换
- `ERS_Strategy`、`Lap_Trigger`、`Detection_Gap_Valid`、`Overtake_Enabled` 等控制/状态信号

### 3. IBD — `Control_Electronics_IBD_drawio.xml`
把 BDD 里的 "ERS Controller" 进一步展开为 8 个内部部件，并且**与 Simulink 子系统命名完全一致**（这是本项目一个值得保留的优点，架构图和模型可以逐块对照检查）：

| IBD 部件缩写 | 全名 | 作用 |
|---|---|---|
| `sm` | Strategy_Manager | 根据 `strategy_cmd` 选择当前策略对应的一组阈值 |
| `btc` | Boost_Torque_Calc | 由功率上限/转速计算 Boost 扭矩指令 |
| `rtc` | Regen_Torque_Calc | 由回收功率上限/转速计算 Regen 扭矩指令 |
| `epls` | ERS_K_Power_Limit_Selector | 结合超车逻辑给出最终 MGU-K 功率上限 |
| `oc` | Overtake_Control | 超车按钮可用性/激活判定 |
| `ldem` | Lap_Deploy_Energy_Manager | 单圈能量部署预算管理 |
| `lem` | Lap_Recharge_Energy_Manager | 单圈能量回收预算管理 |
| （无缩写） | Torque_Command_Selector | 按 `ers_mode` 选择最终送给 MGU-K 的扭矩指令 |

### 4. STM — `ERS_STM_drawio.xml`
主状态机（对应 Simulink 里的 Stateflow chart，输入含 `brake_pressed`、`clear_fault`、`soc`、`battery_temp` 等）包含四个主状态：

- **Off_Standby**：默认态，`ers_mode = 0`
- **Boost_State**：`ers_mode = 1`，进入条件为 `(boost_button==1 || overtake_active==1) && strategy≠Charge && throttle > Throttle_Boost_Threshold && soc > SOC_Boost_Enter && lap_deploy_limit_reached==0`；退出条件包含油门/SOC 不足或 `lap_deploy_limit_reached==1`。内部又区分 **Standard_Deployment** 与 **Overtake_Deployment** 两种子状态。
- **Regen_State**：`ers_mode = 2`，进入条件为刹车 + Recharge 按键 + `soc < SOC_Regen_Cutoff` + `lap_recharge_limit_reached==0`
- **Fail_Safe**：`ers_mode = 3`，进入条件为 `battery_temp > Battery_Temp_HighLim || soc >= SOC_Max_Pct`，需 `clear_fault==1 && battery_temp < Battery_Temp_FaultLim` 才能清除

并行区域 **策略状态机**：根据 `strategy_cmd`（0/1/2）在 entry action 中把 `current_strategy` 置为 Race/Push/Charge，三种策略对应的四个阈值（`Throttle_Boost_Threshold`、`SOC_Boost_Enter`、`MGUK_Power_Boost_W`、`SOC_Regen_Cutoff`）：

| 策略 | Throttle_Boost_Threshold | SOC_Boost_Enter | MGUK_Power_Boost_W | SOC_Regen_Cutoff |
|---|---|---|---|---|
| Race | 80 | 7 | 300,000 | 95 |
| Push | 50 | 3 | 350,000 | 85 |
| Charge | 101（相当于禁用 Boost） | 40 | 0 | 100 |

图上还专门记录了一个设计决策备忘："超车 Bonus 锁存"：`overtake_active` 只要在本圈出现过一次，`Lap_Deploy_Energy_Manager` 内部的锁存器就会保持"已获得 Bonus"状态直到下一次 `lap_trigger`，避免超车窗口结束瞬间部署上限跳变——这个逻辑特意放在 STM 之外（Lap_Deploy_Energy_Manager 内部），不影响 STM 本身的 guard。这是一个值得在文档里保留的细节，因为它体现了"状态机保持简单、边界效应下放到数据流模块处理"的建模取舍。

## 三、Simulink 实现（`ERS_sim.slx`）

### 顶层结构
四个主子系统，与 BDD 完全对应：`Control_Electronics`、`Energy_Store`、`Kinetic_Recovery`、`Vehicle_Longitudinal_Dynamics`；此外还有一组用于场景构造的测试信号（`throttle_test`、`boost_button_test`、`recharge_button_test`、`overtake_button_test`、`overtake_enabled_test`、`detection_gap_valid_test`、`brake_pressed_test`、`strategy_cmd_test`）、一个 `DiscretePulseGenerator` 产生 `lap_trigger`（周期性圈信号）、`Clear_Fault` 常量输入，以及一个 `Scope` 用于查看结果。

### Control_Electronics 内部
与 IBD 一一对应的 8 个子系统：`Strategy_Manager`、`Boost_Torque_Calc`、`Regen_Torque_Calc`、`ERS_K_Power_Limit_Selector`、`Overtake_Control`、`Lap_Deploy_Energy_Manager`、`Lap_Recharge_Energy_Manager`、`Torque_Command_Selector`，核心还包含主状态机 `ERS_STM`（Stateflow chart）。

- **Strategy_Manager**：12 个 `Constant`（3 策略 × 4 阈值）通过 3 个 `MultiPortSwitch` 按 `strategy_cmd` 选择，内部还有一个小型 Stateflow chart（`Strategy_SM`）把 `strategy_cmd` 映射为 `current_strategy` 枚举。
- **Overtake_Control**：内部 `Overtake_Logic` 为一个 Stateflow chart，输入 `overtake_button`、`overtake_enabled`、`detection_gap_valid`、`car_speed_kph`、`soc`、`battery_temp`、`current_strategy`，输出 `overtake_available`、`overtake_active`、`ERS_K_Power_Limit_W`。
- **Lap_Recharge_Energy_Manager**：把 `torque_fb × speed_fb`（经符号翻转）积分为 `lap_recharge_energy_j`，按 `current_strategy` 选择单圈回收上限（Race 1.2 MJ / Push 2.0 MJ / Charge 4.0 MJ），并用一个 taper（斜率渐减而非硬截断）子系统在接近上限时逐步收紧 `regen_power_limit_w`，`lap_trigger` 到来时清零重新计圈。
- **Lap_Deploy_Energy_Manager**：结构对称，管理 `lap_deploy_energy_j` 与部署上限（Race 8.5 MJ / Push 12 MJ / Charge 4 MJ），并包含前面提到的 Overtake Bonus 锁存逻辑（`Overtake_Bonus_Latch` + `Overtake_Bonus_Switch`，命中后给上限加上 `Overtake_Energy_Bonus_J = 500,000 J`，直到下一次 `lap_trigger` 才清零）。
- **Boost_Torque_Calc / Regen_Torque_Calc**：由功率上限与转速反馈算出扭矩指令（`P/ω` 型计算 + `MinMax` 限幅）。
- **Torque_Command_Selector**：按 `ers_mode`（来自 ERS_STM）用 `MultiPortSwitch` 选择最终扭矩指令，再经 `Saturate` 限制在 `MGUK_Torque_SatLimit` 内送给 MGU-K。

### Energy_Store 内部
- **Battery_SOC**：一个简单的电量积分器（`Gain` + `Integrator`，初值 `SOC_Init_Pct = 50`），把电功率转换为 SOC 变化。
- **Battery_Thermal**：一阶热模型，发热项 `Q_gen = HeatLossCoeff × |P|`（`Abs` + `Gain`），散热项按 `Battery_ThermalResistance` 与环境温度差计算（`Sum` 组合），积分得到 `battery_temp`（初值 `Battery_Temp_Init_C = 40`）。

### Kinetic_Recovery 内部
`Mech_Power_In → Power_calc（Product）→ Gain（Inertia_Gain = 0.5）→ 积分器（初值 100）→ Veh_Speed_FB / Elec_Power_Out`。

> ⚠️ **需要注意的一点**：`ers_params_init.m` 里明确写了注释——这个积分器实际算的是 **MGU-K 转速反馈**（`mgu_k_speed`），命名成 `Vehicle_Speed` 只是历史遗留，并不是车辆真实地面速度。真实车速由下面单独的 `Vehicle_Longitudinal_Dynamics` 子系统计算。

### Vehicle_Longitudinal_Dynamics 内部
输入 `torque_cmd_Nm`（来自 MGU-K）、`v_kph_fb`、`brake_pressed`；内部 `Vehicle_Accel_Calc`（Stateflow chart）结合整车质量、气动阻力、滚动阻力、刹车制动力（`Vehicle_Mass_kg`、`Aero_Cd_A_m2`、`Air_Density_kgm3`、`Rolling_Resist_Coeff`、`Wheel_Radius_m`、`Gravity_ms2`、`Brake_Force_N`）算出加速度，再积分得到 `car_speed_kph`，闭环反馈回 `Control_Electronics`（供 `Car_Speed_In`）和 `Overtake_Control`（超车速度窗口 290–355 kph 的判断/渐减）以及 `ERS_STM` 的退出条件（`car_speed_kph ≥ 355` 时强制退出 Boost）。

## 四、参数文件 `ers_params_init.m`

按功能域集中定义了全部 `Simulink.Parameter`：电池/SOC、MGU-K 功率转速、油门阈值、整车纵向动力学、电池热模型、驾驶员按键默认值、三种策略分档阈值、2026 Overtake/Manual Override 参数、单圈能量部署/回收上限。**运行仿真前必须先执行这个脚本**，把参数注入 base workspace，模型才能正确解析这些 `Simulink.Parameter` 引用。

## 五、如何运行仿真

1. 在 MATLAB 中 `cd` 到本项目目录；
2. 运行 `ers_params_init.m` 初始化所有参数；
3. 打开 `ERS_sim.slx`；
4. 通过顶层的 `*_test` 信号源（油门、Boost/Recharge/Overtake 按键、策略指令、刹车、检测有效性等）构造想要的驾驶场景；
5. 运行仿真，用 `Scope` 或添加的信号查看 `mgu_k_torque_cmd`、`ers_mode`、`soc`、`battery_temp`、`lap_deploy_energy_j`、`lap_recharge_energy_j`、`car_speed_kph` 等关键信号；
6. 可以对照四张 SysML 图逐块检查 Stateflow / 子系统内部实现是否与架构设计一致——目前两者吻合度很高，是后续迭代（包括扩展 ICE 部分）时应该保持的好习惯。

## 六、已知简化 / 局限性（后续可以改进的地方）

- `Kinetic_Recovery` 内部积分器输出本质是 MGU-K 转速反馈，与真实车速是两条独立的路径，两者之间目前没有通过传动比等物理关系严格耦合；
- 目前只有 MGU-K 一路扭矩，没有内燃机（ICE）参与整车驱动力计算（见配套的第二份文档）；
- 电池模型是线性 SOC + 一阶 RC 热模型，未考虑内阻压降、OCV-SOC 曲线、放电倍率限制、老化等；
- 整车纵向动力学是简化的纵向单质量模型，没有完整的赛道坡度/弯道剖面；
- Control Electronics 没有建模传感器噪声、CAN 总线延迟、单点故障注入等更贴近真实 ECU 行为的内容；
- 多个并行 Stateflow 区域（策略状态机 / 主状态机 / Overtake 逻辑）之间通过共享变量交互，规模变大后建议引入形式化一致性检查（如 Stateflow 自带的图表检查或 Simulink Design Verifier）。
