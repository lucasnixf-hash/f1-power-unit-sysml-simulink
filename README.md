# f1-power-unit-sysml-simulink: F1 2026 动力单元（Power Unit, PU）SysML 架构设计与 Simulink/Stateflow 仿真系统

本项目提供了一套符合 F1 2026 赛季动力单元规则的完整 Model-Based Systems Engineering (MBSE) 仿真与架构设计体系。涵盖了基于 SysML（BDD / IBD / STM）的系统顶层与子系统架构建模，以及在 Simulink / Stateflow 中构建的可执行闭环仿真模型系统。

系统集成了 **能量回收系统 (ERS)**、**内燃机系统 (ICE)**、**动力单元协调控制器 (Power Unit Coordinator)**、**扭矩融合系统 (Powertrain Torque Fusion)** 以及 **车辆纵向动力学模型 (Vehicle Longitudinal Dynamics)**，并配备了闭环自测试台架（Test Bench）。

---

## 目录

1. [项目概述与设计理念](#一项目概述与设计理念)
2. [仓库文件结构与文件清单](#二仓库文件结构与文件清单)
3. [整体架构与信号流](#三整体架构与信号流)
4. [核心子系统详解](#四核心子系统详解)
   - [4.1 顶层集成与协调系统 (`Power_Unit_sim.slx`)](#41-顶层集成与协调系统-power_unit_simslx)
   - [4.2 能量回收系统 (`ERS_sim.slx` 与 SysML 架构)](#42-能量回收系统-ers_simslx-与-sysml-架构)
   - [4.3 内燃机系统 (`ICE_sim.slx`)](#43-内燃机系统-ice_simslx)
   - [4.4 车辆纵向动力学系统 (`Vehicle_Speed_sim.slx`)](#44-车辆纵向动力学系统-vehicle_speed_simslx)
5. [策略档位与参数配置汇总](#五策略档位与参数配置汇总)
6. [快速入门与仿真运行指南](#六快速入门与仿真运行指南)
7. [已知简化与后续改进方向](#七已知简化与后续改进方向)
8. [嵌入式代码生成指南](#八嵌入式代码生成指南)

---

## 一、项目概述与设计理念

F1 2026 赛季动力单元规则对电驱动与内燃机的分配做出了重大调整（MGU-K 电功率提升至 350 kW，同时取消 MGU-H，内燃机功率调整至约 400 kW），并引入了全新的超车/手动覆盖模式（Overtake / Manual Override）。

本项目采用“架构驱动开发”（Architecture-Driven Development）理念：
1. **SysML 规范先行**：使用 Enterprise Architect / Draw.io 绘制 Block Definition Diagram (BDD)、Internal Block Diagram (IBD) 以及 State Machine Diagram (STM)，明确模块边界、端口信号与工作模式。
2. **Simulink/Stateflow 逐块映射**：Simulink 中的子系统命名、端口定义与 Stateflow 状态机完全与 SysML 图纸保持一一对应，实现了“架构—模型—代码”的高度一致性与可追溯性。
3. **闭环系统集成与联调**：通过 Model Reference 机制将 ERS、ICE 与 Vehicle Dynamics 模块引入顶层集成模型，结合协调控制器实现故障互锁（Fault Interlock）与多源扭矩融合。

---

## 二、仓库文件结构与文件清单

### 1. 架构设计图纸（SysML）
| 文件 | 类型 | 说明 |
|---|---|---|
| `ERS_overall_structure_drawio.xml` | SysML BDD | ERS 顶层 Block 结构分解（组成关系与端口定义） |
| `ERS_IBD_drawio.xml` | SysML IBD | ERS 顶层部件间接口与数据总线流向 |
| `Control_Electronics_IBD_drawio.xml` | SysML IBD | ERS 控制电子单元（Control Electronics）内部 8 个子模块分解与信号连接 |
| `ERS_STM_drawio.xml` | SysML STM | ERS 工作模式主状态机与并行策略状态机设计 |

### 2. Simulink 仿真模型
| 文件 | 模块定位 | 说明 |
|---|---|---|
| `Power_Unit_sim.slx` | 顶层集成模型 | 动力单元整体联调与闭环测试台架（包含协调器、扭矩融合与测试激励源） |
| `ERS_sim.slx` | 子系统模型 | 能量回收系统（MGU-K、电池 SOC/热模型、ERS 控制器、单圈预算管理器） |
| `ICE_sim.slx` | 子系统模型 | 内燃机系统（节气门、涡轮增压器、燃油耗量、发动机热模型、8速档位箱） |
| `Vehicle_Speed_sim.slx` | 子系统模型 | 车辆纵向动力学模型（阻力、质量、刹车力及车速解算） |

### 3. MATLAB 参数脚本与数据文件
| 文件 | 类型 | 说明 |
|---|---|---|
| `common_params_init.m` | MATLAB 脚本 | 整车公共参数定义（质量、气动阻力系数、环境常数等） |
| `ers_params_init.m` | MATLAB 脚本 | ERS 专属参数定义（MGU-K 功率/扭矩上限、电池 SOC/热参数、单圈能量预算等） |
| `ice_params_init.m` | MATLAB 脚本 | ICE 专属参数定义（扭矩/功率上限、涡轮参数、燃油耗率、速比、换挡阈值等） |
| `ICE_State.m` | 枚举类 (`IntEnumType`) | 定义 ICE 状态机 7 个工作状态 (`Off_Standby`, `Cranking`, `Idle`, `Running`, `Fuel_Cut`, `Limp_Home`, `Fail_Safe`) |
| `ICE_Fuel_Mode.m` | 枚举类 (`IntEnumType`) | 定义 ICE 3 种喷油模式 (`Normal`, `Fuel_Saving`, `Lift_and_Coast`) |
| `unified_power_unit_test.mat` | MAT 数据集 | 闭环测试台架 `test` 模块引用的多路驱动激励数据 |

---

## 三、整体架构与信号流

顶层集成模型 `Power_Unit_sim.slx` 采用 Model Reference 架构，信号流拓扑如下：

```
                             ┌────────────────────────┐
                 test ──────►│ 闭环测试激励源(13路)    │
                             └───────────┬────────────┘
                                         │
        ┌────────────────────────────────┼────────────────────────────────┐
        ▼                                ▼                                ▼
  ┌──────────┐                     ┌──────────┐                     ┌───────────┐
  │   ERS    │                     │   ICE    │◄─── car_speed ──────│ (反馈)    │
  │(ModelRef)│                     │(ModelRef)│                     └───────────┘
  └────┬─────┘                     └────┬─────┘
       │ ers_mode                       │ ice_fail_safe
       ▼                                ▼
  ┌───────────────────────────────────────────┐
  │ Power_Unit_Coordinator (故障联锁协调控制器)  │
  └─────────────┬─────────────┬───────────────┘
                │ ice_enable  │ mguk_enable   │ ers_in_failsafe
                │             │               └─────────────────► 反馈至 ERS/ICE
                ▼             ▼
  ┌───────────────────────────────────────────┐
  │ Powertrain_Torque_Fusion (扭矩融合系统)    │ ◄── ers_torque_Nm
  └─────────────────────┬─────────────────────┘ ◄── ice_torque_Nm
                        │ torque_cmd_Nm
                        ▼
             ┌─────────────────────┐
             │ Vehicle_Speed_sim   │──► car_speed_kph ──► 闭环反馈至 ERS / ICE
             └─────────────────────┘
```

---

## 四、核心子系统详解

### 4.1 顶层集成与协调系统 (`Power_Unit_sim.slx`)

1. **Power_Unit_Coordinator（故障联锁协调器）**
   - **输入**：`ers_mode`、`ice_fail_safe`、配置参数 `Enable_Bidirectional_Interlock`（默认 `false`）。
   - **逻辑**：当 ERS 进入 `Fail_Safe`（模式 3）时，触发 `ers_in_failsafe`，系统将自动切断内燃机使能（`ice_enable = false`）。若启用双向互锁，则 ICE 故障时亦会反向切断 MGU-K 使能（`mguk_enable = false`）。
2. **Powertrain_Torque_Fusion（扭矩融合）**
   - **逻辑**：采用两路 `Switch` 门控模块，根据 `mguk_enable` 与 `ice_enable` 状态进行门控。当某一动力源被切断时，其扭矩直接清零，其余正常动力源通过 `Sum` 叠加生成最终的 `torque_cmd_Nm` 下发至整车模型。
3. **闭环自测试台 (`test`)**
   - 包含 13 路驱动信号输出，配合 `unified_power_unit_test.mat`，覆盖油门、刹车、策略指令、超车按键及检测间距等典型驾驶工况。

### 4.2 能量回收系统 (`ERS_sim.slx` 与 SysML 架构)

根据 `Control_Electronics_IBD_drawio.xml` 架构，ERS 控制电子单元细化为 8 个子模块：

| IBD 模块标识 | Simulink 子系统 | 核心功能 |
|---|---|---|
| `sm` | `Strategy_Manager` | 根据策略指令切换对应的一组阈值，并通过 Stateflow 映射为 `current_strategy` 枚举 |
| `btc` | `Boost_Torque_Calc` | 计算 MGU-K 放电/助力扭矩指令：$T = \min(P_{\text{boost}} / \omega, T_{\text{sat}})$ |
| `rtc` | `Regen_Torque_Calc` | 计算 MGU-K 能量回收扭矩指令：$T = \min(P_{\text{regen}} / \omega, T_{\text{sat}})$ |
| `epls` | `ERS_K_Power_Limit_Selector` | 结合策略与 Overtake 状态给出最终 MGU-K 功率上限 |
| `oc` | `Overtake_Control` | 判定超车按钮可用性及激活状态（车速窗口 290–355 kph，SOC 与温度校验） |
| `ldem` | `Lap_Deploy_Energy_Manager` | 单圈放电预算积分管理（含 Overtake Bonus 锁存器，增加 0.5 MJ 单圈额度） |
| `lem` | `Lap_Recharge_Energy_Manager` | 单圈回收预算积分管理，结合渐进平滑（Taper）算法防止功率硬截断 |
| — | `Torque_Command_Selector` | 根据主状态机输出的 `ers_mode` 选择最终扭矩指令 |

**ERS 工作状态机 (`ERS_STM`)**：
- **Off_Standby** ($ers\_mode = 0$)：待机模式。
- **Boost_State** ($ers\_mode = 1$)：电动机助力模式（区分 Standard 与 Overtake 状态）。
- **Regen_State** ($ers\_mode = 2$)：动能回收模式（刹车/Recharge 按键激活）。
- **Fail_Safe** ($ers\_mode = 3$)：保护模式（电池过热或 SOC 过高触发，需清故障指令恢复）。

### 4.3 内燃机系统 (`ICE_sim.slx`)

内燃机系统由 7 个核心物理与控制子模块构成：
1. **Throttle_Actuator**：包含节气门开度限幅与一阶惯性环节（时间常数 $T_{s}$）。
2. **Turbocharger**：前馈+PID闭环控制，具备 Normal/Aggressive 双模式，以及冷却液过热降额与最大增压压力（250 kPa）限幅。
3. **Combustion_Torque_Model**：基于转速/节气门二维查表（`Combustion_Torque_LUT`），结合增压增益与 `Torque_Power_Limiter`（限制 320 Nm / 400 kW）。
4. **Fuel_System**：基于 BSFC（0.24 kg/kWh）计算燃油消耗，支持三种喷油模式控制与高转速低油门断油（Fuel Cut），支持单场比赛 110 kg 燃油总量监控。
5. **Engine_Thermal_Model**：一阶热容模型，模拟燃油燃烧发热与环境散热，实时解算冷却液温度 $coolant\_temp\_C$。
6. **Gearbox**：8 档变速箱，采用转速滞回换挡逻辑（14,500 RPM 上升档，9,000 RPM 下降档），使用 `UnitDelay` 断开代数环。
7. **ICE_STM（状态机）**：包含 `Off_Standby`, `Cranking`, `Idle`, `Running`, `Fuel_Cut`, `Limp_Home`, `Fail_Safe` 7 个状态。

### 4.4 车辆纵向动力学系统 (`Vehicle_Speed_sim.slx`)

基于单质量纵向动力学方程解算车辆加速度与车速：
$$ F_{\text{net}} = F_{\text{traction}} - F_{\text{aero}} - F_{\text{rolling}} - F_{\text{brake}} $$
$$ F_{\text{aero}} = \frac{1}{2} \rho C_d A v^2, \quad F_{\text{rolling}} = f_r m g $$
解算得到 `car_speed_kph` 并反馈给 ERS、ICE 及状态机判定条件（例如 355 kph 超速强制退出 Boost）。

---

## 五、策略档位与参数配置汇总

系统支持 **Race**、**Push**、**Charge** 与 **Lift & Coast** 四种整车策略，策略映射关系与关键阈值如下表所示：

| 策略 (`strategy_cmd`) | 燃油模式 (`ice_fuel_mode`) | 油门 Boost 阈值 (%) | SOC 放电门槛 (%) | MGU-K 放电功率 (kW) | SOC 回收截止 (%) | 单圈放电上限 (MJ) | 单圈回收上限 (MJ) |
|---|---|---|---|---|---|---|---|
| **Race (0)** | `Normal` | 80 | 7 | 300 | 95 | 8.5 | 1.2 |
| **Push (1)** | `Normal` | 50 | 3 | 350 | 85 | 12.0 | 2.0 |
| **Charge (2)** | `Fuel_Saving` | 101 (禁用) | 40 | 0 | 100 | 4.0 | 4.0 |
| **Lift & Coast (3)**| `Lift_and_Coast` | — | — | — | — | — | — |

---

## 六、快速入门与仿真运行指南

### 1. 环境准备
- MATLAB / Simulink R2022b 及以上版本。
- 安装 Stateflow、Simulink Coder、Control System Toolbox。

### 2. 运行闭环集成仿真 (`Power_Unit_sim.slx`)
```matlab
% 1. 设置当前工作目录至项目根目录
cd('path/to/repository');

% 2. 依次加载公共参数与子系统参数脚本
common_params_init;
ers_params_init;
ice_params_init;

% 3. 打开顶层集成模型并运行仿真
open_system('Power_Unit_sim');
sim('Power_Unit_sim');
```

### 3. 运行独立子系统仿真
- **独立调试 ICE 系统**：直接运行 `ice_params_init.m`，打开并运行 `ICE_sim.slx`。
- **独立调试 ERS 系统**：运行 `ers_params_init.m`，打开并运行 `ERS_sim.slx`。

---

## 七、已知简化与后续改进方向

1. **机械耦合物理建模**：`Kinetic_Recovery` 内部积分器当前主要为 MGU-K 转速解算，与整车地面真实车速尚未通过传动比做硬物理耦合。
2. **电池模型精度**：当前采用线性 SOC 积分与一阶 RC 热模型，未引入电池内阻压降、OCV-SOC 极化曲线及老化衰减机制。
3. **赛道与动力学扩展**：整车模型目前为单质量纵向模型，后续可引入赛道坡度、弯道高程剖面及三自由度车辆动力学。
4. **总线与故障注入**：尚未建立 CAN / CAN-FD 总线传输延迟、网络丢包及单点硬件故障注入（Fault Injection）测试。

---

## 八、嵌入式代码生成指南

若计划使用 Embedded Coder 对模型进行 C/C++ 代码生成，请注意以下改动：

1. **求解器配置**：将求解器由 `VariableStepAuto` 改为 **Fixed-step discrete**（如定步长 `ode4`，步长建议设为 `0.001s`）。
2. **代数环消除**：确认所有反馈回路已放置 `UnitDelay` 或离散延迟模块（如 `Gearbox` 中已处理）。
3. **数据类型与 StorageClass**：
   - 将全局 `Simulink.Parameter` 的 Storage Class 明确指定为 `ExportedGlobal` 或通过 Data Dictionary 进行管理。
   - 使用 Fixed-Point Designer 评估部分浮点数转定点数或 `single` 单精度运算的需求。
4. **生成代码策略**：建议保留 `Power_Unit_sim.slx` 作为 MIL/SIL 验证台架，分别对 `ICE_sim.slx` 与 `ERS_sim.slx` 作为独立 ECU 软件组件导出代码。
