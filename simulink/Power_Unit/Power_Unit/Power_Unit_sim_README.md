# Power_Unit_sim.slx — 动力单元（PU）整车集成模型说明

> 本文档基于对 `Power_Unit_sim.slx` 内部 XML 结构的静态解析生成，未在 MATLAB 中实际打开/运行。
> 模型引用了 `ERS_sim.slx`（ModelReference），**该文件未包含在本次上传中**，
> 因此本文档中 ERS 部分仅能基于其输入/输出端口名给出接口级描述，内部逻辑未知。

## 1. 模型定位

`Power_Unit_sim.slx` 是**整个动力单元的顶层集成/联调模型**，通过 Model Reference 把三个独立子模型组合成闭环：

- `ERS_sim.slx`（电动机/电池储能系统，MGU-K，**未提供**）
- `ICE_sim.slx`（内燃机，见配套的 `ICE_sim_README.md`）
- `Vehicle_Speed_sim.slx`（车辆纵向动力学 / 车速解算）

并新增两个顶层协调子系统：`Power_Unit_Coordinator`（故障联锁）与 `Powertrain_Torque_Fusion`（扭矩融合），
同时挂了一个 `test`（`Reference`，SID 26，13 路输出）激励源模块，为**闭环自测试台**——
本模型本身没有外部 Inport，所有驱动信号（油门、策略指令、超车按钮等）都来自这个 `test` 模块，
推测与 `unified_power_unit_test.mat` 配合使用（作为 Signal Editor/Test Sequence 的测试数据源，
具体需在 MATLAB 中打开 `test` 模块确认其类型）。

依赖的外部文件：

| 文件 | 作用 |
|---|---|
| `common_params_init.m` | 整车公共参数 |
| `ers_params_init.m` | ERS 专属参数（**未提供**，模型 InitFcn 中调用） |
| `ice_params_init.m` | ICE 专属参数 |
| `unified_power_unit_test.mat` | 联调测试数据（推测供 `test` 模块使用） |

模型 InitFcn：
```matlab
common_params_init;
ers_params_init;
ice_params_init;
```

求解器：`VariableStepAuto`（ode3，仿真用；代码生成前需改定步长，见 `ICE_sim_README.md` 第 6 节同理适用）。

## 2. 顶层输入 / 输出接口

本模型**无外部 Inport**（闭环自测试台架构）。

### 输出（Outport）

| 信号名 | 来源 | 说明 |
|---|---|---|
| `torque_cmd_Nm` | `Powertrain_Torque_Fusion` 输出 | 融合后下发给车辆模型的总扭矩指令 |
| `Out1` | `Vehicle_Speed` 模型输出 `car_speed_kph` | 当前仿真车速 |

## 3. 顶层信号流

```
                    ┌─────────────────────┐
        test ──────►│ (13路激励信号)        │
                    └──────────┬──────────┘
                               │
        ┌──────────────────────┼───────────────────────┐
        ▼                      ▼                        ▼
   ┌────────┐            ┌──────────┐             ┌───────────┐
   │  ERS   │            │   ICE    │◄─car_speed──│  (反馈)    │
   │(ModelRef)│          │ (ModelRef)│             └───────────┘
   └───┬────┘            └────┬─────┘
       │ers_mode              │ice_fail_safe
       ▼                      ▼
  ┌─────────────────────────────────┐
  │  Power_Unit_Coordinator          │
  │  (Fault_Interlock_Logic)         │
  └───┬───────────┬─────────────┬────┘
      │ice_enable │mguk_enable  │ers_in_failsafe
      │           │             └──────────► 反馈给 ERS(in2) 和 ICE(in8)
      ▼           ▼
  ┌─────────────────────────────────┐
  │  Powertrain_Torque_Fusion        │  ers_torque_Nm ──┐
  │  (两路 Switch 门控 + Sum)         │◄─────────────────┤
  └───────────────┬───────────────────┘  ice_torque_Nm ──┘
                   │ torque_cmd_Nm
                   ▼
            ┌──────────────┐
            │ Vehicle_Speed │──► car_speed_kph ──► 反馈给 ERS / ICE
            │  (ModelRef)   │
            └──────────────┘
```

## 4. 子系统详解

### 4.1 `Power_Unit_Coordinator`（故障联锁协调器）

输入：`ers_mode`、`ice_fail_safe`、参数 `Enable_Bidirectional_Interlock`（默认 `false`）
输出：`ice_enable`、`mguk_enable`、`ers_in_failsafe`

内部为 MATLAB Function `Fault_Interlock_Logic`：

```matlab
ers_in_failsafe = (double(ers_mode) == 3);   % 模式 3 视为 ERS Fail-Safe
ice_enable = ~ers_in_failsafe;               % 单向互锁：ERS 故障 → 切断 ICE

if Enable_Bidirectional_Interlock
    mguk_enable = ~ice_fail_safe;            % 双向互锁：ICE 故障 → 反向切断 MGU-K
else
    mguk_enable = true;                      % 默认：ICE 故障不影响 MGU-K
end
```

**逻辑要点**：默认只做**单向联锁**（ERS 故障会切断 ICE 输出），
ICE 故障是否反向切断 ERS/MGU-K 由 `Enable_Bidirectional_Interlock` 开关控制，默认关闭。

### 4.2 `Powertrain_Torque_Fusion`（扭矩融合）

输入：`ers_torque_Nm`、`ice_torque_Nm`、`mguk_enable`、`ice_enable`
输出：`torque_cmd_Nm`

逻辑：两路 `Switch` 分别以 `mguk_enable`/`ice_enable` 为门控条件，
使能为假时用常量 0 替代对应扭矩，两路结果相加得到 `torque_cmd_Nm`。
即：**任一动力源被联锁切断时，其扭矩贡献直接归零，不影响另一路**。

### 4.3 三个 Model Reference 接口

| 模块 | 引用文件 | 输入端口 | 输出端口 |
|---|---|---|---|
| `ERS` | `ERS_sim.slx`（**未提供**） | `overtake_enabled`, `ers_strategy_cmd`, `overtake_button`, `recharge_button`, `brake_pressed`, `boost_button`, `throttle`, `detection_gap_valid`, `lap_trigger`, `ers_clear_fault`, `car_speed_kph_fb` | `ers_mode`, `ers_torque_Nm`, `ers_failsafe` |
| `ICE` | `ICE_sim.slx` | `overtake_button`, `overtake_enabled`, `detection_gap_valid`, `ignition_on`, `ice_throttle_cmd_pct`, `ice_clear_fault`, `ice_strategy_cmd`, `ers_in_failsafe`, `car_speed_kph` | `ice_fail_safe`, `ice_torque_out_Nm` |
| `Vehicle_Speed` | `Vehicle_Speed_sim.slx` | `torque_cmd_Nm`, `brake_pressed` | `car_speed_kph` |

> ⚠️ `ERS` 模块有 11 个输入端口，但顶层 `system_root` 中实际连线只覆盖了其中一部分
> （其余由 `test` 模块或常量供给）。若需完整重建输入映射，建议在 MATLAB 中打开模型查看 `test` 模块的连线详情。

## 5. 使用建议

- **独立调试 ICE**：直接打开 `ICE_sim.slx`（见配套 README），无需依赖本模型。
- **联调/回归测试**：打开本模型，运行即可驱动 `test` 模块中的预置激励，观察 `torque_cmd_Nm` 与车速闭环响应；
  `unified_power_unit_test.mat` 大概率是该测试台架对应的期望数据/输入数据集，建议结合 Simulink Test 或 Simulink Data Inspector 比对。
- **补充 ERS 模型**：若要完整仿真闭环，需要 `ERS_sim.slx` 与 `ers_params_init.m` 一并加入工程路径。

## 6. 代码生成注意事项

顶层是一个**含测试激励台架的闭环模型**，通常不会直接对整个 `Power_Unit_sim.slx` 做嵌入式代码生成；
更常见的做法是分别对 `ICE_sim.slx` / `ERS_sim.slx` / `Vehicle_Speed_sim.slx`（作为独立 ECU 软件组件）生成代码，
`Power_Unit_sim.slx` 保留作为**MIL/SIL 闭环验证平台**。若确实需要生成 `Power_Unit_Coordinator` /
`Powertrain_Torque_Fusion` 这类顶层协调逻辑的代码，可以把它们单独抽成子模型再走 Embedded Coder 流程。
其余定步长求解器、数据类型、参数 StorageClass 等注意事项同 `ICE_sim_README.md` 第 6 节。
