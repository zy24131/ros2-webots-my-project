# 交换机/FE 端口 DIAG 诊断与参数下发框架（C）

个人项目 / 嵌入式平台软件 | C | 表驱动 CLI、稀疏更新、分层驱动分发

---

## 项目概述（总述，可直接贴简历）

1. 主导设计并实现基于表驱动的 DIAG 命令解析与端口参数下发框架，采用命令树路由 + 函数描述符分发，将文本诊断命令映射为处理函数调用，适配多 unit/多端口场景下的可扩展诊断需求。通过统一解析与分层下发，降低业务侧命令接入与芯片适配的耦合成本。
2. 打通「命令解析 → 稀疏配置更新 → 分层芯片下发」全链路：支持如 `set port 0/96 tx main=50,pre=-1` 经 DIAG 树命中 `PORT_DIAG_TxParamSet`，再经 Get/Sparse/Set 完成局部改配与硬件生效；PortDrv 与 MACsec 采用同一套函数指针挂钩子思路，实现平台解耦。
3. 针对 SerDes 重初始化等场景设计两次下发容错机制，并实现 key=value 稀疏更新，保证未提及字段保持原值，提升配置变更安全性与运维效率。

**一句话标题版：**

> 交换机端口 DIAG 表驱动解析与分层下发框架（C）：实现命令树路由、稀疏配置更新、函数指针芯片分发与两次下发容错，打通诊断命令到硬件生效全链路。

---

## DIAG 命令解析模块

1. 设计命令表（`FE_DIAG_CMD_TABLE_T`）与函数描述符（`FE_DIAG_FUN_DES_T`）两级核心结构，支持父命令挂子表、叶子挂处理函数，实现多叉命令树的表驱动扩展，避免业务层堆积解析 `if-else`。
2. 实现通用命令树递归遍历（`fe_diag_walk`）与多 unit 管理（`g_pstFeDiagMgmt[unit]->pstCmdList`），结合 `FE_DIAG_USE_UNIT` 解析 `unit/port`（如 `0/96`），完成从根表到叶子描述符的逐级匹配。
3. 实现参数类型分发（`fe_diag_dispatch_parms`），按 `aucParmType[]`（NUM/STR）统一提参并支持缺省约定（如 `tx→LSW_SEND`），将「怎么解析参数」从业务处理函数中剥离。

---

## 稀疏更新（Sparse Update）模块

1. 设计稀疏更新模式：命令处理采用「Get 当前参数 → `PORT_DIAG_ParseEqualAdd` 局部修改 → Set 下发」三阶段流程，仅更新命令行出现的 `key=value`，未提及字段保持原值，避免全量重配误覆盖。
2. 实现按逗号切段、`keyMap` 匹配与回调写回的表驱动解析（`PORT_DIAG_ParseComma` / `ParseEqualKey` / `parseKeyGetFun`），新增配置项以注册 keyMap 为主，提升扩展性。
3. 通过外层遍历命令行 pair、内层匹配合法 key 的双层循环，保证只处理输入中存在的字段，从机制上实现「稀疏」语义。

---

## PortDrv 分层下发模块

1. 采用业务/适配/芯片/驱动四层结构：`PORT_DIAG_TxParamSet` → `NpLswTxParamSet` → `AdaptLswDispatchSetPortTxParams` → `NP_PORT_DISPATCH_SET_PORT_TXPARAMS` → `Np5993PortDrvSetTxRxParams` → 驱动 API，实现诊断命令与芯片实现解耦。
2. 设计函数指针分发表（`g_fnNpPortDriver[unit].set_port_txparams`），并通过宏 `NP_PORT_DISPATCH_SET_PORT_TXPARAMS` 统一封装查表调用；初始化阶段注册 5993 芯片实现，业务阶段仅通过该宏/接口分发，支持按 unit 切换芯片策略。
3. 在 `NpLswTxParamSet` 中实现两次下发容错：先 `AdaptLswDispatchSetPortTxParams`，再 `ConfigPortSetTxParamByCmd` 重发（二者内部均经 `NP_PORT_DISPATCH_SET_PORT_TXPARAMS` 落到芯片实现），应对部分芯片 SerDes 重初始化导致的参数丢失问题。

---

## MACsec 驱动注册模块

1. 采用策略模式 + 驱动注册模式（挂钩子）：初始化 `Np5993MacsecFuncRegister(&g_fnNpPortDriver)` 注入 `np_create_sc` 等函数指针，业务通过 `NpPortXsecCreateSC` → 适配层 → `fnDriver->np_create_sc` 实际执行芯片实现。
2. 与 PortDrv 业务域分离、扩展方式统一：均以「注册挂钩子、运行时调接口」实现平台解耦，便于多芯片并行演进而不改上层调用方式。

---

## 超短版（仅 3 条，版面紧时用）

1. 设计并实现表驱动 DIAG 命令树与函数描述符分发，支持多 unit 管理与参数类型统一提参，将文本命令路由到处理函数。
2. 实现稀疏更新：Get 当前配置后按 `key=value` 只改命令行字段，再分层 Set 下发，保证未提及字段保持原值。
3. 实现 PortDrv/MACsec 函数指针挂钩子分层架构，并在 TX 参数路径设计两次下发容错，打通诊断命令到芯片生效全链路。

---

## 面试提示

- 能画清全链路：命令树 → `PORT_DIAG_TxParamSet` → Get/Sparse/Set → Adapt → 5993 → 驱动。
- 能区分：Sparse（改哪些字段）vs 分层挂钩子（怎么下到哪颗芯片）vs 两次下发（为何发两遍）。
- 表述建议用「按平台框架实现并串联」，避免「从零发明架构」；量化数据如无实测，不要写「降低 70%」这类无法佐证的数字。
