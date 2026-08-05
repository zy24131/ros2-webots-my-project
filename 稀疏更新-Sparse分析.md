# Sparse（稀疏更新）部件综合分析

## 1. 它是什么

**Sparse = 稀疏更新（Sparse Update）模式**，不是另一种命令树。

位置在 DIAG 调用链的**后半段**：命令树已经把你带到业务处理函数（如 `PORT_DIAG_TxParamSet`），并抽出 `item` 字符串（如 `"main=50,pre=-1"`）之后，由 `PORT_DIAG_ParseEqualAdd` 把这段 `key=value` 列表**只更新提到的字段**，没提到的字段保持原值。

```text
set port 0/96 tx main=50,pre=-1
        │
        ▼
DIAG 命令树（fe_diag_walk 等）
        │
        ▼
PORT_DIAG_TxParamSet(unit, port, item, side, devId)
        │
        item = "main=50,pre=-1"
        ▼
PORT_DIAG_ParseEqualAdd(item, ...)   ← Sparse 部件
        │
        ▼
只改 info.main / info.pre，其它字段不动
```

## 2. 核心行为（设计模式）

| 阶段 | 内容 |
|------|------|
| 调用前 | `{ txPort1588Delay=140, rxPort1588Delay=0, main=0, ... }` |
| 命令行 | `"main=50,pre=-1"`（只传了 main、pre） |
| 解析 | main→更新；pre→更新；未出现的字段→跳过 |
| 调用后 | `{ txPort1588Delay=140, rxPort1588Delay=0, main=50, pre=-1, ... }` |

要点：**按需改字段，未出现的字段保持原值**。这就是「稀疏」的含义。

## 3. 和命令树的关系

| 层级 | 部件 | 输入形态 | 作用 |
|------|------|----------|------|
| 上 | DIAG 命令树 | `set port 0/96 tx ...` | 路由到哪个处理函数 |
| 下 | Sparse 解析 | `main=50,pre=-1` | 往 `info` 里稀疏写字段 |

两者都是表驱动，但表不一样：

- 命令树表：`FE_DIAG_CMD_TABLE_T` / `FE_DIAG_FUN_DES_T`
- Sparse 表：`g_portDiagParseKeyInfo[item].keyMap`（key → 如何解析 value → 如何写回 info）

## 4. 关键数据结构

- **`FE_PORT_DIAG_PARSE_KEY_INFO_S`**：某类 item 的解析配置（含 `keyMap`、写回回调等）
- **`FE_PORT_DIAG_KEY_INFO_MAP_S`**：单个合法 key 的映射（key 名、value 元信息、解析函数等）
- **`info`**：业务配置结构体（被稀疏更新的目标）

用 `item` 下标选 `g_portDiagParseKeyInfo[item]`，不同端口参数集可挂不同 key 表。

## 5. 算法流程（PORT_DIAG_ParseEqualAdd）

```text
外层：按逗号切段（PORT_DIAG_ParseComma）
  │
  ├─ 校验段长：a=b 至少 2，最长 30
  │
  └─ 内层：遍历 keyMap
        ├─ ParseEqualKey：当前段是否匹配该 key，并抽出 value → sTemp
        ├─ 匹配失败 → 下一个 key
        ├─ 匹配成功 → match.fun 把 sTemp 转成 param
        │     ├─ OK → parseKeyGetFun(key, param, info) 写回结构体
        │     └─ 失败 → PutKeyValueInfo 打印/记录错误信息
        └─ break（本段处理完）
  若本段在整张 keyMap 都没匹配 → Invalid key，返回错误
  推进 startpos，处理下一段
全部成功 → FE_OK
```

特征：

1. **只遍历命令行里出现的 key=value**（外层跟输入走）
2. **keyMap 用于识别与写回**，不是把结构体所有字段扫一遍强制赋值
3. 因此未出现的字段天然「保持原值」

## 6. 为什么需要它

1. **命令行友好**：只需写要改的项，不用把整个配置重敲一遍  
2. **安全**：避免把未提及字段误清零/改成默认值  
3. **扩展**：新字段往 keyMap 加一项即可，业务函数仍收一个 `item` 字符串  
4. **和 DIAG 树配套**：树负责「调谁」，Sparse 负责「改 info 里哪几项」

## 7. 和 demo 工程的对应

| 仓库文件 | 角色 |
|----------|------|
| `diag_demo.c` | 上半段：命令树、unit 管理、提参到 `PORT_DIAG_TxParamSet` |
| `sparse.c` | 下半段：`PORT_DIAG_ParseEqualAdd` 稀疏解析 `item` |

当前 `diag_demo` 里 `item` 仍是整串打印；完整产品中会在 Tx/Rx 处理函数内部再调 Sparse 去改 `info`。

## 8. 一句话总结

**Sparse 部件 = 面向 `key=value,key=value,...` 的表驱动稀疏写回器：只更新命令行出现的字段，其余保持原值。**
