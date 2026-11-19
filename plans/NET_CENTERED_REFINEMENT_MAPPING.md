# 最大跨板需求 Net 中心 + 线图局部展开：伪代码—源码映射

日期：2026-11-17
状态：D1–D10 已确认；模块 1（读取、状态重建、需求统计）已实现并核验，未执行任何 Move 搜索。后续 NC 搜索的接入路线已按 2026-11-17 修订为“逐层反粗化 + 需求/线图引导候选激活 + 原有堆选择动作”，见第 15 节。

本方案在本文档中简称 **NC**（Net-Centered）。其 A/B/C 三个实验版本为避免与项目既有基线 A、社区 H、评分 S 混淆，实验中建议命名为 **NC-A / NC-B / NC-C**。

> 说明：下文映射表里的“尚未确认项”保留首轮核对时的源码风险点；最终采用的规则以第 10 节“已确认决策”为准。

## 0. 本轮结论摘要

1. 伪代码中的 Net 可直接对应现有 `Hyperedge`：`_incidence_array` 的第一个 pin 是源点，其余是汇点；`Hypernode::label` 是主位置 P(v)。
2. “最大跨板需求中心 + 一跳 Net 邻域”不需要构建完整线图，可以在选中中心后通过“中心 Net → 节点 → 关联 Net”现场累加。
3. 完整 Gain 可以按伪代码要求覆盖被移动节点的全部关联 Net；现有独立探针 `refinement_native_probe::netCost` 已实现 V2 WTHD（含副本语义），且既有实验已核对原生 Move 堆键与完整 WTHD 差一致。
4. **不能直接静默复用 `Hyperedge::target_pins_in_part` 作为伪代码的 \(c_{e,b}\)**：该缓存按 pin 出现次数计数，包含 sink 副本板，并排除 source 副本板；伪代码只按 sink 的主位置 \(P(t)\) 计数。有副本或重复 pin 时两者不同，见第 9 节。
5. **不能直接复用 `hopDistanceCost` 作为候选 Gain 或合法性判断**：它在不可达 hop 为负时仍把负值加进代价；候选评估应以 `netCost` 的抛错行为判断不可达。
6. 现有 `moveIsFeasible` 只检查 8 维资源，不检查通信容量、hop 可达性或“目标板已有该节点副本”的非法重叠；本方案要输出合法划分，必须显式补这三类检查。
7. D1 已确认：模块 1 和同起点实验使用**独立后处理工具**，读取同一份放置快照。后续搜索的正式接入路线已在 2026-11-17 修订为：**逐层反粗化后，用需求和线图规则决定哪些候选先被激活，仍由原细化器的堆选择具体动作**；不再把“独立工具只做正收益 Move”当作替代原细化流程的方案，见第 15 节。

## 1. 输入、层与目标函数

### 1.1 输入层

| 项目 | 映射 | 源码位置 | 说明 |
|---|---|---|---|
| 逻辑超图 | `Hypergraph`，节点=`Hypernode`，Net=`Hyperedge` | `RePart/datastructure/hypergraph.h:31-120` | 方案要求“原始逻辑节点”，应使用最细层 `Hypergraphs[0]`，不是粗化层 |
| 源点/汇点 | `_incidence_array[e._begin]` 为源；`[_begin+1, _begin+_size)` 为汇 | `hypergraph.h:112-114`，`hypergraph.h:134-136`；读取见 `RePart/io/hypergraph_readin.h:43-62` | `design.net` 每行第一个名字为源，其余为汇 |
| 当前划分 P(v) | `hypernodes[v].label` | `hypergraph.h:56-68` | 主位置；副本在 `replication_fpga_labels` |
| 8 维资源需求 | `hypernodes[v].weights[0..7]` | `hypergraph.h:56-68`；读取见 `hypergraph_readin.h:21-29` | 用户举例 LUT/FF/BRAM/DSP，实际输入是 8 维，沿用现有 8 维 |
| FPGA 容量 | `fpga_manager.fpga_info[b][1..8]` | `RePart/io/fpga_manager.h:14-24, 93-97` | `fpga_info[b][0]` 是通信容量 |
| 资源占用 | `std::vector<std::vector<int>> fpga_resource_usage` | `RePart/refinement_move/refiner_base.h:20-27` 持有引用 | 每板 8 维 |
| 通信占用 | `std::vector<int> fpga_comm_weight_sum` | `RePart/refinement_move/kway_fm_refiner.h:42, 175-191` | 每板一个标量 |
| 目标函数 | V2 加权 THD：每个 Net 源主位置到 sink 主位置/副本位置去重后的跳数和 × Net 权重 | `RePart/refinement_native_move_probe.h:9-33` | 与 `tools/replication_thd_validator.py` 的 v2 定义一致 |

### 1.2 目标函数必须使用 V2，而不是程序自报 Best Cost

进度记录已确认：含副本时，程序自报 Best Cost 与最终 V2 WTHD 可能相差 +1 到 +422。本方案的“最终目标值、实际 Gain”统一用 `refinement_native_probe::cost()` 的 V2 口径；不能用 `partitioner` 打印的 `Best Cost` 直接代替。依据：`PROJECT_PROGRESS.md` 2026-11-17 服务器条目及 `runs/titan_server_20261115/REPORT.md`。

### 1.3 现有可复用接口

| 接口 | 作用 | 源码位置 | 备注 |
|---|---|---|---|
| `Hypergraph::pins(e)` | 取 Net 的 pin 区间 | `hypergraph.h:112-114` | 源点在区间首 |
| `Hypergraph::getSourceid(e)` | 取源点 | `hypergraph.h:134-136` | |
| `Hypergraph::pinCountInPart(e,b)` | 取缓存的目标板 pin 数 | `hypergraph.h:122-124` | 注意第 9 节的语义差异 |
| `Hypergraph::initPinsInPart()` | 从当前 label/副本重建 `source_part_id` 与 `target_pins_in_part` | `hypergraph.h:358-383` | 副本感知；不清零，必须只在新建状态上调用一次 |
| `FMRefinerBase::moveIsFeasible(u,from,to)` | 8 维资源检查 | `refiner_base.h:92-100` | 只查目标板 |
| `KWayFMRefiner::evaluateFPGACommChange(u,from,to,delta)` | 计算通信占用变化 | `kway_fm_refiner.h:193-279` | 不提交，供容量判断 |
| `KWayFMRefiner::checkAndChangeComm(u,from,to,delta)` | 检查并提交通信占用 | `kway_fm_refiner.h:175-191` | 选中的 Move 才调用 |
| `KWayFMRefiner::moveHypernode(u,from,to)` | 提交 label、源板、目标板 pin 计数、8 维资源 | `kway_fm_refiner.h:135-173` | 副本语义内建；不更新通信总量 |
| `refinement_native_probe::netCost(g,e,moving,to)` | 单个 Net 的 V2 代价；投影某节点到目标板 | `refinement_native_move_probe.h:9-28` | 不可达 hop 会抛 `std::runtime_error` |
| `refinement_native_probe::cost(g)` | 全图 V2 代价 | `refinement_native_move_probe.h:29-33` | 用于最终核验和可选逐步核验 |
| `ConnectionProjector::load` / `validatePlacements` | 读取并校验 `design.fpga.out` | `RePart/route_prediction/connection_projector.cpp:21-109` | 独立工具可复用 |
| `route_prediction_b5::writePlacement` | 写出 `design.fpga.out` 格式 | `RePart/route_prediction/placement_writer.cpp:14-32` | 独立工具可复用 |

## 2. 伪代码—源码对象映射总表

| 伪代码变量/对象 | 数学含义 | C++ 类型/表示 | 数据来源 | 拟复用函数及源码位置 | 尚未确认项 |
|---|---|---|---|---|---|
| \(e, f, g\) | Net 编号 | `int`（`Hyperedge::id`） | `hypergraph.hyperedges` | `hypergraph.pins(e)` `hypergraph.h:112-114` | 层 0 是否存在 deleted Net；本方案应跳过 deleted |
| \(s_e\) | Net 源点 | `int` 节点编号 | `_incidence_array[e._begin]` | `getSourceid` `hypergraph.h:134-136` | 源点是否也出现在 sink 列表 |
| \(\operatorname{sinks}(e)\) | 汇点列表 | pin 区间 | `_incidence_array[e._begin+1 .. e._begin+e._size)` | `pins(e)` | 按“出现次数”还是“去重后的 sink 身份”计数 |
| \(P(v)\) | 节点主位置板 | `int` | `hypernodes[v].label` | | 副本是否参与本方案的“节点所在板” |
| \(\operatorname{rep}(v)\) | 副本板集合 | `std::unordered_set<int>` | `hypernodes[v].replication_fpga_labels` | | 是否参与 \(c_{e,b}\)、候选目标板 |
| \(\mathcal N(v)\) | 包含 v 的 Net 集合 | 两个 `unordered_set<int>` 的并集 | `incident_nets_as_source`、`incident_nets_as_target` | `hypergraph.h:61-62` | 同一 Net 同时出现在两侧时需去重；deleted 过滤 |
| \(w_e\) | Net 权重 | `int`（`Hyperedge::weight`） | `hypergraph.h:35` | `netCost` 内使用 | 是否可能为 0 或负 |
| \(R(v)\) | 8 维资源需求 | `vector<int>[8]` | `hypernodes[v].weights` | `moveIsFeasible` `refiner_base.h:92-100` | 用户只列 4 类，实际 8 维 |
| \(C_b\) | FPGA 8 维容量 | `fpga_info[b][1..8]` | `fpga_manager.h:14-24, 93-97` | `moveIsFeasible` | |
| \(U_b\) | 当前 8 维占用 | `vector<vector<int>>` | `fpga_resource_usage` | `moveHypernode` `kway_fm_refiner.h:135-173` | 独立工具需从 placement 重建并核验 |
| \(\operatorname{comm}_b\) | 当前通信占用 | `vector<int>` | `fpga_comm_weight_sum` | `evaluateFPGACommChange`/`checkAndChangeComm` `kway_fm_refiner.h:175-279` | 独立工具需从 placement 重建并核验；伪代码未把它列为约束 |
| \(C(P)\) | V2 加权 THD | `int64_t` | 由 pin/label/副本/hop 计算 | `refinement_native_probe::cost` `refinement_native_move_probe.h:29-33` | 不用程序自报 Best Cost |
| \(G\) | 单节点 Move 的完整 Gain | `int64_t` | 受影响 Net 前后 `netCost` 差 | `refinement_native_move_probe.h:9-28` | 用独立 `netCost` 还是现有 kpq 键，见第 10 节 D5 |

## 3. \(d_e\)：自身跨板需求

### 3.1 公式与数据

\[
c_{e,b}=|\{t\in\operatorname{sinks}(e):P(t)=b\}|,\qquad
d_e=\sum_{b\ne P(s_e)}\mathbf 1[c_{e,b}>0].
\]

| 项 | 类型 | 数据来源 | 拟用实现 | 尚未确认项 |
|---|---|---|---|---|
| \(c_{e,b}\) | `int` 或 `vector<int>` | 遍历 Net 的 sink pins 与 `label` | 新增按 Net 现算，或复用 `Hyperedge::target_pins_in_part` | 副本/重复 pin 语义，见第 9 节 |
| \(d_e\) | `int`（最大为板数−1） | 由 \(c_{e,b}\) 导出 | 新增 `std::vector<int> d_e` | source 的副本是否应视为已覆盖板 |
| 跨板 Sink 数 | `int` | 遍历 sink，比较 \(P(t)\) 与 \(P(s_e)\) | 新增 `std::vector<int> cross_sink_count` | 记录用，不进入评分；按出现次数还是去重身份 |
| 跨板 Net 判据 | `bool` | \(d_e>0\) | `d_e[e] > 0` | |
| 最大需求堆 | 最大 \(d_e\)，同值最小 Net ID | `std::set<std::pair<int,int>>`，键 `{-d_e, e}`；或带惰性失效的堆 | 新增 | 数据结构是工程决定；语义固定为“最大 \(d_e\)、并列最小 Net ID” |
| 已检查标记 | 当前划分状态下检查过 | `std::vector<char>` 或 epoch 计数器 | 新增 | 成功后统一失效；第一版按伪代码全部清除 |

### 3.2 更新范围

一次成功 Move \(u:A\to B\) 后，\(c_{e,b}\)、\(P(s_e)\)、\(d_e\) 只可能对 **包含 u 的 Net** 变化：

\[
I(u)=\operatorname{incident\_nets\_as\_source}(u)\cup\operatorname{incident\_nets\_as\_target}(u).
\]

第一版建议对 \(I(u)\) 中的每条 Net 重新扫描其 pins 计算 \(d_e\) 和跨板 Sink 数，复杂度为这些 Net 的 pin 数；不引入第二套 \(c_{e,b}\) 缓存。工程上会记录每个 Move 更新的 Net 数，便于评估开销。

## 4. \(A_{ef}\)、中心选择与局部处理顺序

### 4.1 \(A_{ef}\) 的映射

\[
A_{ef}=\sum_{v\in e\cap f}\frac{1}{|\mathcal N(v)|-1}.
\]

| 项 | 类型 | 数据来源 | 拟用实现 | 尚未确认项 |
|---|---|---|---|---|
| \(\mathcal N(v)\) | `unordered_set<int>` | `incident_nets_as_source` ∪ `incident_nets_as_target` | 直接遍历两个集合；同一 Net 需去重 | 重复 pin 或同 Net 两侧出现 |
| \(\deg(v)=|\mathcal N(v)|\) | `int` | 上述并集大小 | 初始化时预计算 `net_degree[v]`；单纯 Move 不改变连接，可保持不变 | 若后续加入复制/删除，必须重算 |
| \(\frac1{\deg(v)-1}\) | `double` | 预计算 | `inv_deg_minus_1[v]`；\(\deg(v)=1\) 时不使用 | 精度与比较容差 |
| 一跳邻域 \(\mathcal R(e^*)\) | `unordered_map<int,double>` 或 touched 数组 | 中心 Net 的 pin 节点 → 各自 \(\mathcal N(v)\) → 累加 | 新增 `build_neighborhood(e*)` | 邻居数量上限：伪代码未设，第一版不截断 |
| \(A_{e^*f}\) | `double` | 上述累加 | 同一映射 | 浮点累加顺序；建议按 Net ID 固定顺序 |

**实现要点（不建完整线图）**：对中心 \(e^*\) 的每个**去重后的 pin 节点 v**，遍历 \(\mathcal N(v)\setminus\{e^*\}\)，对每个 f 执行 `A[f] += inv_deg_minus_1[v]`。同一 f 被多个共享节点命中时自然累加；\(\deg(v)=1\) 的节点不会产生任何 f，因此分母不会为 0。

### 4.2 中心选择

| 项 | 映射 |
|---|---|
| 候选中心 | `d_e[e] > 0` 且当前 epoch 下未检查 |
| 选择键 | 最大 \(d_e\)；并列最小 Net ID（伪代码明确要求固定 Net ID） |
| 结果 | \(e^*\) |
| 数据源 | `d_e`、`checked_epoch`、`std::set<pair<int,int>>` |
| 尚未确认 | 是否只处理未 deleted Net；层 0 通常无；仍按 `deleted` 过滤 |

### 4.3 局部范围与排序

\[
\mathcal R(e^*)=\{e^*\}\cup\{f:A_{e^*f}>0\}.
\]

\[
S(f\mid e^*)=\frac{d_f}{\max(1,\max_{g\in\mathcal R(e^*)}d_g)}
+\alpha\frac{A_{e^*f}}{\max_{g\in\mathcal R(e^*)\setminus\{e^*\}}A_{e^*g}}.
\]

| 项 | 类型 | 映射 | 尚未确认项 |
|---|---|---|---|
| \(\mathcal R(e^*)\) | `unordered_map<int,double>` 的键集合 | `center + neighbor A>0`；保留 \(d_f=0\) 的邻居 | 伪代码未设邻居截断，第一版不截断 |
| 中心优先 | 排序规则 | 中心固定第一 | |
| 第一项分母 | `int` | `max(1, max d_g over R)` | |
| 第二项分母 | `double` | `max A` over `R\{e*}`；没有邻居时整项不计算 | 邻居 A 全为 0 不应出现（邻居定义为 A>0） |
| \(\alpha\) | `double` | 运行时参数 | 探索值 0.5/1/2；默认值待定 |
| 同分排序 | 未在伪代码指定 | **建议**：Net ID 升序 | 需要确认；会改变处理顺序和结果 |
| 版本 A | \(\mathcal R=\{e^*\}\) | 不建邻域 | |
| 版本 B | 中心 + 一跳邻居，\(\alpha=0\) | 第二项为 0 | |
| 版本 C | 同 B，\(\alpha>0\) | 第二项生效 | |

## 5. 从 Net 生成单节点 Move 与完整 Gain

### 5.1 候选节点与候选目标板

对当前 Net f：

| 项 | 映射 | 源码/数据 | 尚未确认项 |
|---|---|---|---|
| 候选逻辑节点 | f 的源点 + 全部汇点，去重 | `pins(f)`；源点在首 | 是否允许有副本的节点；重复 pin 去重 |
| 待移动节点 u | 上述某个节点 | `hypernodes[u]` | |
| 起点 from | \(P(u)\) | `hypernodes[u].label` | |
| 候选目标板集合 | \(\mathcal B_f=\{P(v):v\in \text{nodes}(f)\}\)，去掉 `from` | labels | 是否把副本板也加入 \(\mathcal B_f\)；目标板是 u 的副本板时必须拒绝 |
| 目标板 b | \(\mathcal B_f\setminus\{from\}\) | | 候选生成顺序建议按板 ID 升序 |

伪代码明确：**高需求 Net 不强制整条放入一块板，也不强制移动源点**；因此源点和全部汇点都进入候选，具体动作由完整 Gain 决定。

### 5.2 合法性检查

| 检查 | 复用 | 源码位置 | 是否伪代码明示 | 说明 |
|---|---|---|---|---|
| 8 维异构资源 | `FMRefinerBase::moveIsFeasible(u,from,to)` | `refiner_base.h:92-100` | 是（“先检查移动后的异构资源约束”） | 只检查目标板；移出不会增加源板占用 |
| 通信容量 | `evaluateFPGACommChange` 填 delta，再逐板比较 `fpga_comm_weight_sum + delta <= fpga_info[b][0]` | `kway_fm_refiner.h:193-279`；用法先例 `refinement_native_move_probe.h:55-61` | 否 | “输出合法划分”要求保留；第一版建议检查 |
| hop 可达性 | `netCost` 对不可达目的板抛异常；或显式 `getHopDistance < 0` | `refinement_native_move_probe.h:23-24`；`fpga_manager.h:291-303` | 否 | 现有 `hopDistanceCost` 会把负 hop 加进代价，不能作为合法性判断 |
| 主位置/副本重叠 | 拒绝 `to ∈ repl(u)`；拒绝 `to == from` | `unified_native_heap_adapter.cpp:74` 已有先例 | 否 | 否则会生成同一节点主/副本同板，输出非法 |
| 资源、通信、hop 之后才计算 Gain | 伪代码顺序 | | 是（资源→Gain） | 通信/hop 的检查顺序会影响“Gain 计算次数”计数，需固定 |

### 5.3 完整 Gain

\[
G(u:A\to B)=\sum_{e\in I(u)} \left[C_e(P)-C_e(P\circ(u:A\to B))\right],
\]

其中 \(I(u)\) 是 u 作为 source 或 sink 的全部 Net，**包含局部范围之外的 Net**。

| 项 | 映射 | 源码位置 | 尚未确认项 |
|---|---|---|---|
| \(C_e(P)\) | `refinement_native_probe::netCost(g,e)` | `refinement_native_move_probe.h:9-28` | V2 口径；不可达会抛错 |
| \(C_e(P\circ move)\) | `refinement_native_probe::netCost(g,e,u,to)` | 同上 | |
| \(I(u)\) | `incident_nets_as_source[u]` ∪ `incident_nets_as_target[u]`，过滤 deleted | `hypergraph.h:61-62` | 重复 Net 去重 |
| G 类型 | `int64_t` | | 现有 kpq 键是 `int`；V2 总量使用 long long |
| 一致性 | 既有探针已验证原生 Move 堆键 = 完整 WTHD 差 | `refinement_native_move_probe.h:44-78`；`tools/unified_native_heap_adapter.cpp:71-93` | 本方案仍需自己的逐步核验 |
| 替代方案 | 现有 `KWayFMRefiner::rateOneNodeToAllParts` + `kpq[to].getKey(u)` | `kway_fm_refiner.h:347-474` | 只对激活的边界 Net 和正评分板生成键；不是任意 (u,to) 的完整查询接口，需额外处理激活状态 |
| 同状态重复 Move | 缓存 `(u,to) -> gain/illegal` | 新增 `unordered_map<uint64_t,...>` | 缓存只在新划分状态失效；建议每个成功 Move 后清空 |
| 选择规则 | 当前 Net 内最大正 Gain；无正 Gain 换下一个 Net | 新增 | 同 Gain 平局未指定；建议节点 ID、目标板 ID 升序 |
| 严格正 | `gain > 0` | | 不执行零/负收益动作 |

### 5.4 单节点 Move 的控制流

```
扫描一次局部范围：
  for f in [e*] + sorted(R \ {e*}, S 降序, Net ID 升序):
      best = 无
      for u in distinct(source(f) ∪ sinks(f)):       # 建议按 pin 顺序，源点先
          for b in sorted({P(v) for v in nodes(f)} \ {P(u)}):  # 板 ID 升序
              if b in repl(u): 跳过
              if not resource_feasible(u, P(u), b): 跳过
              if not comm_feasible(u, P(u), b): 跳过
              if cached(u,b) is illegal: 跳过
              G = full_gain(u, b)                     # hop 不可达则视为非法
              if G > best.gain: best = (u,b,G,f)
      若 best.gain > 0:
          提交 best（通信检查并提交 → moveHypernode）
          更新资源、通信、target pins、源板、d_e、堆
          清空 checked / 提升 epoch
          清空 Gain 缓存
          结束本次局部扫描，重新选全局中心
  若整个局部范围都无正 Gain:
      标记中心 e* 为当前划分状态下已检查
```

这与伪代码第 5、6 节一致；版本 A 只把 `[e*] + ...` 缩短为 `[e*]`。

## 6. 成功 Move 后的增量更新

执行 \(u:A\to B\) 后：

| 数据 | 更新范围 | 拟复用/新增 | 源码位置 | 注意 |
|---|---|---|---|---|
| 8 维资源占用 | 板 A、B | `KWayFMRefiner::moveHypernode` | `kway_fm_refiner.h:135-173` | 每维加减 `weights` |
| 通信占用 | 板 A、B 及受影响板 | `checkAndChangeComm` 在提交前更新 | `kway_fm_refiner.h:175-191` | 选中后再调；候选评估只算 delta |
| `target_pins_in_part`、`source_part_id` | 所有含 u 的 Net | `moveHypernode` | `kway_fm_refiner.h:135-173` | 副本感知；见第 9 节 |
| \(c_{e,b}\)、\(P(s_e)\)、\(d_e\)、跨板 Sink 数 | \(I(u)\) 中的 Net | 重新扫描 pins 计算 | 新增 | 建议不新建第二套 \(c\) 缓存 |
| 最大需求堆 | \(d_e\) 变化的 Net | 从 `std::set` 删除旧键、插入新键 | 新增 | 旧 \(d_e\) 需保留；\(d_e=0\) 不插入 |
| \(A_{ef}\) | 不变 | 单纯 Move 不改变节点—Net 连接 | 伪代码第 6 节 | 若日后加入复制/删除必须重算 |
| Gain 缓存 | 全部失效 | 清空 | 新增 | 新划分状态下 Gain 全部重算 |
| 已检查标记 | 全部失效 | epoch 计数器自增，或 `vector<char>` 全部清零 | 新增 | 伪代码要求成功 Move 后统一失效 |
| 目标函数 | 减 Gain | `cost -= gain` | | 可选 `verify` 模式每步用 `cost(g)` 全量核验 |

## 7. 停止规则、计数与记录

### 7.1 停止规则

| 规则 | 实现判据 | 记录 |
|---|---|---|
| 所有 \(d_e=0\) | 活跃集合为空且不存在 \(d_e>0\) | `stop_reason=no_cross_board_net` |
| 所有跨板中心已检查且无正 Gain | 活跃集合中的 Net 全部处于当前 checked epoch | `stop_reason=all_centers_checked` |
| Gain 计算预算 | `gain_evals >= max_gain_evals` | `stop_reason=gain_budget` |
| 运行时间上限 | `steady_clock` 超过上限 | `stop_reason=time_limit` |

只接受严格正 Gain，因此每次接受后 V2 总代价必下降；停止只说“本方案检查范围内未找到改善”，不声称全局最优。

### 7.2 计数

| 计数 | 含义 | 建议类型 |
|---|---|---|
| `start_cost` / `final_cost` | 输入/输出 V2 WTHD | `int64_t` |
| `total_gain` | 接受 Move 的 Gain 之和 | `int64_t` |
| `accepted_moves` | 接受动作数 | `uint64_t` |
| `gain_evals` | 实际完整 Gain 计算次数（缓存命中不计） | `uint64_t` |
| `centers_examined` / `centers_checked` | 中心扫描次数/无收益检查次数 | `uint64_t` |
| `runtime_seconds` | 墙钟 | `double` |
| `initial/final_cross_nets` | \(d_e>0\) 的 Net 数 | `int` |
| `initial/final_sum_d` | \(\sum_e d_e\) | `int64_t` |

### 7.3 动作记录字段

每个接受动作至少记录：

| 字段 | 含义 |
|---|---|
| `seq` | 动作序号 |
| `center_net` | 当前中心 \(e^*\) 的 Net ID |
| `candidate_net` | 产生该候选的 Net f |
| `is_center_net` | f 是否就是中心 |
| `d_center` / `d_f` | 中心/候选 Net 的 \(d_e\) |
| `association` | \(A_{e^*f}\)；f 为中心时记为 `-` 或 0，需确认 |
| `shared_nodes` | 可选：共享节点 ID，便于解释 |
| `node_id` / `node_name` | 被移动逻辑节点 |
| `from_board` / `to_board` | 起点/目标板 ID 与名字 |
| `gain` | 完整 V2 Gain |
| `cost_after` | 提交后的 V2 总代价（可选） |

另建议输出逐 Net 的 `d_e`、跨板 Sink 数、源板、不同目的板集合，起始/结束各一份，便于解释和复算。

## 8. 版本 A/B/C 与实验口径

| 版本 | 中心与候选组织 | 目的 | 实现顺序 |
|---|---|---|---|
| NC-A | 最大 \(d_e\)，只检查中心 Net | 单条高需求 Net 的直接调整效果 | 先做 |
| NC-B | 中心 + 一跳邻居，\(\alpha=0\) | 关联范围本身是否带来收益 | 再做 |
| NC-C | 同 B，\(\alpha>0\) | 关联加权排序是否进一步改善 | 最后只加 \(\alpha\) 评分 |

第一轮 \(\alpha=0.5,1,2\) 是探索值，不是已确认最优。三个版本必须：

1. 从**完全相同的输入划分**开始；
2. 使用相同的 8 维资源/通信/hop 约束、完整 Gain、候选目标板规则和计算预算；
3. 保留原有细化流程作为参考；
4. 记录最终目标值、接受动作数、Gain 计算次数、运行时间；
5. 接通独立路由器后，再比较实际路由后的 TDM 与延时。

**同起点实验口径（D1/D9 已确认）**：NC-A/B/C 使用独立后处理入口读取同一份放置快照，便于在同一划分上做消融。**后续搜索的正式接入方向已修订**：不是另写一套“只做正收益 Move”的替代流程，而是在逐层反粗化后，用需求和线图规则决定候选激活顺序，再交给原细化器的堆选择动作；见第 15 节。当前模块 1 只做读取/统计，不运行任何版本。

## 9. 源码语义冲突与必须显式处理的问题

以下不是猜测，均对应到现有源码；在实现前必须由用户确认取哪一种语义。

1. **`target_pins_in_part` 与伪代码 \(c_{e,b}\) 不同**
   - `Hypergraph::initPinsInPart`（`hypergraph.h:358-383`）对每个 sink 的主位置和副本位置计数，但排除 source 副本板；重复 pin 会重复计数。
   - `KWayFMRefiner::moveHypernode`（`kway_fm_refiner.h:135-173`）按同样语义增量维护。
   - 伪代码 \(c_{e,b}\) 只统计 `P(t)` 的 sink 主位置，且是集合计数。
   - 有副本或重复 pin 时，直接复用缓存会改变 \(d_e\)、中心选择和版本结果。
   - 若采用严格伪代码语义，需要从 pins 独立计数，不能把 `target_pins_in_part` 当作 \(c_{e,b}\)。

2. **`hopDistanceCost` 不适合做候选合法性/收益**
   - `refiner_base.h:106-120`：`getHopDistance` 返回 -1 时，`if (a < 0) {}` 为空，随后仍执行 `hop_distance += a`，会把不可达负值计入。
   - 候选 Gain 应使用 `refinement_native_probe::netCost`，它遇到不可达 hop 直接抛异常；已有先例 `unified_native_heap_adapter.cpp:75-76`。

3. **`moveIsFeasible` 不检查通信容量、hop 和副本重叠**
   - `refiner_base.h:92-100` 只检查 8 维资源。
   - `moveHypernode` 不主动拒绝目标板已有该节点副本的情况。
   - 输出合法划分必须额外：
     - `evaluateFPGACommChange` + 逐板容量比较；
     - `netCost` 的 hop 可达性；
     - `to ∉ replication_fpga_labels[u]`。
   - 先例：`tools/unified_native_heap_adapter.cpp:71-93`。

4. **现有 Gain 键不是任意 (u,b) 的查询接口**
   - `KWayFMRefiner::rateOneNodeToAllParts`（`kway_fm_refiner.h:347-474`）只对激活的边界 Net 生成 `tmp_ratings`，且只把正评分放入 `kpq`。
   - 既有探针核对的是“原生键 = 完整 WTHD 差”（`refinement_native_move_probe.h:44-78`），但这不等于它能为任意候选板给出完整查询。
   - 因此本方案建议用 `netCost` 直接算完整 Gain；若要严格复用现有 Gain 接口，需要另行设计激活状态和查询包装，不能假设 `kpq` 中一定有候选。

5. **重复 pin 的集合语义**
   - `design.net` 允许同一 sink 名重复；`read_in` 不去重，`target_pins_in_part` 按出现次数计数。
   - 伪代码 \(c_{e,b}\)、\(\operatorname{sinks}(e)\)、\(e\cap f\) 都是集合语义。
   - 第一版建议：\(d_e\)、跨板 Sink 数、\(A_{ef}\) 均按去重后的节点身份计算；若用户要按 pin 出现次数，必须明确，否则结果不同。

6. **副本语义**
   - 伪代码只写了 \(P(v)\)、\(P(s_e)\)，没有写 `replication_fpga_labels`。
   - 现有 V2 目标函数和 Move 都包含副本。
   - 可能的三种选择：
     - **严格伪代码**：\(d_e\) 只看 sink 主位置和 source 主位置；副本不参与 \(c,d\) 和候选板。
     - **目标一致**：\(c_{e,b}\) 用 V2 目的板语义（含 sink 副本、排除 source 副本），\(d_e\) 是正代价目的板数。
     - **限制输入**：只在无副本的节点/状态上运行。
   - 选择不同会改变中心、邻域、候选和最终结果；不能静默选。

7. **程序自报 Best Cost ≠ V2 WTHD**
   - 已在 2026-11-17 服务器实验确认含副本时二者可能不同。
   - 本方案所有“最终目标值/实际 Gain”必须用 `refinement_native_probe::cost`；报告中不得混用。

8. **`initPinsInPart` 不清零**
   - `Hypergraph::initPinsInPart` 是增量累加，不是重置；`FMRefinerBase::initPinsInPart` 同样不清零且忽略副本（`refiner_base.h:36-45`）。
   - 独立工具必须在新构造的图或显式清零后调用一次；不能对已初始化的状态重复调用。

9. **版本命名冲突**
   - 项目已有“A 基线”“H 社区”“S 评分”。本方案 A/B/C 建议在实验目录和报告中写 **NC-A/NC-B/NC-C**，避免与基线 A 混淆。

## 10. 已确认决策（2026-11-17）

| 编号 | 确认结果 |
|---|---|
| D1 | 使用独立后处理工具；NC-A/B/C 读取同一份放置快照；先不嵌入 `partitioner.cpp`。后续搜索接入路线另见第 15 节修订。 |
| D2 | \(c_{e,b}\)、\(d_e\) 按 **sink 主位置** 计算；另记含副本的 V2 目的板统计作诊断。需求评分与最终目标允许采用不同表达，但字段必须明确区分。 |
| D3 | 候选目标板从当前所检查 Net 的逻辑节点 **主位置** 生成；排除自身主位置，以及该待移动节点的副本板。 |
| D4 | 保留 **8 维资源、通信容量、hop 可达性、副本板冲突** 检查；检查移动后的状态。 |
| D5 | 完整 Gain 使用 `netCost` 对受影响的全部关联 Net 计算前后差；每条 Net 只累计一次。整体目标统一用 `refinement_native_probe::cost`，不依赖 `kpq` 是否存在候选键。 |
| D6 | 中心按 \(d_e\) 降序、Net ID 升序；邻居按 \(S\) 降序、\(d_e\) 降序、Net ID 升序；Move 按 Gain 降序、节点 ID 升序、目标板 ID 升序。 |
| D7 | 预算计数固定为：一次合法候选 \((u,b)\) 的完整 Gain 求值记一次；缓存命中不消耗预算但单独记录；合法性拒绝、实际 `netCost` 调用次数和总运行时间分别统计。**固定 \(10^6\) 默认值已撤回**，预算待测量后确定，见第 16 节。 |
| D8 | NC 结构与需求统计按 **逻辑节点 ID 去重**：同一节点在同一 Net 中只贡献一次 sink 计数或关联。原生 V2 成本、通信状态继续沿用原生语义，不改底层 pin 数组。 |
| D9 | 第一版只处理 **一份明确指定的最终细粒度放置快照**；若项目称其为 layer 0，就只处理 layer 0；不自动遍历每个 exploration。 |
| D10 | 中心单独标记 `is_center=true`，始终先检查；自关联不参与评分与归一化。日志中中心关联值记为 `N/A`，不人为设成最大值。 |

**后续修订（同日晚些时候）**：
- \(10^6\) 是候选 Gain 计算次数上限，不是必须计算这么多次；该固定默认值已撤回，先测平均涉及 Net 数、`netCost` 调用次数和耗时，再定预算。
- 搜索接入改为“逐层反粗化 + 需求/线图引导候选激活 + 原有堆选择动作”；独立工具仍用于同起点分析和模块 1 验证，但不是替代原细化流程的最终方案。见第 15 节。
- 完整 Gain 不等于全图重算：维护每条 Net 当前成本 \(C_e(P)\)，只算 \(I(u)\) 中 Net 的移动后成本；用板上计数跳过目标板集合不变的项；候选 Gain 只有在相关 Net 状态变化时失效。见第 16 节。

## 11. 模块 1 已新增文件与后续模块

### 11.1 模块 1（已创建，2026-11-17）

| 文件 | 职责 |
|---|---|
| `tools/net_centered_module1.cpp` | C++ 读取、原生状态重建、NC 需求统计；不执行 Move |
| `tools/net_centered_reference.py` | 独立 Python 参考实现，不调用 RePart C++；从输入/快照独立重建并计算 NC/V2 统计 |
| `tools/net_centered_module1_run.py` | 运行 C++、参考实现和既有独立 evaluator，比较并写报告 |
| `tools/net_centered_comm_variants.py` | 通信口径诊断，仅用于定位快照缓存差异，不参与算法 |
| `runs/net_centered_module1_20261117/` | case01、case02 快照、case03 快照、合成 fixture 的输出与报告 |

### 11.2 后续模块（尚未创建）

| 模块 | 位置（拟） | 职责 | 状态 |
|---|---|---|---|
| 候选激活接口 | `RePart/refinement_move/` 或独立头文件 | 在逐层反粗化后，把需求/线图选出的候选节点、目标板或 Net 先激活到原细化器批次 | 待实现 |
| 需求与邻域 | `RePart/net_centered_refinement/demand.h` | \(c_{e,b}\)、\(d_e\)、跨板 Sink 数、`net_degree`、\(A_{e^*f}\)、\(S(f\mid e^*)\) | 模块 1 已覆盖需求统计；邻域部分待实现 |
| 增量 Gain | `RePart/net_centered_refinement/incremental_gain.h` | 缓存 \(C_e(P)\)，只算 \(I(u)\)，用板上计数跳过零变化项，候选 Gain 按依赖失效 | 待实现 |
| 主循环/接线 | 原细化器调用点 | 需求引导候选激活，仍由原堆按 Gain 选择 Move/Exchange/Replication/Deletion | 待实现 |
| 输出 | 原运行入口 + 日志 | 动作记录、中心/候选 Net、\(d_f\)、关联度、Gain、计数器 | 待实现 |

工程决定（可自行处理但会报告）：整数类型、`std::set` 作为最大堆、epoch 代替整表清零、`double` 累加顺序、输出精度、错误信息、文件组织。任何会改变算法结果的决定必须先确认。

## 12. 验证计划（实现后，逐模块进行）

1. **微型图单测（无副本）**
   - 构造 3 板、1 条源 + 3 个 sink 的 Net，核对 \(c_{e,b}\)、\(d_e\)、跨板 Sink 数。
   - 构造共享节点，核对 \(A_{ef}\) 与 \(\deg(v)\)；\(\deg(v)=1\) 时不产生邻居。
   - 核对 \(S\) 的两个归一化分母；无邻居时第二项不计算。
2. **微型图单测（有副本 / 重复 pin）**
   - 分别用严格伪代码语义和 V2 语义计算，确认差异被显式记录，不把两者混用。
   - 验证主位置/副本重叠的候选被拒绝。
3. **状态装载一致性**
   - 从已有 `REFINEMENT_SNAPSHOT_DIR` 快照读取同一点，比较重建的 `usage`、`comm`、`target_pins`、V2 代价与快照值；不一致则停止，不进入 Move。
4. **Gain 一致性**
   - 每个接受 Move 前后用 `refinement_native_probe::cost` 全量重算；断言 `before - after == gain`。
   - 首次实现可对每个候选也做全量核验，之后只对接受动作核验。
5. **合法性**
   - 每个接受 Move 后检查 8 维资源、通信容量、全部 Net 的 hop 可达、主/副本不重叠；全部通过才写入输出。
6. **增量 \(d_e\) 一致性**
   - 每个接受 Move 后，用全图重算所有 Net 的 \(d_e\) 与跨板 Sink 数，和增量维护值逐 Net 比较。
7. **可复现性**
   - 同一输入、同一参数重复运行，`actions.tsv` 与输出放置逐字节一致；浮点排序使用固定累加顺序和明确平局规则。
8. **冒烟顺序**
   - 先在 case01 跑通装载和 NC-A；再 case02；确认 A/B/C 能共用同一输入放置。暂不在 case04/09/10 大图上直接开始。

## 13. 实施顺序与停止点

按用户指定顺序，并遵守“一次只实现一个最小模块、每步停下确认”：

1. **模块 1：装载与只读统计 — 已完成（2026-11-17）**
   - 读入 case + 放置；重建并核验资源/通信/目标板缓存；计算 \(d_e\)、跨板 Sink 数、`net_degree`。
   - 不执行任何 Move；结果见第 17 节。
2. **模块 2：逐层反粗化 + 候选激活接口（路线已修订）**
   - 先只做“反粗化一层后，按需求/线图选出候选并激活”的接口，不改变原 Move/Exchange/Replication/Deletion 语义。
   - 验证：候选激活只影响检查顺序/批次，不改变合法性；原堆仍按 Gain 选择动作。
3. **模块 3：NC-A/NC-B 的引导模式**
   - NC-A：只激活中心 Net 的候选。
   - NC-B：中心 + 一跳邻居，\(\alpha=0\)。
   - 两者都在原细化器内，不另写替代搜索循环。
4. **模块 4：NC-C 与增量 Gain**
   - 只增加 \(\alpha\) 第二项；同时接入第 16 节的 Net 成本缓存、零变化跳过和候选 Gain 失效。
   - 先测每候选平均涉及 Net 数、`netCost` 调用次数和耗时，再确定预算。
5. **模块 5：实验脚本与报告**
   - 同一输入放置比较 A/H/S 原流程与 NC 引导版本；记录最终 V2、接受动作数、Gain 计算次数、墙钟。
   - 路由器接通后再比较 TDM/延时；本轮不做该比较。

## 14. 模块 1 范围与未做事项

- 模块 1 只做读取、状态重建和需求统计；未执行任何 Move、Exchange、Replication 或 Delete 搜索。
- 未修改 `RePart/` 下任何 `.h/.cpp`，未修改 `partitioner.cpp`，未修改 `clean/`，未运行 `partitioner` 或新分配实验。
- 未修改任何输入放置；case01 使用已保存的 `design.fpga.out`，快照验证使用已有 `REFINEMENT_SNAPSHOT` 文件。
- 根目录与 `docs/` 的 plans 副本整理未在本轮处理。
- 模块 1 完成后按 `AGENTS.md` 报告状态核对、语义差异、阻塞和最大 \(d_e\) Net；见第 17 节。

## 15. 后续搜索接入路线修订（2026-11-17）

用户进一步确认：目标不是另写一套“独立工具、只做正收益单节点 Move”的替代细化流程，而是把需求与线图用于**原细化器的候选激活环节**。建议的接入顺序：

1. **反粗化一层**：拆开粗节点，继承当前板分配。
2. **需求与线图引导**：确定优先检查的 Net、相关逻辑节点及移动候选。
3. **计算候选 Gain，进入原有堆**：仍由 Gain 比较具体动作。
4. **执行并增量更新**：沿用原合法性检查、动作执行和受影响收益更新，同时更新需求评分。

关键约束（必须显式处理）：

- 如果所有候选照旧全部入堆、仍按相同 Gain 选最大值，仅改变计算先后顺序通常不会改变结果。因此引导必须实际影响**哪些候选先被激活或进入当前搜索批次**，再由原堆从这一批中选择。
- 原目标函数、8 维资源、通信容量、hop、副本以及 Move/Exchange/Replication/Deletion 语义尽量保留；需求评分只组织候选，不替代 Gain。
- 模块 1 的独立工具仍保留为同起点分析和状态核验入口，不作为最终在线算法入口。

## 16. 增量 Gain 与预算（2026-11-17）

完整 Gain 不需要每次全图重算。\(u:a\to b\) 只影响 \(I(u)=\mathcal N(u)\)：

\[
G(u:a\to b)=\sum_{e\in I(u)}\left[C_e(P)-C_e(P^{u\to b})\right].
\]

分三层实现：

1. **缓存 Net 当前成本**：维护每条 Net 的 \(C_e(P)\)；候选评估只计算 \(I(u)\) 中各 Net 的移动后成本，不扫描其他 Net，不复制整个超图。“完整”指没有漏掉受影响的 Net。
2. **板上计数跳过零变化**：
   - 无副本、移动汇点时，若原板 \(a\) 上仍有其他汇点，且目标板 \(b\) 上已有汇点，则目标板集合不变，成本变化为 0，可直接跳过。
   - 原板最后一个汇点离开、目标板首次出现汇点、移动源点、或有副本时，按 V2 的源覆盖/目标覆盖规则判断哪些贡献改变。
   - 通用 `netCost` 保留为正确性核验依据；后续可替换为严格符合 V2 的差量公式。
3. **候选 Gain 缓存与失效**：
   - 若决定该收益的 Net 状态没有变化，缓存继续有效；相关 Net 变化才重算。
   - 即使收益不变，目标板资源和通信容量可能变化，合法性仍需单独更新或重新检查；不能只凭“两个节点不相邻”认定缓存有效。

**预算状态**：原先固定的 \(10^6\) 次候选 Gain 上限已撤回。后续先统计每次候选平均涉及多少 Net、实际调用多少次 `netCost`、耗时多少，再确定预算。模块 1 不使用该预算。

## 17. 模块 1 实现与核验结果（2026-11-17）

### 17.1 实现

| 文件 | 说明 |
|---|---|
| `tools/net_centered_module1.cpp` | C++ 被测实现：读取 case+placement 或 snapshot；重建 label/副本/8 维资源/通信/target pins；计算 strict \(c_{e,b}\)、\(d_e\)、跨板 Sink 数、`net_degree`、V2 目的板与 `netCost`；输出 TSV/JSON |
| `tools/net_centered_reference.py` | 独立 Python 参考实现；不调用 C++；placement 模式复用既有 `replication_thd_validator.py` 的解析与 V2 评估，snapshot 模式自行解析 |
| `tools/net_centered_module1_run.py` | 运行 C++、参考实现、既有 `evaluate_partition_wthd.py`，比较并写 `identity.json`、`comparison.json`、`verification.json`、`REPORT.md` |
| `tools/net_centered_comm_variants.py` | 通信口径诊断；只用于定位快照缓存差异 |

编译命令（WSL g++ 13.3）：

```bash
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra tools/net_centered_module1.cpp \
  -o runs/net_centered_module1_20261117/bin/net_centered_module1
```

`-DNDEBUG` 是必要的：现有 `hypergraph.h`/`kway_fm_refiner.h` 的断言宏在非 NDEBUG 下会引用项目未引入的 `kahypar::Logger`。编译保留了现有头文件的警告，没有给项目加 `-Werror`。

### 17.2 case01（placement）

- 输入：`testcase/case01`，放置 `runs/assignment_first_complete_more_20261117/case01/A_search/design.fpga.out`。
- 规模：16 节点、13 Net、4 板；无副本、无重复 pin。
- 初始 V2 WTHD = 11；\(\sum d_e=10\)，跨板 Net 9 条，\(\max d_e=2\)。
- 状态核对：placement、8 维资源、通信容量、hop、`netCost`、`initPinsInPart` 与独立重建全部 PASS。
- 交叉核对：C++ vs Python 13/13 Net、16/16 节点、4/4 板、52/52 target-pin 行全部一致；既有 `evaluate_partition_wthd.py` 给出 WTHD=11、legal=true，与 C++ 一致。
- 最大 \(d_e\)：net 4（source board 1，\(d_e=2\)，跨板 Sink 3，V2 目的板 2，V2 cost 2）；其余最高为 1。
- 报告：`runs/net_centered_module1_20261117/case01/REPORT.md`。

### 17.3 case02 layer0 快照

- 输入：`runs/exchange_action_pairs_finer_20261115/baseline/snapshots/snapshot_11_layer_0.txt`，case 目录 `testcase/case02`（输入哈希与快照来源一致）。
- 规模：600 节点、1239 Net、8 板；7 个副本节点、13 个副本位置；无重复 pin。
- 初始 V2 WTHD = 2433；\(\sum d_e=1516\)，跨板 Net 980 条，\(\max d_e=6\)。
- 状态核对：placement、8 维资源、通信、hop、`netCost`、`initPinsInPart` 全部 PASS；快照 USAGE、COMM、source board、Net 结构全部 PASS；快照 META cost 2433 = V2 cost。
- 交叉核对：C++ vs Python 全部一致；语义差异 121 行，均来自副本/源副本覆盖，符合 D2/D3 的 strict 主位置定义。
- 最大 \(d_e\)：net 397、619、755、1134 均为 6；例如 net 397（source board 7，跨板 Sink 69，V2 目的板 3，V2 cost 5）。
- 报告：`runs/net_centered_module1_20261117/case02_snapshot/REPORT.md`。

### 17.4 case03 layer0 快照（附加已知差异检查）

- 输入：`runs/refinement_snapshot_20261111/case03/snapshot_11_layer_0.txt`。
- 规模：11451 节点、31071 Net、32 板；468 个副本节点、2447 个副本位置。
- 初始 V2 WTHD = 6674；\(\sum d_e=6475\)，跨板 Net 4148 条，\(\max d_e=27\)。
- 硬状态核对：placement、资源、通信容量、hop、`netCost`、`initPinsInPart` 全部 PASS；快照 USAGE、source board、Net 结构、META cost 全部 PASS。
- **已定位的差异**：快照通信缓存与独立重建在 4 块板上不一致：board 11（306 vs 301）、15（623 vs 621）、23（295 vs 293）、27（615 vs 610）。C++ 与 Python 参考实现互相一致，并且与既有 `runs/refinement_snapshot_20261111/case03/verification.json` 中 `rebuilt_communication` 的数值一致；差异只在快照保存的原生增量通信缓存。
- 既有报告已记录首个差异出现在 `snapshot_2_layer_3`（FPGA15 666 vs 664、FPGA23 322 vs 320），之后在更细层持续存在。仅凭快照无法定位到具体动作；需要给原细化器的通信更新加动作级日志才能进一步归因，本轮未做。
- 这不是模块 1 的重建失败，也没有被“看起来一致”覆盖；模块 1 报告保留该差异并标为已知原生缓存问题。
- 最大 \(d_e\)：net 248、734、1247、1524、2152 等均为 27；例如 net 248（source board 5，跨板 Sink 84，V2 目的板 2，V2 cost 3）。
- 报告：`runs/net_centered_module1_20261117/case03_snapshot/REPORT.md`。

### 17.5 合成 fixture：副本与重复 pin

- 输入：`runs/net_centered_module1_20261117/fixture_replica_dup/`（6 节点、4 Net、3 板）。
- 预期与实际一致（`fixture_expected` PASS）：
  - net 0：`a -> b c c`，sink c 重复一次且 c 有副本。strict \(c\)：board0=1、board1=1；\(d_e=1\)；V2 目的板 {1,2}，V2 目标数 2；native target pins `0:1;1:2;2:2`。说明 NC 去重后 \(c\) 与原生按出现次数/副本的缓存不同。
  - net 3：source f 有副本 board2，sink e 在 board2。strict \(d_e=1\)；V2 目的板为空（源副本覆盖），V2 目标数 0。说明 strict 主位置需求与 V2 覆盖语义不同。
- 总 V2 WTHD=5；\(\sum d_e=4\)；副本节点 2、副本位置 2；重复 pin Net 1。
- 报告：`runs/net_centered_module1_20261117/fixture_replica_dup/REPORT.md`。

### 17.6 独立基线说明

模块 1 不是用 C++ 自己的重建结果验证自己，使用了三层对照：

1. **独立 Python 参考实现** `tools/net_centered_reference.py`：不调用 C++，独立解析输入/快照并计算 NC/V2 统计。
2. **既有独立 evaluator** `tools/evaluate_partition_wthd.py`：对 placement 模式独立计算 V2 WTHD、资源/通信/hop 合法性。
3. **已有原生快照缓存**：对 snapshot 模式比较 USAGE/COMM/META/source board/Net 结构；差异保留并定位。

### 17.7 阻塞与结论

- 模块 1 范围内无阻塞。
- 已知原生通信缓存差异（case03 快照）不属于模块 1 的重建错误，未修复、未覆盖；后续若要修复，应单独给原细化器的通信增量更新加动作级诊断。
- 未执行 Move 搜索，未改变输入放置，未开展 case04 服务器实验。

## 18. 当前状态与下一步

- 模块 1 已完成并核验；D1–D10 已写回；固定 \(10^6\) 预算已撤回。
- 下一步不是直接写独立 Move 搜索，而是按第 15 节设计并实现“逐层反粗化 + 需求/线图候选激活”的最小接口；实现前先报告接口位置、激活批次定义和与原堆的交互，等待确认。
- 增量 Gain 的 Net 成本缓存、板上计数跳过和候选失效按第 16 节单独实现与核验；预算在测量后确定。

## 19. Move 通信增量修复（2026-11-18）

诊断已确认：layer3 入口缓存与全量重算一致，首个错误发生在 layer3 Move 动作的通信增量更新。按用户确认，修复直接采用已验证的全量通信定义，不再依赖 `target_pins_in_part` 的旧增量分支。

### 19.1 公式与源码映射

\[
C_{e,b}(P): \text{Net } e \text{ 在板 } b \text{ 的通信贡献，含 source/sink 副本规则}
\]

\[
\Delta\mathrm{COMM}_b=\sum_{e\in\mathcal N(u)}
\bigl[C_{e,b}(P')-C_{e,b}(P)\bigr],
\quad P'=P\circ(u:a\to b)
\]

| 伪代码对象 | 数学含义 | C++ 实现 | 源码位置 |
|---|---|---|---|
| \(C_{e,b}(P)\) | 单 Net 单板贡献 | `exact_move_comm::net_contribution(g,e)` | `RePart/refinement_move/exact_move_comm.h` |
| \(C_{e,b}(P')\) | u 投影到 b 后的贡献 | `net_contribution(g,e,u,b)` | 同上 |
| \(\mathcal N(u)\) | u 关联的不同 Net | `incident_nets_as_source` ∪ `incident_nets_as_target`，去重 | 同上 |
| \(\Delta\mathrm{COMM}_b\) | 受影响的逐板增量 | `exact_move_comm::move_delta(g,u,b)` | 同上 |
| 容量检查 | `COMM_b + delta_b <= capacity_b`，且非负 | `exact_move_comm::feasible` | 同上 |
| 接受后写回 | `COMM_b += delta_b` | `exact_move_comm::apply` | 同上 |
| Move 接入 | 替换 Move 的旧通信检查 | `KWayFMRefiner::checkAndChangeCommExact` / `applyCommDelta` | `RePart/refinement_move/kway_fm_refiner.h` |
| 开关 | 默认 exact；旧入口保留 | 默认走 exact；`REPART_LEGACY_MOVE_COMM` 选择旧路径 | `kway_fm_refiner.h` |

### 19.2 约束与边界

- 只重算 \(u\) 关联的不同 Net；其他 Net 的放置和贡献不变。
- 每个 Net 的贡献按“source 主位置 + source 副本、sink 主位置 + sink 副本、目标板去重”的已验证定义计算；不读取 `target_pins_in_part`。
- 候选 Move 用 `当前 COMM + 正确 delta` 检查容量；接受后写回同一份 delta。
- 旧 `evaluateFPGACommChange` 保留，不删除；用于旧快速路径或后续根因对照。
- 只修 Move；Exchange/Replication/Delete 的通信路径不在本次改动内。
- 初始 COMM 必须来自已核对状态；如果入口本身不一致，修复不会自动修正它。
- 2026-11-18 用户确认后，精确路径已设为默认；原入口保留，使用 `-DREPART_LEGACY_MOVE_COMM` 可恢复旧 `evaluateFPGACommChange/checkAndChangeComm`。
- 默认构建和 legacy 构建均已在 case01 冒烟通过（WTHD=11）。

### 19.3 验证口径

- 用 `REPART_COMM_PROBE` 的全量重算逐板对照，不依赖目标函数或程序自报值。
- 检查 layer4 结束、layer3 入口、layer3 出口及 Move 动作前后；修复后 `comm_probe.tsv` 的 diff 应为 0。
- 若其他动作类型仍引入差异，单独记录，不归因于 Move 修复。
- case03 仅用于机制验证；在 COMM 修复确认前不作为 NC 效果结论。

### 19.4 验证结果（2026-11-18）

- 代码已加入 `exact_move_comm.h` 和宏保护接入；运行见 `runs/net_centered_exact_move_20261118/`。
- 全动作、全 layer 逐板对照：`comm_probe_action_checks.tsv` 共 15,100 条实际动作，`delta_match=1` 为 15,100/15,100；其中 Move 2,536、Exchange 340、Replication 11,216、Delete 1,008。
- `comm_probe.tsv` 的 60/60 个 layer 入口/出口检查 `diff_boards=0`。
- 原错误动作节点 4822 `board15→23` 在该运行中出现 5 次（exploration 0/1/2/3/4 的不同层），全部 `delta_match=1`；单元回放给出 board15 −1、board23 0，与全量重算一致。
- 最终放置独立核验：`wthd=6664`，`legal=true`，资源/通信/hop 违规均为 0。
- 结论：Move 精确增量在本次运行中逐动作、逐板通过；其他动作类型在本次全动作检查中也未出现不匹配。用户确认后，精确路径已设为默认；旧入口保留在 `-DREPART_LEGACY_MOVE_COMM` 下。不把 6664 与旧运行直接比较作质量结论。

## 20. NC 首批候选激活（2026-11-18）

按用户确认的批次语义实现：

- 每层 `initPinsInPart` 之后，先确定首批候选；
- 用原有 Gain 堆跑一次受限 Move；
- 首批结束后销毁该 refiner/堆/临时状态，基于当前划分重建无过滤候选；
- 再按原顺序执行 Exchange → Move → Delete → Replication。

| 版本 | 首批允许入堆的节点 |
|---|---|
| NC-A | 最大 \(d_e\) 的中心 Net 所含逻辑节点 |
| NC-B | 中心 Net 及其一跳邻居（\(A_{ef}>0\)）所含逻辑节点，按节点去重，\(\alpha=0\) |

实现：

| 组件 | 位置 | 作用 |
|---|---|---|
| 需求/中心/邻域 | `RePart/net_centered/activation.h` | `net_demand`、`center_net`、`build`、日志 |
| 入堆过滤器 | `RePart/refinement_move/kway_fm_refiner.h` | `ActivationFilter`；`rateAllActivatedHypernodes` 与 `rateOneNodeToAllParts` 都检查 |
| 首批接入 | `RePart/partitioner.cpp` | 每层 `initPinsInPart` 后运行受限 `Refiner`，然后进入原流程 |
| 开关 | 编译 `-DREPART_NC_ACTIVATION`；运行 `REPART_NC_MODE=off/A/B` | 默认构建不受影响 |
| 日志 | stdout + `REPART_NC_LOG` 指向的 TSV | 首批节点数、实际入堆节点数、接受 Move 数、gain、批耗时 |

约束：

- 过滤只控制谁能入堆；Gain 仍覆盖该 Move 影响的全部 Net；
- 通信容量检查走默认精确 Move 增量；
- 初始入堆和增量入堆共用同一 `ActivationFilter`；
- 首批后原流程不加过滤，顺序不变。

验证（case01/case02，2026-11-18）：

- case01：off/A/B 最终 WTHD 均 11；批次 inserted/accepted 均为 0（该例没有正收益首批 Move）。
- case02：off/A/B 最终 WTHD 均 2394，独立 evaluator 均 legal；墙钟约 951/1079/1037 ms。
  - NC-A：layer1 首批 batch=37、inserted=1、accepted=1、gain=1；其余层 0。
  - NC-B：layer2 batch=150、inserted=2、accepted=2、gain=4；layer1 batch=300、inserted=1、accepted=1、gain=1；layer0 batch=599、inserted=1、accepted=1、gain=5。
- 这些结果只验证过滤器生效、首批确有动作、额外耗时；最终 WTHD 相同，不能据此声称 NC 质量收益。

### 20.1 更大案例 off/A/B（2026-11-19）

单次运行，全部输出经独立 evaluator 核验合法（资源/通信/hop 违规 0）：

| case | off V2 | NC-A V2 | NC-B V2 | 墙钟 off/A/B (s) |
|---|---:|---:|---:|---|
| case03 | 6702 | 6348 | 6581 | 184.52 / 190.88 / 166.57 |
| case07 | 5807 | 5663 | 5869 | 66.33 / 62.81 / 63.09 |
| case08 | 3817 | 3752 | 3608 | 60.64 / 58.62 / 60.97 |

首批统计：

| case | mode | inserted 合计 | accepted 合计 | gain 合计 | 最大 batch | 批耗时合计 (s) |
|---|---|---:|---:|---:|---:|---:|
| case03 | A | 20 | 20 | 1825 | 89 | 0.295 |
| case03 | B | 2693 | 2392 | 10101 | 9095 | 1.740 |
| case07 | A | 12 | 12 | 484 | 75 | 0.087 |
| case07 | B | 337 | 281 | 1059 | 5059 | 0.436 |
| case08 | A | 5 | 5 | 44 | 96 | 0.072 |
| case08 | B | 211 | 208 | 1295 | 3799 | 0.330 |

观察：

- NC-A 在这三次单次运行中最终 WTHD 都低于 off；NC-B 在 case03/case08 低于 off，在 case07 高于 off（+62）。
- 首批搜索本身的耗时很小（0.07–1.74 s 合计），总墙钟差异主要来自后续轨迹和搜索，而不是首批计算量。
- **单次运行，不能据此声称稳定收益或因果结论**；assignment/refinement 存在线程/预算波动，且没有做同起点或多次重复。NC-B 的大批次是否稳定优于 NC-A 尚未确认。
- 下一步：对这三个案例重复 off/A/B，或先用同一初始划分做同起点对照，分离激活范围与搜索噪声；再决定是否调整 NC-B 的邻域范围。
