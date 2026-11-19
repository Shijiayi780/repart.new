# NC-FB-v1

Net-Centered First Batch, version 1.

Base: `main` at `6c288a14` (clean workspace + EDA testcases + Titan23 benchmarks).

## 包含的改进

1. **精确 Move 通信增量（默认）**
   - 按已核对的通信定义计算受 Move 影响的 Net 的前后逐板贡献差；
   - 候选用 `COMM + delta` 检查容量，接受后写回同一 delta；
   - 旧快速入口保留：编译时加 `-DREPART_LEGACY_MOVE_COMM`。

2. **NC 首批候选激活（可选编译开关）**
   - 每层 `initPinsInPart` 后先跑一次受限 Move；
   - NC-A：最大 \(d_e\) 中心 Net 的逻辑节点入堆；
   - NC-B：中心 Net + 一跳邻居 Net 的逻辑节点入堆，去重，\(\alpha=0\)；
   - 过滤同时作用于初始入堆和增量入堆；
   - 首批结束后销毁该批堆，再按原顺序走 Exchange → Move → Delete → Replication；
   - Gain、资源/通信约束、原流程顺序不变。

3. **观察式诊断**
   - `REPART_COMM_PROBE` 记录层入口/出口和逐动作通信缓存与全量重算的对照；
   - 只读，不修改缓存或候选顺序。

## 编译与运行

默认构建（精确 Move，NC 关闭）：

```bash
g++ -std=c++17 -Ofast -DNDEBUG -I boost_1_86_0/include \
  RePart/partitioner.cpp -L boost_1_86_0/lib \
  -static -lboost_thread -lboost_system -pthread -o partitioner
```

NC 实验构建：

```bash
g++ -std=c++17 -Ofast -DNDEBUG -DREPART_NC_ACTIVATION \
  -I boost_1_86_0/include RePart/partitioner.cpp \
  -L boost_1_86_0/lib -static -lboost_thread -lboost_system -pthread \
  -o partitioner_nc
```

运行模式：

```bash
REPART_NC_MODE=off partitioner_nc -t testcase/case02 -s out.off
REPART_NC_MODE=A   partitioner_nc -t testcase/case02 -s out.A
REPART_NC_MODE=B   partitioner_nc -t testcase/case02 -s out.B
REPART_NC_LOG=nc.tsv REPART_NC_MODE=B partitioner_nc ...
```

旧 Move 通信路径：

```bash
-DREPART_LEGACY_MOVE_COMM
```

## 验证摘要

- 精确 Move 全动作逐板对照：15,100/15,100 个实际动作匹配；60/60 个 layer 入口/出口 diff=0。
- case03/07/08 单次 off/A/B（V2 WTHD）：

| case | off | NC-A | NC-B |
|---|---:|---:|---:|
| case03 | 6702 | 6348 | 6581 |
| case07 | 5807 | 5663 | 5869 |
| case08 | 3817 | 3752 | 3608 |

- 全部输出资源/通信/hop 合法。
- 首批统计和日志位于 `runs/net_centered_nc_20261118/`。

## 明确限制

- NC-A/NC-B 为**单次运行**，没有同起点、没有重复；不能声称稳定收益或因果结论。
- `t5_tdm_route` 是本地 TDM 代理，不是 HoPart 原生路由器，也不是正式物理路由器。
- HoPart 原生 router 未接入：当前缺 `libgurobi120.so`、7 个头文件和固定划分注入接口。
- 本版本只覆盖分区/细化改进和评价工具；不改变 RePart 的目标函数、资源约束或 Net 权重。

## 关键文件

- `RePart/refinement_move/exact_move_comm.h`
- `RePart/net_centered/activation.h`
- `RePart/refinement_comm_probe.h`
- `RePart/refinement_move/kway_fm_refiner.h`
- `RePart/partitioner.cpp`
- `tools/net_centered_module1.cpp`
- `tools/net_centered_reference.py`
- `tools/exact_move_comm_tests.cpp`
- `plans/NET_CENTERED_REFINEMENT_MAPPING.md`
