# 调度策略：兜底分区 + 时间预算

> 负责人：D（调度）  代码：`nodes/task_rules.py`（纯逻辑）、`nodes/runtask.py`（接入）
> 配置：`config/items_config.yaml` 的 `task` 段
> 最后更新：2026-11-23

## 1. 为什么改

两个场景会直接丢分：

| 场景 | 原来的行为 | 后果 |
| --- | --- | --- |
| 分类板没识别出来（ROI 未标/板子没摆正） | 所有物品都落到 `board_fallback_zone`（默认 B 区） | 10 块金砖全堆 B 区：既大概率不得分，还把 B 区堆满、挡住后续正确放置 |
| 10 分钟到了还在抓 | 一直抓到 10 块为止 | 规则「超时放置的金砖不得分」，且超时每秒扣 2 分 |

## 2. 兜底分区（`choose_drop_zone`）

放置区判定优先级：

```
蛋糕(ID=5) ──────────────► Collection_A（规则硬性要求）
   └─ 分类板映射里有这个物品 ─► 用映射结果
        └─ 按物品的兜底表 ─────► task.fallback_by_name
             └─ 全局兜底 ─────► board_fallback_zone（默认 B）
```

配置：

```yaml
task:
  fallback_by_name:
    bear: Collection_B
    wine: Collection_C
    clock: Collection_C
    banana: Collection_D
```

效果：即使分类板完全没识别出来，10 块金砖也会**分散到 B/C/D**，不会全塞一个区；
每类仍有 1/3 概率正好落对，且不会因为堆满一区而影响后续放置。

日志会在每次走兜底时打一条 warn：

```
[WARN] 分类板映射里没有物品 4，使用兜底区 Collection_D（本场分类板可能没识别出来）
```

## 3. 时间预算（`should_stop`）

```yaml
task:
  time_budget_s: 540     # 9 分钟（规则 10 分钟，留 1 分钟收尾）
  min_remaining_s: 90    # 剩余不足 90s 就停止“新的”抓取
```

`runtask.run_task()` 开始时记录 `task_deadline = now + time_budget_s`，
每轮循环开头判断：剩余时间 < `min_remaining_s` 就 `break`，收臂、结束任务。

一轮「导航 → 抓取 → 放置」大约 60~90s，所以预留 90s 保证最后一块能放完，
不会出现「抓在手上、时间到了、金砖没落地」的情况（那种情况按规则不得分）。

日志：

```
[INFO] 时间预算 540s，截止时刻约 1795406xxx（剩余不足 90s 时停止抓取）
[WARN] 时间预算将尽（剩余 78s < 预留 90s），停止继续抓取，共完成 7 块
```

## 4. 单测

`test/test_task_rules.py` 共 14 个用例，覆盖：蛋糕固定 A、映射优先、映射缺失时分散兜底、
部分映射只影响缺失项、未知 ID 用全局兜底、截止时间判定（提前/刚好/超时）。

```bash
cd auto_match_25/auto_match && python3 -m unittest discover -s test -t test -v
```

## 5. 调参建议

| 情况 | 调整 |
| --- | --- |
| 场地大、导航慢，最后一块总放不完 | `min_remaining_s` 提到 120 |
| 想多抓一块、接受风险 | `time_budget_s` 提到 570，`min_remaining_s` 降到 60 |
| 分类板识别很稳 | `fallback_by_name` 可留空（走全局兜底） |
| 想让所有物品都去 A（保守放弃 B/C/D） | 把 `fallback_by_name` 全改成 `Collection_A` |
