# 夺宝奇兵自动赛 · 自动任务组 · 交接文档

> 最后更新：2026-11-23
> 用途：换工作区/换会话后能直接接手，也可转发队友

---

## 一、项目概述

- **赛项**：睿抗「夺宝奇兵」自动任务赛（自动组）
- **机器人**：Spark-I（硬件不可改装，必须用 ROS）
- **场地**：3×3m，A/B/C/D 四角得分区（各 1×1m，含 0.45m 收取区），中央金砖区（一层 3×3 共 9 块：玩偶/时钟/香蕉各 3；二层中间 1 块蛋糕）
- **规则要点**：
  - 10 分钟全自动，程序启动后不能碰键鼠
  - **蛋糕必须先放 A 区**（否则蛋糕不计分）
  - 分类板每局随机决定"玩偶/时钟/香蕉 → B/C/D 哪个区"
  - 金砖可堆叠，层数越高分越高
  - 一层正中间固定是时钟

---

## 二、代码仓库

- **GitHub**：https://github.com/Luckyzzzzx/spark_auto_match
- **最新 commit**：`602ccce`（main 分支）
- **结构**：
  ```
  auto_match_25/
    auto_match/
      nodes/          detect_ros.py, board_classifier.py, board_mapping.py,
                      nav_manager.py, nav_route.py, mark_move.py, runtask.py, cali_node.py
      config/         items_config.yaml, nav_params.yaml, route_table.yaml,
                      mark_map.yaml/pkl, test_map.pgm/yaml, move_base_param/
      launch/         auto_match.launch, vision_test.launch
      scripts/        teach_pad.py, pkl2yaml.py, robot_preflight.py,
                      run_detect_ros.sh, analyze_map.py, route_report.py
      docs/           task_log.md
    common/, spark_alignment_mark/, spark_move/
  tools/
    c_calibration/    C 组机械臂标定工具
    c_vision/         C 组蛋糕识别/数据集工具
    calibration/      thefile.txt, thefile2.txt（手眼标定备份）
  models/             auto.pt, banana.pt, cake.pt（.gitignore 忽略，本地备份）
  CHANGELOG.md, README.md
  ```

---

## 三、机器人 / 连接

| 项目 | 内容 |
|---|---|
| 我们用的机器 | **10.24.170.0**（Ubuntu 20.04 + ROS Noetic） |
| 另一台 | 10.25.141.8（同款，之前也同步过代码） |
| 别组的机器 | 10.24 上还跑过 namasSinjali 的代码，注意别覆盖 |
| SSH | `ssh spark@10.24.170.0`，密码 `spark`（已配公钥免密） |
| VNC | `10.24.170.0:5900` |

**启动/操作**：
```bash
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash

# 建图（open_guide=true）
roslaunch auto_match auto_match.launch open_guide:=true

# 键盘控制（VNC 终端里）
rosrun spark_teleop spark_teleop_node 0.14 0.5

# 关气泵
rostopic pub -1 /pump_topic swiftpro/status "{status: 0}"

# 归位机械臂
rostopic pub -1 /position_write_topic swiftpro/position "{x: 130, y: 0, z: 35}"
```

**注意**：机器人连的是**需认证的校园网**（外网要登录，内网 VNC/SSH 不受影响）。

---

## 四、各组进展

### B 组（导航，本次重点）
- `nav_manager.py`：NavigationManager 封装（goto/arrive/step_go/step_back/就近选点/定位检查/重定位），直接调 move_base action，带超时/卡死/重试
- `nav_route.py`：路径规划纯逻辑（距离/距离矩阵/最近点/贪心顺序），单测 `test/test_nav_route.py`（7 用例）
- `route_table.yaml`：Step 点路由表
- 修复：`goto_local("Collection_A")` → `goto_local(sorting_name)`（原致命 bug）
- 修复：`local_costmap plugins` 缩进（障碍/膨胀层此前未生效）
- 修复：costmap 障碍层启用、inflation 0.01→0.30、激光高度
- 就近选点：runtask 每次送完区选最近分拣点

### A 组（视觉）
- `detect_ros.py`：多模型清单（ultralytics/yolov5 自动回退），支持自动/banana/cake 多模型
- `board_classifier.py` + `board_mapping.py`：分类板识别（ROI/几何校验/多帧投票），输出 `/board_mapping`
- `items_config.yaml`：items（bear=1/wine=2/clock=3/banana=4/cake=5 + site）、多模型清单、分类板配置
- 修复：yolov5 `forward()` 不接受 `iou` 时回退；ultralytics `fuse()` 不接受 `verbose` 时回退

### C 组（机械臂/标定）
- `tools/c_calibration/`：arm_jog、swiftpro_serial_move、pick/place_contact、cake_corners、new_robot_layer2_anchor
- `tools/c_vision/`：cake 数据集构建、模型评估、RGB-D 工具
- 蛋糕模型：`cake_dataset_v3_20261122`（480训练+90验证，mAP50=0.995）

### D 组（调度）
- `runtask.py`：导航接入 nav_manager、就近选点、蛋糕优先 A、防死循环、堆叠（drop/drop1/drop2）、异常守卫

---

## 五、手眼标定（关键）

**文件**：`~/thefile.txt`（一层）、`~/thefile2.txt`（二层），格式 `x_kb[0] x_kb[1] y_kb[0] y_kb[1]`

| 层 | 当前值 | 状态 |
|---|---|---|
| 一层 | `-1.190005 618.095469 -0.853707 251.551805` | ✅ 验证准（误差 ~5mm） |
| 二层 | `-0.613821 396.345642 -0.650891 179.853521` | ✅ 能抓（精度要求低） |

**公式**：`X = x_kb[0]*像素y + x_kb[1]`，`Y = y_kb[0]*像素x + y_kb[1]`

**重新标定方法**（`scripts/teach_pad.py`）：
- 两步记录：**G** 记金砖像素（金砖在可见处）→ **M** 记机械臂坐标（吸盘对准金砖）
- 键盘：W/S 前后、A/D 左右、R/F 上下，P 打印，Q 保存到 `/tmp/teach_points.txt`
- 拟合：`X=a*py+b, Y=c*px+d`（用 `np.polyfit`）
- 备份在 `tools/calibration/`

**机械臂限位**：x∈[0,320] y∈[-250,250] z∈[-100,200]

---

## 六、已验证（成功）

1. **建图**：gmapping，临时场地，384×384（19.2m），边界可辨
2. **识别**：detect_ros 能识别 clock/bear/banana/wine（4Hz），无 iou/verbose 报错
3. **一层抓取**：静止抓取验证准（误差 ~5mm）
4. **二层抓取**：蛋糕能抓（精度要求低）
5. **堆叠两层**：成功（底层 + 第二层）
6. **标定**：一层/二层都完成

---

## 七、未完成 / 待办（重要）

1. **点位标定 `mark_map`**：分拣点/放置区/分类区/Step 点，需**正式场地**建图标定
2. **三层堆叠问题（未解决）**：
   - uArm 的 `set_pose` 上升时走**关节弧线**（x/y 会漂），会撞倒底层金砖
   - 尝试过：分步（慢）、闭环（v6，读实际位置校正 x/y）——**v6 已写，待验证**
3. **完整流程联调**：导航 → 抓取 → 放置
4. **蛋糕识别不稳**：cake 模型对"印刷图案金砖"框偏，建议**用固定坐标抓蛋糕**（不依赖识别）
5. **正式场地**：需围全、清空人/物

---

## 八、已知问题 / 坑

- **uArm 弧线**：`set_pose` 是关节插值，不是直线，"垂直上升"带水平偏移 → 撞金砖
  - 解决方向：分步小幅度 / 闭环读 SwiftproState 校正（v6）
- **底座太远**：底座 x>280 + 层3 z=160 超臂展，够不到 → 底座要放近（x<260）
- **detect_ros 进程**：改代码后要**重启节点**（launch 不 respawn），否则跑旧代码
- **/objects 空帧**：帧率 4-5Hz，偶尔空帧，脚本要重试或等一下
- **site 标记**：id=10/20/30/40 是场地标记，不是金砖，检测/记录要排除（id<10）
- **蛋糕识别**：cake 模型对印刷金砖不稳，别依赖
- **相机遮挡**：机械臂在吸盘正上方时会挡住金砖（识别不到）——示教时用"G记金砖、M记机械臂"两步
- **10.24 vs 10.25**：10.24 上有别组（namasSinjali）的代码，改前先对比，别覆盖

---

## 九、测试脚本（/tmp 或本地）

| 脚本 | 用途 |
|---|---|
| `teach_pad.py` | 键盘示教标定（G/M 两步） |
| `grasp_test.py` | 一层静止抓取（ArmAction.grasp） |
| `grasp1_test3.py` | 二层抓取（thefile2，先抬再移） |
| `stack_test.py` | 两层堆叠 |
| `stack3_test_v6.py` | 三层堆叠（闭环版，待验证） |
| `analyze_map.py` | 地图 pgm 分析 |

---

## 十、下一步建议

1. **先解决三层堆叠**（uArm 弧线）：验证 v6 闭环版，或调整
2. **正式场地建图 + 标点**（mark_map）
3. **完整流程联调**（runtask 跑通）
4. **蛋糕用固定坐标**抓（避开识别不稳）

---

## 十一、当前进行中的任务（2026-11-23 交接时状态）

**正在做**：三层堆叠调试，卡在 **uArm 上升弧线**问题。

- 两层堆叠已成功（底层 + 第二层）
- 三层失败：机械臂抓第三个后上升时，**关节插值走弧线（x/y 漂）**，撞倒底层
- 已尝试：
  - v4 快速版（一步抬 180）→ 撞
  - v5 分步版（每步 15mm）→ 撞/慢
  - **v6 闭环版**（每步读 SwiftproState 校正 x/y）→ 已写，**待验证**
- 脚本在机器人 `/tmp/stack3_test_v6.py`（本地也有一份在开发时的临时目录，可能不在仓库）

**机器人当前已清理**（所有 ROS 已停，设备空闲），可直接启动。

## 十二、新会话如何接手（给下一个 AI/队友）

新会话没有本次对话记忆，请**先读本文件 + 相关代码**，再继续：

1. **读 `HANDOVER.md`**（本文件，全貌）
2. **读 `auto_match/docs/task_log.md`**（详细记录）
3. **读关键代码**：`nav_manager.py`、`runtask.py`、`detect_ros.py`、`teach_pad.py`
4. **连机器人**：`ssh spark@10.24.170.0`（密码 spark，免密已配）
5. **继续任务**：先验证三层堆叠（`/tmp/stack3_test_v6.py` 闭环版），或按"未完成清单"推进

**开场白模板**（新会话直接发）：
```
读一下仓库里的 HANDOVER.md 和 docs/task_log.md，了解当前进度，
然后继续做三层堆叠的验证（uArm 弧线问题）。
```

## 十三、相关文档

- `CHANGELOG.md`：变更日志
- `auto_match/docs/task_log.md`：详细任务记录
- GitHub Issue #2：交接记录
