# 项目长期记忆（GNSS 冷启动 / 热启动测试）

## 项目背景
- 仓库：`https://github.com/mingzhewei/GNSS_WARMSTART_TEST.git`（main；2026-10-26 起 coldstart 旧脚本已移除，仓库聚焦 warmstart）
- 被测设备：北云 BY（UG016 组合惯导）、华测 HUACE（M720 模组）、北斗 BD（和芯星通 UM980）
- 测试目的：
  - 冷启动（历史）：对比室内/室外冷启动定位收敛速度，找室内固定解慢的瓶颈
  - 热启动（当前主线）：户外断天线→重接，卫星数归零后重新搜星到恢复单点/浮点/固定的多设备对比
- 差分基站 ID 固定为 1793，差分龄期正常 ~1.3s

## 目录与数据约定（当前仓库）
- 代码：`gnss_parsers.py`（ParserBase+@register 注册表，含 BeiyunParser/HuaceParser/BeidouParser、设备自动识别、逐帧完整性）、`warmstart_report.py`（多设备汇总）、`gnss_warmstart_hmi.py`（HMI）、`BY_warmstart_.py`/`HUACE_warmstart.py`/`BD_warmstart.py`（单机版）
- 数据：`by_data/`（.dat×5）、`huace_data/`（.log×2）、`beidou_data/`（.log×3），全部已入 Git
- 手册：`by_manual/`（UG016）、`huace_manual/`（M7 PDF）、`beidou_manual/`（UM980+N4 协议 PDF、指令对照表）
- **报告不入库**：reports*/ 已从 Git 删除，需要时运行脚本重新生成；`reports_tmp_*/` 在 .gitignore

## 核心口径（均按手册核实，非猜测）
- 热启动起点 t=0 = locktime 0→>0：北云 TRACKSTATA（UG016 4.2.26 字段12）、华测 RANGEA（M7 3.2.25 字段14）、北斗 OBSVMB（N4 R1.15 7.3.65 表7-137 字段12，二进制 AA44B5 帧+CRC）
- 多设备对齐：放弃第 1 次循环（无卫星时间），第 2 次起按绝对 GPS 周秒 ±30s 配对，每循环取最早 locktime 为统一 t0
- 断线判定：连续 0 锁定 ≥8s 为一次断线，<8s 抖动段合并
- 时间轴：标称周期=相邻帧报头差中位数；真实间断 >2.5×周期；跳变 >3600s 按周重定标连续处理（无电池 RTC 退回默认周，重捕后由电文解出真实周）
- 离群值：IQR 法 dynamic_ylim，upper=Q3+2×IQR，y 轴上限 1.15×upper，离群点计数标注不截断
- 华测位置源优先连续 BESTPA（RTKPA 稀疏仅作 fallback）；事件窗口按本设备 next_outage_start 截断，反向/无效阶段耗时不报
- 华测 PSRDIFF：手册无此字面量，实测出现且 Stn ID 有值，按伪距差分归单点类（实现事实，待官方确认）

## 已确认的重要技术结论（冷启动批次）
- FINESTEERING（时间精对准）**不是** RTK 浮点/固定解的前置条件：多个批次在 COARSE 时已进入 NARROW_FLOAT
- 室内冷启动慢的大头是"待在室内收不到星"的等待段（跟踪星均值 0.2–0.4 颗）；到开阔处后时间→单点仅 3 秒级
- 固定解质量室内外一致（固定段纬度 σ 0.021–0.025m），前提是观测可用

## 用户偏好
- 只关心 GNSS（时间获取/单点/浮点/固定、卫星数），**不要分析 INS/惯导状态**
- 报告需要时再生成，不进 Git；数据（北云/华测/北斗）全部保留并上传
- 表格/图表要能直接看懂；字段口径必须有手册依据，不允许猜测

## 工程注意
- Python 源文件去 BOM；运行前 $env:PYTHONIOENCODING=utf-8；终端显示中文乱码时用 python -X utf8 读文件验证实际编码
- PowerShell 批量删除目录可能被策略拦截：改用 cmd /c rmdir /s /q
- GitHub >50MB 文件仅警告（GH001 建议 LFS），可正常推送
- 历史细节见 2026-08-12 ~ 2026-10-26 各日记文件
