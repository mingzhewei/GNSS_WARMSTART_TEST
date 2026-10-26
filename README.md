# GNSS_WARMSTART_TEST

GNSS 热启动（断天线重接）多设备收敛分析工具。支持**多设备同一轮"断天线→重新接上天线"热启动测试**的统一时序对齐、统一取早起点、统一出一份汇总报告。当前已注册 **北云 BY（UG016 组合惯导）**、**华测 HUACE（M720 模组）** 与 **北斗 BD（和芯星通 UM980）**，可通过注册机制扩展新设备。

## 测试场景

户外连续录制中直接断开天线再接上，卫星数归零后重新搜星，直到重新获得定位（单点→浮点→固定）。一份日志内含多次（10~20 次）循环。

## 核心口径（均按手册/官方文档核实，非猜测）

- **起点事件 t=0 = locktime 0→>0**（信号重新锁定）：
  - 北云：TRACKSTATA（UG016 4.2.26 字段12 locktime）；华测：RANGEA（M7 3.2.25 字段14 locktime）；北斗：OBSVMB（N4 R1.15 7.3.65 表7-137 字段12 locktime）。
  - locktime 翻正早于该星计入 #SVs、更早于定位解，比"第一颗星(#SVs>0)"更早且对搜星慢的机器公平。
  - 天线接上的物理瞬间在日志中无报文记录（需 ANTENNASTATUS/EVENT 硬件标记，本批未录制）。
- **多设备取早**：放弃第 1 次循环（接收机尚未持有卫星时间、周秒无法对齐）；第 2 次循环起各设备均已持有卫星时间，用绝对 GPS 周秒对齐（±30s 配对断线起点），每次循环取所有设备中最早的 locktime 作为统一 t0。
- **自动周期切割**：连续 0 锁定 ≥8s 记为一次断线，相邻 <8s 的抖动段合并。
- **数据完整性**：实测周期 = 相邻帧报头时间差中位数（不假设频率）；真实间断 > 2.5×周期；时间轴跳变 > 3600s（无电池备份 RTC、长时间无星后时标失效退回默认周，重捕后由电文解出真实周，按连续处理）。

## 文件说明

| 文件 | 说明 |
|---|---|
| `gnss_parsers.py` | 设备解析模块注册表（ParserBase 标准接口 + 北云/华测三个 parser + 高频源动态选择 + 逐帧完整性检查 + 设备自动识别；北斗 OBSVMB 走二进制帧扫描和 CRC 校验） |
| `warmstart_report.py` | 多设备汇总分析器（统一对齐取早 + 出 HTML/MD/PNG/JSON） |
| `gnss_warmstart_hmi.py` | HMI 界面（列表式：下拉选类型→添加文件，左文件名右设备类型，多设备混合，一键汇总） |
| `BY_warmstart_.py` / `HUACE_warmstart.py` / `BD_warmstart.py` | 单机版分析（可独立运行） |
| `by_data/` `huace_data/` `beidou_data/` | 示例数据 |
| `by_manual/` `huace_manual/` `beidou_manual/` | 手册（UG016.md / M7 系列 PDF / UM980 与 N4 协议 PDF） |

## 使用

```bash
# 方式一：HMI（推荐）
python gnss_warmstart_hmi.py

# 方式二：命令行（多设备汇总，设备类型自动识别）
python warmstart_report.py <输出目录> <日志1> [日志2 ...]
```

## 扩展新设备

在 `gnss_parsers.py` 中实现一个 `ParserBase` 子类（`match_path` + `parse` + 手册条款），用 `@register` 注册。HMI 下拉会自动出现该设备，汇总器自动纳入统一对齐取早。

## 依赖

- Python 3.9+，仅标准库 + `matplotlib`