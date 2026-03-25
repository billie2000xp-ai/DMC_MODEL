import subprocess
import re
import itertools
import os
import sys
import shutil
import csv

# ================= 路径配置区 =================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DRAMSIM_EXE = os.path.join(BASE_DIR, "DRAMSim.exe")
CONFIG_FILE = os.path.join(BASE_DIR, "parameter", "HBM", "zhuque_hbmsystem.ini")
LOG_FILE = os.path.join(BASE_DIR, "log", "hbm_state0.log")
CSV_RESULT_FILE = os.path.join(BASE_DIR, "optimization_results.csv")

# ================= 测试矩阵 =================
DATA_SIZES = [64, 128, 256]
WR_RATIOS = [33, 50, 66]

# ================= 绝对值搜索空间 =================
# 基于最新黄金基准的最后一档 (WR:48, WR_MOST:64, RD:25) 展开地毯式搜索
# range(start, stop, step)

# WR_LEVEL[4]: 写触发阈值 (42 到 56, 步长 2) -> 8个值
WR_SPACE = range(42, 65, 2) 

# WR_MOST_LEVEL[4]: 最大连续写配额 (56 到 80, 步长 4) -> 7个值
WRM_SPACE = range(40, 70, 4)

# RD_LEVEL[4]: 读刹车阈值 (21 到 29, 步长 2) -> 5个值
RD_SPACE = range(19, 35, 2)

# 定义前 4 个档位的平滑基准
BASE_WR  = [16, 28, 32, 37]
BASE_WRM = [13, 18, 28, 35]
BASE_RD  = [2, 6, 10, 16]

# ================= 核心逻辑 =================

def load_baseline_config():
    if not os.path.exists(CONFIG_FILE):
        print(f"Critical Error: Cannot find {CONFIG_FILE}!")
        sys.exit(1)
    with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
        return f.readlines()

def update_config(baseline_lines, wr_val, wrm_val, rd_val):
    """直接用绝对数值拼接新数组，绝对安全且防错"""
    wr_str  = f"[{','.join(map(str, BASE_WR))},{wr_val}]"
    wrm_str = f"[{','.join(map(str, BASE_WRM))},{wrm_val}]"
    rd_str  = f"[{','.join(map(str, BASE_RD))},{rd_val}]"
    
    new_lines = []
    for line in baseline_lines:
        stripped = line.strip()
        if stripped.startswith("WR_LEVEL"):
            line = re.sub(r'=\s*\[.*?\]', f'={wr_str}', line)
        elif stripped.startswith("WR_MOST_LEVEL"):
            line = re.sub(r'=\s*\[.*?\]', f'={wrm_str}', line)
        elif stripped.startswith("RD_LEVEL"):
            line = re.sub(r'=\s*\[.*?\]', f'={rd_str}', line)
            
        new_lines.append(line)

    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

def run_9_cases():
    total_score = 0.0
    case_results = []
    print(" [", end="", flush=True)
    
    for ds, wr in itertools.product(DATA_SIZES, WR_RATIOS):
        if os.path.exists(LOG_FILE):
            os.remove(LOG_FILE)
            
        cmd = [DRAMSIM_EXE, f"DATA_SIZE={ds}", f"WR_RATIO={wr}"]
        try:
            subprocess.run(cmd, capture_output=True, text=True, cwd=BASE_DIR, timeout=60)
            
            if os.path.exists(LOG_FILE):
                with open(LOG_FILE, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    match = re.search(r"DFI Total efficiency\s*:\s*([\d\.]+)", content, re.IGNORECASE)
                    if match:
                        val = float(match.group(1))
                        total_score += val
                        case_results.append(val)
                        print(f"{val:.1f}% ", end="", flush=True)
                    else:
                        case_results.append(0.0)
                        print("E ", end="", flush=True)
            else:
                case_results.append(0.0)
                print("X ", end="", flush=True)
        except Exception:
            case_results.append(0.0)
            print("! ", end="", flush=True)
            
    print(f"] -> Sum: {total_score:.2f}")
    return total_score, case_results

def main():
    print("Initializing Absolute Grid Search (Fusion Baseline)...")
    
    # 构建并过滤搜索空间
    raw_combinations = list(itertools.product(WR_SPACE, WRM_SPACE, RD_SPACE))
    valid_combinations = []
    
    for wr, wrm, rd in raw_combinations:
        # 新物理约束：连续排空(wrm) 必须大于等于 触发(wr) 以获得滞回优势
        # 但不能大得离谱（比如比 wr 大 30 以上可能导致读严重超时）
        if wr <= wrm <= wr + 24: 
            valid_combinations.append((wr, wrm, rd))

    total_runs = len(valid_combinations)
    print(f"Generated {len(raw_combinations)} configs, filtered to {total_runs} logic-safe configs.")
    print(f"Data will be appended to: {CSV_RESULT_FILE}\n")

    baseline_lines = load_baseline_config()
    backup_file = CONFIG_FILE + ".backup"
    if not os.path.exists(backup_file):
        shutil.copyfile(CONFIG_FILE, backup_file)

    # 初始化 CSV 头
    with open(CSV_RESULT_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["ID", "WR_L4", "WR_MOST_L4", "RD_L4", "Sum_Effi", 
                         "D64_W33", "D64_W50", "D64_W66", 
                         "D128_W33", "D128_W50", "D128_W66", 
                         "D256_W33", "D256_W50", "D256_W66"])

    best_effi = 0
    best_params = {}

    try:
        for i, (wr, wrm, rd) in enumerate(valid_combinations):
            print(f"[{i+1:03d}/{total_runs}] WR4:{wr:2d} | WRM4:{wrm:2d} | RD4:{rd:2d}", end="", flush=True)
            
            update_config(baseline_lines, wr, wrm, rd)
            score, case_scores = run_9_cases()
            
            # 写入 CSV
            with open(CSV_RESULT_FILE, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([i+1, wr, wrm, rd, round(score, 2)] + case_scores)
            
            # 记录最优
            if score > best_effi:
                best_effi = score
                best_params = {"WR4": wr, "WR_MOST4": wrm, "RD4": rd}
                
    except KeyboardInterrupt:
        print("\n\n[!] Search interrupted by user.")
        
    finally:
        print("\nRestoring baseline hbmsystem.ini...")
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            f.writelines(baseline_lines)

    if best_effi > 0:
        print("\n" + "="*50)
        print("🏆 FUSION OPTIMIZATION RESULTS 🏆")
        print("="*50)
        print(f"Highest Sum Efficiency : {best_effi:.2f}")
        print(f"Optimal Parameters     : {best_params}")
        print("="*50)

if __name__ == "__main__":
    main()