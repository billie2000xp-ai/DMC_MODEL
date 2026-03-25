import subprocess
import re
import itertools
import os
import sys
import shutil

# ================= 路径配置区 =================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DRAMSIM_EXE = os.path.join(BASE_DIR, "DRAMSim.exe")
CONFIG_FILE = os.path.join(BASE_DIR, "parameter", "HBM", "hbmsystem.ini")
LOG_FILE = os.path.join(BASE_DIR, "log", "hbm_state0.log")

# ================= 搜索空间配置 =================
DATA_SIZES = [64, 128, 256]
WR_RATIOS = [33, 50, 66]

WR_OFFSETS = [-10, 0, 10]
RD_OFFSETS = [-10, 0, 10]
WLEVELH_SPACE = [30, 40, 50] 

# 初始默认值基准
BASE_WR = [13, 28, 35, 38, 55]
BASE_RD = [1, 3, 4, 8, 30]

# ================= 核心逻辑 =================

def load_baseline_config():
    if not os.path.exists(CONFIG_FILE):
        print(f"Critical Error: Cannot find {CONFIG_FILE}!")
        sys.exit(1)
    with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
        return f.readlines()

def update_config(baseline_lines, wr_offset, rd_offset, w_high):
    new_lines = []
    for line in baseline_lines:
        stripped = line.strip()
        
        # 修改 WR_LEVEL 和 WR_MOST_LEVEL
        if stripped.startswith("WR_LEVEL") or stripped.startswith("WR_MOST_LEVEL"):
            match = re.search(r'\[(.*?)\]', line)
            if match:
                nums = [int(x.strip()) for x in match.group(1).split(',')]
                nums[-1] = max(0, nums[-1] + wr_offset) 
                new_arr_str = f"[{','.join(map(str, nums))}]"
                line = re.sub(r'\[.*?\]', new_arr_str, line)
                
        # 修改 RD_LEVEL
        elif stripped.startswith("RD_LEVEL"):
            match = re.search(r'\[(.*?)\]', line)
            if match:
                nums = [int(x.strip()) for x in match.group(1).split(',')]
                nums[-1] = max(0, nums[-1] + rd_offset)
                new_arr_str = f"[{','.join(map(str, nums))}]"
                line = re.sub(r'\[.*?\]', new_arr_str, line)
                
        # 修改 PERF_CMD_WLEVELH
        elif stripped.startswith("PERF_CMD_WLEVELH"):
            line = re.sub(r'(PERF_CMD_WLEVELH\s*=\s*)\d+', fr'\g<1>{w_high}', line)
            
        new_lines.append(line)

    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

def run_9_cases():
    total_score = 0.0
    print(" [", end="", flush=True) # 开始打印这9个子任务的进度
    
    for ds, wr in itertools.product(DATA_SIZES, WR_RATIOS):
        if os.path.exists(LOG_FILE):
            os.remove(LOG_FILE)
            
        # 安全传参模式
        cmd = [DRAMSIM_EXE, f"DATA_SIZE={ds}", f"WR_RATIO={wr}"]
        try:
            # 删除了 shell=True，用列表传参更安全
            subprocess.run(cmd, capture_output=True, text=True, cwd=BASE_DIR, timeout=120)
            
            if os.path.exists(LOG_FILE):
                with open(LOG_FILE, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    match = re.search(r"DFI Total efficiency\s*:\s*([\d\.]+)", content, re.IGNORECASE)
                    if match:
                        val = float(match.group(1))
                        total_score += val
                        # 实时打印刚刚跑完的这个Case的带宽
                        print(f"{val:.1f}% ", end="", flush=True)
                    else:
                        print("E ", end="", flush=True) # 解析错误
            else:
                print("X ", end="", flush=True) # 日志没生成 (可能崩溃了)
                
        except subprocess.TimeoutExpired:
            print("T ", end="", flush=True) # 超时
        except Exception as e:
            print("! ", end="", flush=True) # 其他错误
            
    print(f"] -> Sum Effi: {total_score:.2f}")
    return total_score

def main():
    if not os.path.exists(DRAMSIM_EXE):
        print(f"Error: {DRAMSIM_EXE} not found!")
        return

    baseline_lines = load_baseline_config()
    backup_file = CONFIG_FILE + ".backup"
    if not os.path.exists(backup_file):
        shutil.copyfile(CONFIG_FILE, backup_file)

    best_effi = 0
    best_params = {}
    combinations = list(itertools.product(WR_OFFSETS, RD_OFFSETS, WLEVELH_SPACE))
    total_runs = len(combinations)

    print(f"\nStarting Grid Search... Total {total_runs} Configs")
    print(f"Each config will run 9 cases. Please wait patiently...")
    print("-" * 75)

    try:
        for i, (wr_off, rd_off, w_high) in enumerate(combinations):
            # 打印当前正在测的配置
            status = f"[{i+1:02d}/{total_runs}] WR4_Off:{wr_off:+3d} | RD4_Off:{rd_off:+3d} | WH:{w_high}"
            print(f"{status:<40}", end="", flush=True)
            
            update_config(baseline_lines, wr_off, rd_off, w_high)
            score = run_9_cases()
            
            if score > best_effi:
                best_effi = score
                best_params = {"WR Offset": wr_off, "RD Offset": rd_off, "WLEVELH": w_high}
                
    except KeyboardInterrupt:
        print("\n\n[!] User aborted the script. Exiting gracefully...")
        
    finally:
        print("Restoring original hbmsystem.ini...")
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            f.writelines(baseline_lines)

    if best_effi > 0:
        print("\n" + "="*50)
        print("🎯 FINAL OPTIMIZATION RESULTS")
        print("="*50)
        print(f"Best Sum Efficiency : {best_effi:.2f}")
        print(f"Optimal Parameters  : {best_params}")
        print("="*50)

if __name__ == "__main__":
    main()