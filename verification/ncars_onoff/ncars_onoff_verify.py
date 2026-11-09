"""复现双极性卷积仿真，并保留源码哈希、逐例周期和工具日志。"""

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TOOLS = Path(r"D:\Me\study\CPU\my_risc-v\iverilog\iverilog\iverilog\bin")
SOURCES = ["GPT_event_driven_conv_pe_onoff_tb.sv", "../../top/event_driven_conv_pe_onoff.v",
           "../../top/event_driven_conv_pe_onoff_9cycle.v", "ncars_onoff_verification/event_driven_conv_pe_reference32.v",
           "../../top/spike_event_scanner.v", "../../top/bitmap_to_addr_stream.v"]


def run(command, log):
    result = subprocess.run([str(arg) for arg in command], cwd=ROOT,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, errors="replace", timeout=300)
    log.write_text(result.stdout, encoding="utf-8")
    if result.returncode:
        raise RuntimeError("Command failed: %s\n%s" % (command, result.stdout[-4000:]))
    return result.stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--modes", nargs="+", default=["synthetic", "limits", "cancel", "trained"])
    parser.add_argument("--collect-existing", action="store_true", help="汇总本轮已运行的日志，不重复仿真")
    args = parser.parse_args()
    for mode in args.modes:
        directory = ROOT / "ncars_onoff_verification" / mode
        directory.mkdir(parents=True, exist_ok=True)
        if args.collect_existing:
            output = (directory / "simulation.log").read_text(encoding="utf-8")
            if any((ROOT / file).stat().st_mtime > (directory / "simulation.out").stat().st_mtime for file in SOURCES):
                raise RuntimeError("Source newer than simulation: " + mode)
        else:
            output = run([sys.executable, ROOT / "ncars_onoff_generate.py", "--mode", mode], directory / "generate.log")
            print(output.strip(), flush=True)
            compile_command = [TOOLS / "iverilog.exe", "-g2012", "-I", directory,
                               "-s", "GPT_event_driven_conv_pe_onoff_tb", "-o", directory / "simulation.out"]
            run(compile_command + [ROOT / file for file in SOURCES], directory / "compile.log")
            output = run([TOOLS / "vvp.exe", "-i", directory / "simulation.out",
                          "+DATA_DIR=" + directory.as_posix()], directory / "simulation.log")
        summary = re.search(r"^PASS cases=(\d+) channels=4 fast_cycles=(\d+) serial_cycles=(\d+) baseline_cycles=(\d+)$",
                            output, re.M)
        if not summary or "FATAL" in output:
            raise RuntimeError("Missing PASS: " + output[-4000:])
        cases = re.findall(r"CASE_PASS id=(\d+) events=(\d+) fast=(\d+) serial=(\d+) baseline=(\d+) writes=(\d+)", output)
        count, fast, serial, baseline = map(int, summary.groups())
        if len(cases) != count:
            raise RuntimeError("Incomplete case log")
        report = {"mode": mode, "cases": count, "channels": 4, "fast_cycles": fast,
                  "serial_cycles": serial, "baseline_cycles": baseline,
                  "baseline_note": "original optimized PE with only signed row width fixed for 32-row tail flush",
                  "source_sha256": {file: hashlib.sha256((ROOT / file).read_bytes()).hexdigest() for file in SOURCES},
                  "vector_sha256": {file: hashlib.sha256((directory / file).read_bytes()).hexdigest()
                                    for file in ["input_on.mem", "input_off.mem", "golden.mem", "active.mem", "ncars_onoff_vectors.vh"]},
                  "per_case": [dict(zip(["id", "events", "fast", "serial", "baseline", "writes"], map(int, row)))
                               for row in cases]}
        (directory / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(summary.group(0), flush=True)


if __name__ == "__main__":
    main()
