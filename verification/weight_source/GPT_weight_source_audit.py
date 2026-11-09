"""只读核对训练 checkpoint、导出权重、RTL 常量和 Vivado 初始化文件。"""

import hashlib
import json
import re
from pathlib import Path

import torch


ROOT = Path(r"D:\Me\study\Accelerator")
TRAIN = ROOT / "SNN Acc/保存权重训练文件/保存权重训练文件"
RUN = TRAIN / "runs_lif_fc_mnist/T100_b64_adam_lr0.001"
PROJECT = ROOT / "B shoucuo"
DESKTOP = Path(r"D:\Desktop\python")
TOP = Path(__file__).resolve().parents[2] / "top"
IP = PROJECT / "XILINX_PRJ/Snn_top/Snn_top.srcs/sources_1/ip"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quantize(values):
    return [max(-32768, min(32767, round(float(v) * 4096))) & 0xFFFF
            for v in values]


def coe_values(path):
    text = path.read_text(encoding="utf-8-sig")
    radix = int(re.search(r"radix\s*=\s*(\d+)", text, re.I).group(1))
    payload = re.search(r"vector\s*=\s*(.*?);", text, re.I | re.S).group(1)
    words = re.findall(r"[0-9a-fA-F]+", payload)
    assert radix == 16, (path, radix)
    # 一个 64bit ROM 字按从高到低的顺序装入四个 16bit 权重。
    return [int(word[i:i + 4], 16) for word in words
            for i in range(0, len(word), 4)]


def mif_values(path):
    words = path.read_text().split()
    assert all(len(w) == 64 and set(w) <= {"0", "1"} for w in words), path
    return [int(w[i:i + 16], 2) for w in words for i in range(0, 64, 16)]


def compare(actual, expected):
    differences = sum(a != b for a, b in zip(actual, expected))
    return {"count": len(actual), "expected_count": len(expected),
            "mismatches": differences,
            "match": len(actual) == len(expected) and differences == 0}


def rtl_conv(path):
    text = path.read_text(encoding="utf-8-sig")
    values = []
    for channel in range(2):
        body = re.search(r"P_CONV%d_WEIGHTS_PACKED\s*=\s*\{(.*?)\}" % channel,
                         text, re.S).group(1)
        values.extend(int(v, 16) for v in re.findall(r"16'h([0-9A-Fa-f]+)", body))
    return values


def main():
    checkpoints = [RUN / "checkpoint_max.pth", RUN / "checkpoint_latest.pth",
                   PROJECT / "auxiliary code/auxiliary code/weight/checkpoint_max.pth"]
    reference = torch.load(checkpoints[0], map_location="cpu", weights_only=True)
    net = reference["net"]
    conv = quantize(net["layer.0.weight"].flatten().tolist())
    fc = net["layer.3.weight"]
    hardware_fc = torch.cat((fc[:, 784:], fc[:, :784]), dim=1)
    report = {"checkpoints": [], "conv_rtl": {}, "fc": []}
    for path in checkpoints:
        ckpt = torch.load(path, map_location="cpu", weights_only=True)
        report["checkpoints"].append({
            "path": str(path), "sha256": digest(path),
            "epoch": ckpt.get("epoch"), "max_test_acc": ckpt.get("max_test_acc"),
            "shapes": {key: list(value.shape) for key, value in ckpt["net"].items()},
            "same_net_as_training_max": set(ckpt["net"]) == set(net) and all(
                torch.equal(value, net[key]) for key, value in ckpt["net"].items())})
    for path in [TOP.parent / "tb/snn_top_tb.sv",
                 PROJECT / "snn_verilog/fpga_power_eval/fpga_top_power_eval.v"]:
        report["conv_rtl"][str(path)] = compare(rtl_conv(path), conv)
    for digit in range(10):
        expected_float = hardware_fc[digit].tolist()
        expected_q = quantize(expected_float)
        txt = DESKTOP / ("顶层测试/weight_txt/fc_digit_%d_float.txt" % digit)
        exported_float = [float(v) for v in txt.read_text().split()]
        assert len(exported_float) == len(expected_float)
        item = {"digit": digit, "float_max_abs_error": max(
            abs(a - b) for a, b in zip(exported_float, expected_float)), "files": {}}
        exported_q = quantize(exported_float)
        item["rounding_differences"] = [
            {"hardware_index": index, "checkpoint_float": expected_float[index],
             "exported_float": exported_float[index],
             "direct_q_hex": "%04X" % expected_q[index],
             "via_txt_q_hex": "%04X" % exported_q[index]}
            for index in range(1568) if expected_q[index] != exported_q[index]]
        for directory in [DESKTOP / "顶层测试/weight_coe", DESKTOP / "weight_coe"]:
            path = directory / ("fc_digit_%d.coe" % digit)
            item["files"][str(path)] = {
                "vs_direct_checkpoint": compare(coe_values(path), expected_q),
                "vs_export_pipeline": compare(coe_values(path), exported_q)}
        for directory in [IP / ("weights_%d" % digit),
                          PROJECT / "XILINX_PRJ/Snn_top/Snn_top.sim/sim_1/behav/modelsim"]:
            path = directory / ("weights_%d.mif" % digit)
            if path.exists():
                item["files"][str(path)] = {
                    "vs_direct_checkpoint": compare(mif_values(path), expected_q),
                    "vs_export_pipeline": compare(mif_values(path), exported_q)}
        report["fc"].append(item)
    result_path = Path(__file__).resolve().parent / "GPT_weight_source_audit_result.json"
    result_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Checkpoint metadata:")
    for item in report["checkpoints"]:
        print("epoch=%s best_acc=%s shapes=%s same_net=%s" % (
            item["epoch"], item["max_test_acc"], item["shapes"],
            item["same_net_as_training_max"]))
    print("Conv RTL matches:", [item["match"] for item in report["conv_rtl"].values()])
    print("FC float max error:", max(item["float_max_abs_error"] for item in report["fc"]))
    print("All COE/MIF match TXT export pipeline:", all(
        file["vs_export_pipeline"]["match"] for item in report["fc"]
        for file in item["files"].values()))
    print("Rounding differences:", [difference for item in report["fc"]
                                   for difference in item["rounding_differences"]])
    print("Report: GPT_weight_source_audit_result.json")


if __name__ == "__main__":
    main()
