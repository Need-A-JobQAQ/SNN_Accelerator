"""生成双极性卷积测试向量；黄金模型使用 PyTorch 的稠密 cross-correlation。"""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F


ROOT = Path(__file__).resolve().parent
EXPERIMENT = Path(r"D:\Desktop\python\DSEC\experiments\ncars_single_conv_maxpool4")


def make_cases():
    cases = []

    def add(name, points=(), array=None):
        image = np.zeros((2, 32, 32), dtype=np.uint8) if array is None else array.copy()
        for polarity, row, col in points:
            image[polarity, row, col] = 1
        cases.append((name, image))

    add("empty")
    add("on_top_left", [(0, 0, 0)])
    add("off_bottom_right", [(1, 31, 31)])
    add("both_same_pixel", [(p, 16, 16) for p in range(2)])
    add("all_corners", [(p, y, x) for p in range(2) for y in [0, 31] for x in [0, 31]])
    add("continuous_2_3_4_5", [(p, 8, x) for p in range(2) for x in range(2, 6)])
    add("jump_2_15", [(0, 7, 2), (1, 7, 15), (0, 7, 15)])
    add("row_transition", [(0, 5, 27), (1, 5, 31), (0, 6, 0), (1, 6, 0)])
    add("first_event_lower_right", [(1, 23, 30)])
    add("empty_middle_rows", [(0, 1, 4), (1, 29, 25)])
    add("overlapping_neighborhoods", [(p, y, x) for p in range(2) for y in [15, 16] for x in [5, 6, 7]])
    for name, polarities in [("dense_on", [0]), ("dense_off", [1]), ("dense_both", [0, 1])]:
        image = np.zeros((2, 32, 32), dtype=np.uint8)
        image[polarities] = 1
        add(name, array=image)
    grid = np.indices((32, 32)).sum(0) % 2
    add("checkerboard", array=np.stack([grid, 1 - grid]).astype(np.uint8))
    rng = np.random.default_rng(20261109)
    for i in range(64):
        density = [0.001, 0.01, 0.05, 0.2, 0.5, 0.9, 1.0, 0.0][i % 8]
        add("random_%02d" % i, array=(rng.random((2, 32, 32)) < density).astype(np.uint8))

    # 只取已固定的数据帧验证卷积层，不重跑分类或据此选择模型。
    datasets = [
        ("train", EXPERIMENT.parent / "ncars_single_conv/ncars_official_train_T10_ONOFF_32x32.npz"),
        ("test", EXPERIMENT / "official_test/ncars_official_test_T10_ONOFF_32x32.npz"),
    ]
    for name, path in datasets:
        with np.load(path, allow_pickle=False) as data:
            sample = data["X"][0].copy()
            for timestep in range(10):
                add("ncars_%s_sample0_t%d" % (name, timestep), array=sample[timestep])
    return cases


def weights_for(mode):
    if mode == "trained":
        path = EXPERIMENT / "runs/seed2026/checkpoint_epoch80.pth"
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
        values = checkpoint["net"]["layer.0.weight"].double()
        assert tuple(values.shape) == (4, 2, 3, 3)
        return torch.round(values * 4096).clamp(-32768, 32767).to(torch.int64), str(path)
    values = torch.empty((4, 2, 3, 3), dtype=torch.int64)
    for ch in range(4):
        for polarity in range(2):
            for index in range(9):
                if mode == "limits":
                    # 两极性同为极限值，直接覆盖32767+32767和-32768-32768。
                    value = [32767, -32768, 32767, -32768, 0][(ch + index) % 5]
                elif mode == "cancel":
                    value = (index * 43 - 151 + ch * 19) * (1 if polarity == 0 else -1)
                else:
                    value = ((ch + 1) * 173 + polarity * 307 + index * 97) % 3001 - 1500
                values[ch, polarity, index // 3, index % 3] = value
    return values, "directed_integer_%s" % mode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["synthetic", "limits", "cancel", "trained"], required=True)
    args = parser.parse_args()
    torch.set_num_threads(2)
    directory = ROOT / "ncars_onoff_verification" / args.mode
    directory.mkdir(parents=True, exist_ok=True)
    cases = make_cases()
    weights, provenance = weights_for(args.mode)
    inputs = torch.from_numpy(np.stack([image for _, image in cases])).double()
    # 全部输入和权重均为整数，float64 稠密卷积在本测试范围内可精确表示结果。
    golden = F.conv2d(inputs, weights.double(), padding=1)
    assert torch.equal(golden, golden.round())
    assert golden.min() >= -(1 << 25) and golden.max() < (1 << 25)
    active = F.conv2d((inputs.sum(1, keepdim=True) != 0).double(),
                      torch.ones((1, 1, 3, 3), dtype=torch.float64), padding=1) != 0

    for polarity, name in [(0, "on"), (1, "off")]:
        # 图像第0个像素放向量最高位，与原 scanner 的约定一致。
        lines = ["%0256x" % int("".join(str(int(v)) for v in image[polarity].flat), 2)
                 for _, image in cases]
        (directory / ("input_%s.mem" % name)).write_text("\n".join(lines) + "\n", encoding="ascii")
    (directory / "golden.mem").write_text("\n".join(
        "%07x" % (int(v) & ((1 << 26) - 1)) for v in golden.flatten().tolist()) + "\n", encoding="ascii")
    (directory / "active.mem").write_text("\n".join(
        str(int(v)) for v in active.flatten().tolist()) + "\n", encoding="ascii")
    packed = []
    for polarity in range(2):
        packed.append("".join("%04x" % (int(v) & 0xffff) for v in weights[:, polarity].flatten().tolist()))
    header = "localparam integer CASE_COUNT = %d;\n" % len(cases)
    header += "localparam [575:0] WEIGHTS_ON = 576'h%s;\n" % packed[0]
    header += "localparam [575:0] WEIGHTS_OFF = 576'h%s;\n" % packed[1]
    (directory / "ncars_onoff_vectors.vh").write_text(header, encoding="ascii")
    report = {"mode": args.mode, "weight_source": provenance, "quantization": "round-to-even, saturate signed Q4.12",
              "shape": list(weights.shape), "cases": [name for name, _ in cases],
              "golden_min": int(golden.min()), "golden_max": int(golden.max()),
              "spatial_events": [int(np.logical_or(image[0], image[1]).sum()) for _, image in cases]}
    (directory / "vectors.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Generated %s: %d cases, 4 channels, golden range [%d, %d]" % (
        args.mode, len(cases), report["golden_min"], report["golden_max"]))


if __name__ == "__main__":
    main()
