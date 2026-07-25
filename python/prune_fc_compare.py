import argparse
from pathlib import Path

import torch
import torch.nn.functional as F
import torch.utils.data as data
import torchvision
from spikingjelly.activation_based import encoding, functional, layer, neuron, surrogate


class SNN(torch.nn.Module):
    def __init__(self, tau):
        super().__init__()
        self.layer = torch.nn.Sequential(
            layer.Conv2d(in_channels=1, out_channels=2, kernel_size=3, padding=1, bias=False),
            neuron.LIFNode(tau=tau, surrogate_function=surrogate.ATan()),
            layer.Flatten(),
            layer.Linear(2 * 28 * 28, 10, bias=False),
            neuron.LIFNode(tau=tau, surrogate_function=surrogate.ATan()),
        )

    def forward(self, x):
        return self.layer(x)


def load_fc_txt_weight(weight_dir):
    """读取 fc_digit_0_float.txt ~ fc_digit_9_float.txt，得到 10 x 1568 的全连接权重。"""
    weight_dir = Path(weight_dir)
    files = sorted(weight_dir.glob("fc_digit_*_float.txt"))
    if len(files) != 10:
        raise RuntimeError(f"期望找到 10 个 fc_digit_x_float.txt，但实际找到 {len(files)} 个。")

    rows = []
    for file_path in files:
        values = [float(x) for x in file_path.read_text(encoding="utf-8").split()]
        rows.append(values)

    weight = torch.tensor(rows, dtype=torch.float32)
    if weight.shape != (10, 1568):
        raise RuntimeError(f"全连接权重形状应为 (10, 1568)，实际为 {tuple(weight.shape)}。")
    return weight


def apply_fc_prune(net, baseline_fc_weight, threshold):
    """只剪全连接层：|weight| < threshold 的连接置 0。"""
    pruned_weight = baseline_fc_weight.clone()
    pruned_weight[pruned_weight.abs() < threshold] = 0.0
    with torch.no_grad():
        net.layer[3].weight.copy_(pruned_weight)
    return pruned_weight


def calc_sparse_stats(weight):
    """统计剪枝后的稀疏率和每个输入事件平均连接到几个输出类别。"""
    nonzero_mask = weight != 0
    kept = int(nonzero_mask.sum().item())
    total = weight.numel()
    fanout = nonzero_mask.sum(dim=0).float()
    return {
        "sparsity": 1.0 - kept / total,
        "avg_fanout": float(fanout.mean().item()),
        "zero_fanout_inputs": int((fanout == 0).sum().item()),
        "max_fanout": int(fanout.max().item()),
    }


@torch.no_grad()
def evaluate(net, data_loader, encoder, device, time_steps, max_samples):
    """使用和训练脚本一致的 Poisson 编码 + T 时间步发放率方式评估准确率。"""
    net.eval()
    total = 0
    correct = 0
    loss_sum = 0.0

    for img, label in data_loader:
        if max_samples is not None and total >= max_samples:
            break

        if max_samples is not None:
            remain = max_samples - total
            if img.shape[0] > remain:
                img = img[:remain]
                label = label[:remain]

        img = img.to(device)
        label = label.to(device)
        label_onehot = F.one_hot(label, 10).float()

        out_fr = 0.0
        for _ in range(time_steps):
            encoded_img = encoder(img)
            out_fr += net(encoded_img)
        out_fr = out_fr / time_steps

        loss = F.mse_loss(out_fr, label_onehot)
        pred = out_fr.argmax(1)

        total += label.numel()
        correct += (pred == label).float().sum().item()
        loss_sum += loss.item() * label.numel()

        functional.reset_net(net)

    return {
        "acc": correct / total,
        "loss": loss_sum / total,
        "samples": total,
    }


def build_test_loader(data_dir, batch_size, num_workers, download):
    test_dataset = torchvision.datasets.MNIST(
        root=data_dir,
        train=False,
        transform=torchvision.transforms.ToTensor(),
        download=download,
    )
    return data.DataLoader(
        dataset=test_dataset,
        batch_size=batch_size,
        shuffle=False,
        drop_last=False,
        num_workers=num_workers,
        pin_memory=True,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="FC 剪枝对照测试")
    parser.add_argument("--checkpoint", type=str, required=True, help="训练脚本保存的 checkpoint_max.pth 或 checkpoint_latest.pth")
    parser.add_argument("--fc-weight-dir", type=str, default=r"D:\Desktop\python\顶层测试\weight_txt", help="fc_digit_x_float.txt 所在目录")
    parser.add_argument("--use-fc-txt", action="store_true", help="用 txt 权重覆盖 checkpoint 中的全连接层权重")
    parser.add_argument("--data-dir", type=str, default=r"D:\Me\mnist_data", help="MNIST 数据集目录")
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--T", type=int, default=100)
    parser.add_argument("--tau", type=float, default=2.0)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--max-samples", type=int, default=None, help="只测前 N 张图，调试时可设小一点")
    parser.add_argument("--thresholds", type=float, nargs="+", default=[0.1, 0.2, 0.3, 0.5])
    parser.add_argument("--download", action="store_true", help="如果本地没有 MNIST，则允许下载")
    return parser.parse_args()


def main():
    args = parse_args()
    device = torch.device(args.device if torch.cuda.is_available() or "cuda" not in args.device else "cpu")

    net = SNN(tau=args.tau).to(device)
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    net.load_state_dict(checkpoint["net"])

    if args.use_fc_txt:
        fc_weight = load_fc_txt_weight(args.fc_weight_dir)
        with torch.no_grad():
            net.layer[3].weight.copy_(fc_weight.to(device))

    baseline_fc_weight = net.layer[3].weight.detach().cpu().clone()
    test_loader = build_test_loader(args.data_dir, args.batch_size, args.num_workers, args.download)
    encoder = encoding.PoissonEncoder()

    print("=== baseline ===")
    baseline_result = evaluate(net, test_loader, encoder, device, args.T, args.max_samples)
    print(f"samples={baseline_result['samples']}, acc={baseline_result['acc']:.4f}, loss={baseline_result['loss']:.6f}")

    print("\n=== prune fc only ===")
    print("threshold  sparsity   avg_fanout  zero_inputs  max_fanout  acc     acc_drop")
    for threshold in args.thresholds:
        pruned_weight = apply_fc_prune(net, baseline_fc_weight.to(device), threshold).cpu()
        stats = calc_sparse_stats(pruned_weight)
        result = evaluate(net, test_loader, encoder, device, args.T, args.max_samples)
        acc_drop = baseline_result["acc"] - result["acc"]

        print(
            f"{threshold:9.3f}  "
            f"{stats['sparsity'] * 100:7.2f}%    "
            f"{stats['avg_fanout']:8.3f}  "
            f"{stats['zero_fanout_inputs']:11d}  "
            f"{stats['max_fanout']:10d}  "
            f"{result['acc']:.4f}  "
            f"{acc_drop:.4f}"
        )


if __name__ == "__main__":
    main()
