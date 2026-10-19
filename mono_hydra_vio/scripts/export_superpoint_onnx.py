#!/usr/bin/env python3
"""Export SuperPoint (Magic Leap superpoint_v1.pth) to ONNX for rvio2_sp_mono_node.

The exported model exposes the raw network heads that RVIO2-SP decodes in C++:
  input : 1x1xHxW grayscale in [0, 1]
  semi  : 1x65x(H/8)x(W/8) detector logits
  desc  : 1x256x(H/8)x(W/8) coarse descriptors (not normalised)

Example (ITC RealSense stream):
  python3 export_superpoint_onnx.py --weights superpoint_v1.pth \
      --height 480 --width 848 --output ../models/superpoint_480x848.onnx

Requires torch, onnx and ml_dtypes. Note: the superpoint_v1 weights are distributed under Magic Leap's
non-commercial research license.
"""

import argparse

import torch
import torch.nn as nn


class SuperPointNet(nn.Module):
    """Architecture of the reference SuperPointPretrainedNetwork, without output normalisation."""

    def __init__(self):
        super().__init__()
        self.relu = nn.ReLU(inplace=True)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        c1, c2, c3, c4, c5, d1 = 64, 64, 128, 128, 256, 256
        self.conv1a = nn.Conv2d(1, c1, kernel_size=3, stride=1, padding=1)
        self.conv1b = nn.Conv2d(c1, c1, kernel_size=3, stride=1, padding=1)
        self.conv2a = nn.Conv2d(c1, c2, kernel_size=3, stride=1, padding=1)
        self.conv2b = nn.Conv2d(c2, c2, kernel_size=3, stride=1, padding=1)
        self.conv3a = nn.Conv2d(c2, c3, kernel_size=3, stride=1, padding=1)
        self.conv3b = nn.Conv2d(c3, c3, kernel_size=3, stride=1, padding=1)
        self.conv4a = nn.Conv2d(c3, c4, kernel_size=3, stride=1, padding=1)
        self.conv4b = nn.Conv2d(c4, c4, kernel_size=3, stride=1, padding=1)
        self.convPa = nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convPb = nn.Conv2d(c5, 65, kernel_size=1, stride=1, padding=0)
        self.convDa = nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convDb = nn.Conv2d(c5, d1, kernel_size=1, stride=1, padding=0)

    def forward(self, x):
        x = self.relu(self.conv1a(x))
        x = self.pool(self.relu(self.conv1b(x)))
        x = self.relu(self.conv2a(x))
        x = self.pool(self.relu(self.conv2b(x)))
        x = self.relu(self.conv3a(x))
        x = self.pool(self.relu(self.conv3b(x)))
        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))
        semi = self.convPb(self.relu(self.convPa(x)))
        desc = self.convDb(self.relu(self.convDa(x)))
        return semi, desc


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True, help="superpoint_v1.pth")
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--output", required=True)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    if args.height % 8 or args.width % 8:
        parser.error("height and width must be multiples of 8")

    net = SuperPointNet()
    net.load_state_dict(torch.load(args.weights, map_location="cpu"))
    net.eval()

    dummy = torch.zeros(1, 1, args.height, args.width)
    torch.onnx.export(
        net,
        dummy,
        args.output,
        input_names=["input"],
        output_names=["semi", "desc"],
        opset_version=args.opset,
        dynamo=False,
    )
    print(f"exported {args.output} ({args.width}x{args.height})")


if __name__ == "__main__":
    main()
