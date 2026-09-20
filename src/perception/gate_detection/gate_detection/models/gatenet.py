"""
GateNet: U-Net-style gate segmentation network.

Architecture (f=4, so channels are divided by 4 vs the base description):
  Encoder: inc-16 → down1-32 → down2-64 → down3-128 → down4-128
  Decoder: up1-64 → up2-32 → up3-16 → up4-16
  Outputs: y0 (24x24) … y4 (384x384), y4 used at deployment.

Loss: L_tot = 4*L0 + 2*L1 + L2 + L3 + L4
      L_i   = L_Dice + 2*L_BCE
"""

import os
import math
import random

_GATE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
try:
    from tqdm import tqdm
except ImportError:
    # tqdm is only used by the training loop (train_one_epoch); inference
    # callers (detector_node) don't need it. Degrade to a no-op so this
    # module imports cleanly when tqdm isn't installed.
    def tqdm(iterable, *_args, **_kwargs):
        return iterable


IMG_SIZE   = 384
DATA_ROOT  = os.path.join(_GATE_ROOT, "data")
DEVICE     = "cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------
class DoubleConv(nn.Module):
    """double 3x3 convolutional layer"""
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.net(x)


class Down(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.net = nn.Sequential(nn.MaxPool2d(2), DoubleConv(in_ch, out_ch))

    def forward(self, x):
        return self.net(x)


class Up(nn.Module):
    """Transposed conv (halves spatial dims → doubles), add skip, double conv."""
    def __init__(self, in_ch, out_ch):
        super().__init__()
        # transconv keeps channel count the same, skip has same channels → addition
        self.up   = nn.Sequential(
            nn.ConvTranspose2d(in_ch, in_ch, 2, stride=2, bias=False),
            nn.BatchNorm2d(in_ch),
        )
        self.conv = DoubleConv(in_ch, out_ch)

    def forward(self, x, skip):
        x = self.up(x)
        # handle odd spatial sizes
        if x.shape != skip.shape:
            x = F.interpolate(x, size=skip.shape[2:], mode="bilinear", align_corners=False)
        x = x + skip
        return self.conv(x)


class OutConv(nn.Module):
    def __init__(self, in_ch):
        super().__init__()
        self.net = nn.Sequential(nn.Conv2d(in_ch, 1, 1), nn.Sigmoid())

    def forward(self, x):
        return self.net(x)


# ---------------------------------------------------------------------------
# GateNet
# ---------------------------------------------------------------------------
class GateNet(nn.Module):
    """
    U-Net segmentation model.
    """
    def __init__(self, f: int = 4):
        super().__init__()
        c = [64 // f, 128 // f, 256 // f, 512 // f, 512 // f]

        # Encoder
        self.inc   = DoubleConv(3, c[0])
        self.down1 = Down(c[0], c[1])
        self.down2 = Down(c[1], c[2])
        self.down3 = Down(c[2], c[3])
        self.down4 = Down(c[3], c[4])

        # Decoder
        self.up1 = Up(c[4], 256 // f)
        self.up2 = Up(256 // f, 128 // f)
        self.up3 = Up(128 // f,  64 // f)
        self.up4 = Up( 64 // f,  64 // f)

        # Output heads
        self.outc0 = OutConv(c[4])
        self.outc1 = OutConv(256 // f)
        self.outc2 = OutConv(128 // f)
        self.outc3 = OutConv( 64 // f)
        self.outc4 = OutConv( 64 // f)

        self._init_weights()

    # Xavier init
    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, (nn.Conv2d, nn.ConvTranspose2d)):
                nn.init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    def forward(self, x):
        # Encoder s = skip connect
        s4 = self.inc(x)
        s3 = self.down1(s4)
        s2 = self.down2(s3)
        s1 = self.down3(s2)
        b  = self.down4(s1)

        # Output bottom
        y0 = self.outc0(b)

        # Decoder
        d1 = self.up1(b,  s1)
        y1 = self.outc1(d1)

        d2 = self.up2(d1, s2)
        y2 = self.outc2(d2)

        d3 = self.up3(d2, s3)
        y3 = self.outc3(d3)

        d4 = self.up4(d3, s4)
        y4 = self.outc4(d4)

        return y0, y1, y2, y3, y4


# ---------------------------------------------------------------------------
# Loss
# ---------------------------------------------------------------------------
def dice_loss(pred, target, eps=1e-6):
    pred   = pred.float().flatten(1)
    target = target.float().flatten(1)
    inter  = (pred * target).sum(1)
    return 1 - (2 * inter + eps) / (pred.sum(1) + target.sum(1) + eps)


def gate_loss(outputs, mask):
    weights = [4, 2, 1, 1, 1]
    total   = 0.0
    for w, pred in zip(weights, outputs):
        gt = F.interpolate(mask.float(), size=pred.shape[2:], mode="nearest")
        d  = dice_loss(pred, gt).mean()
        b  = F.binary_cross_entropy(pred, gt)
        total = total + w * (d + 2 * b)
    return total


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------
def corners_to_mask(label_path: str, out_size: int) -> np.ndarray:
    """ Build a binary segmentation mask from a label file."""
    
    mask = np.zeros((out_size, out_size), dtype=np.uint8)
    if not os.path.exists(label_path):
        return mask

    with open(label_path) as f:
        for line in f:
            vals = list(map(float, line.split()))
            if len(vals) < 17:
                continue

            def visible_px(start, n):
                pts = []
                for k in range(n):
                    b = start + k * 3
                    vx, vy, vis = vals[b], vals[b+1], vals[b+2]
                    if vis > 0:
                        px = int(np.clip(vx * out_size, 0, out_size - 1))
                        py = int(np.clip(vy * out_size, 0, out_size - 1))
                        pts.append([px, py])
                return pts

            inner_pts = visible_px(5, 4)

            if len(vals) >= 29:
                # Ring mask: outer filled − inner filled
                outer_pts = visible_px(17, 4)
                if len(outer_pts) >= 3:
                    cv2.fillPoly(mask, [np.array(outer_pts, np.int32)], 1)
                if len(inner_pts) >= 3:
                    cv2.fillPoly(mask, [np.array(inner_pts, np.int32)], 0)
            else:
                # Original format: fill inner polygon
                if len(inner_pts) >= 3:
                    cv2.fillPoly(mask, [np.array(inner_pts, np.int32)], 1)
                elif len(inner_pts) == 2:
                    cv2.line(mask, tuple(inner_pts[0]), tuple(inner_pts[1]), 1, thickness=4)
    return mask


SYN_ROOT = os.path.join(_GATE_ROOT, "data", "synthetic")


class GateDataset(Dataset):
    def __init__(self, syn_dirs: list[str], augment: bool = True):
        """
        syn_dirs: list of flat directories (e.g. synthetic_labels/train,
                  synthetic_labels/train_aug) each containing .jpg + .txt pairs.
        """
        self.augment = augment
        self.samples = []

        for d in syn_dirs:
            if not os.path.isdir(d):
                continue
            for f in sorted(os.listdir(d)):
                if not f.endswith(".jpg"):
                    continue
                stem    = os.path.splitext(f)[0]
                img_p   = os.path.join(d, f)
                lbl_p   = os.path.join(d, stem + ".txt")
                if os.path.exists(lbl_p):
                    self.samples.append((img_p, lbl_p))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, lbl_path = self.samples[idx]

        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mask = corners_to_mask(lbl_path, IMG_SIZE)

        # Resize image to 384×384
        img = cv2.resize(img, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_LINEAR)

        if self.augment:
            img, mask = self._augment(img, mask)

        img  = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        mask = torch.from_numpy(mask).unsqueeze(0).float()
        return img, mask

    def _augment(self, img, mask):
        # Random horizontal flip
        if random.random() < 0.5:
            img  = cv2.flip(img, 1)
            mask = cv2.flip(mask, 1)

        # Random affine: rotation ±15°, translation ±10%, scale 0.9–1.1
        angle  = random.uniform(-15, 15)
        tx     = random.uniform(-0.1, 0.1) * IMG_SIZE
        ty     = random.uniform(-0.1, 0.1) * IMG_SIZE
        scale  = random.uniform(0.9, 1.1)
        cx, cy = IMG_SIZE / 2, IMG_SIZE / 2
        M = cv2.getRotationMatrix2D((cx, cy), angle, scale)
        M[0, 2] += tx
        M[1, 2] += ty
        img  = cv2.warpAffine(img,  M, (IMG_SIZE, IMG_SIZE), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REFLECT)
        mask = cv2.warpAffine(mask, M, (IMG_SIZE, IMG_SIZE), flags=cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_REFLECT)

        # HSV colour jitter
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV).astype(np.float32)
        img_hsv[..., 0] = (img_hsv[..., 0] + random.uniform(-10, 10)) % 180
        img_hsv[..., 1] = np.clip(img_hsv[..., 1] * random.uniform(0.8, 1.2), 0, 255)
        img_hsv[..., 2] = np.clip(img_hsv[..., 2] * random.uniform(0.8, 1.2), 0, 255)
        img = cv2.cvtColor(img_hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)

        # Motion blur
        k = random.choice(range(5, 16, 2))
        kernel = np.zeros((k, k), dtype=np.float32)
        kernel[k // 2, :] = 1.0 / k
        angle_blur = random.uniform(0, 180)
        rot = cv2.getRotationMatrix2D((k // 2, k // 2), angle_blur, 1)
        kernel = cv2.warpAffine(kernel, rot, (k, k))
        kernel /= kernel.sum() + 1e-6
        img = cv2.filter2D(img, -1, kernel)

        # Gaussian noise
        noise = np.random.normal(0, random.uniform(0, 15), img.shape).astype(np.float32)
        img   = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        return img, mask


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------
def train(epochs: int = 100, batch_size: int = 16, base_lr: float = 1e-3, use_aug: bool = True):
    
    model = GateNet(f=4).to(DEVICE)
    
    train_dirs = [os.path.join(SYN_ROOT, "train")]
    val_dirs   = [os.path.join(SYN_ROOT, "val")]
    if use_aug:
        aug_train = os.path.join(SYN_ROOT, "train_aug")
        aug_val   = os.path.join(SYN_ROOT, "val_aug")
        if os.path.isdir(aug_train):
            train_dirs.append(aug_train)
        if os.path.isdir(aug_val):
            val_dirs.append(aug_val)

    train_ds = GateDataset(train_dirs, augment=True)
    val_ds = GateDataset(val_dirs,   augment=False)
    
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_ds,   batch_size=batch_size, shuffle=False, num_workers=2, pin_memory=True)
    print(f"Train: {len(train_ds)} samples  Val: {len(val_ds)} samples")

    optimizer = torch.optim.AdamW(model.parameters(), lr=base_lr)

    # LR scheduler
    milestones = [10, 33, 66, 90]
    gamma = math.sqrt(0.1)
    scheduler  = torch.optim.lr_scheduler.MultiStepLR(optimizer, milestones=milestones, gamma=gamma)
    best_val = float("inf")
    
    # train
    for epoch in range(1, epochs + 1):
        
        model.train()
        train_loss = 0.0
        for imgs, masks in tqdm(train_loader, desc=f"Epoch {epoch:3d}/{epochs} [train]", leave=False):
            imgs, masks = imgs.to(DEVICE), masks.to(DEVICE)
            optimizer.zero_grad()
            outputs = model(imgs)
            loss    = gate_loss(outputs, masks)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * imgs.size(0)

        train_loss /= len(train_ds)
        scheduler.step()

        # validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for imgs, masks in tqdm(val_loader, desc=f"Epoch {epoch:3d}/{epochs} [val]  ", leave=False):
                imgs, masks = imgs.to(DEVICE), masks.to(DEVICE)
                outputs  = model(imgs)
                val_loss += gate_loss(outputs, masks).item() * imgs.size(0)
        val_loss /= len(val_ds)

        lr_now = scheduler.get_last_lr()[0]
        print(f"Epoch {epoch:3d}/{epochs}  lr={lr_now:.2e}  "
              f"train={train_loss:.4f}  val={val_loss:.4f}")

        if val_loss < best_val:
            best_val = val_loss
            ckpt_dir = os.path.join(_GATE_ROOT, "checkpoints")
            os.makedirs(ckpt_dir, exist_ok=True)
            torch.save({"epoch": epoch, "model": model.state_dict(),
                        "optimizer": optimizer.state_dict(),
                        "val_loss": val_loss}, os.path.join(ckpt_dir, "best_gatenet.pth"))
            print(f" best_gatenet -> {os.path.join(ckpt_dir, 'best_gatenet.pth')}")

    print(f"\nDone. Best val loss: {best_val:.4f}")
    return model


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Train GateNet")
    p.add_argument("--epochs",     type=int,   default=100)
    p.add_argument("--batch-size", type=int,   default=16)
    p.add_argument("--lr",         type=float, default=1e-3)
    p.add_argument("--no-aug",     action="store_true",
                   help="skip train_aug/val_aug, use curated data only")
    args = p.parse_args()

    train(
        epochs=args.epochs,
        batch_size=args.batch_size,
        base_lr=args.lr,
        use_aug=not args.no_aug,
    )
