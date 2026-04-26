"""
Example: Training a simple imitation learning policy using RealSense camera data.

This demonstrates how to use the HDF5 logged teleoperation data with camera frames
for training a visuomotor policy network using PyTorch.

The policy learns to predict robot arm poses from RGB observations + hand state.
"""

import argparse
import os
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split

try:
    from replay_with_camera import HDF5CameraDataLoader
except ImportError:
    print("Error: replay_with_camera.py not found. Ensure it's in the same directory.")
    exit(1)


class TeleoperationImitationDataset(Dataset):
    """
    Load teleoperation trajectories with context windows for sequential prediction.
    
    Each sample is a short sequence of observations that predicts the next action.
    """

    def __init__(self, hdf5_path, context_length=5, action_horizon=1):
        """
        Args:
            hdf5_path: Path to HDF5 teleoperation log
            context_length: Number of past frames to include (context window)
            action_horizon: How many steps ahead to predict (1 = next action)
        """
        self.loader = HDF5CameraDataLoader(hdf5_path)
        self.context_length = context_length
        self.action_horizon = action_horizon
        self.hdf5_path = hdf5_path

        # Usable range: need context_length history + action_horizon lookahead
        self.usable_length = (
            len(self.loader) - context_length - action_horizon + 1
        )

        if self.usable_length <= 0:
            raise ValueError(
                f"Dataset too short. Need at least {context_length + action_horizon} "
                f"samples, got {len(self.loader)}"
            )

        print(f"Dataset initialized: {len(self)} usable samples from {len(self.loader)} total")

    def __len__(self):
        return self.usable_length

    def __getitem__(self, idx):
        """
        Get a sample with context and target action.
        
        Returns:
            dict with:
                - 'rgb': [context_length, C, H, W] tensor (float32, normalized to [0,1])
                - 'depth': [context_length, H, W] tensor (float32, normalized)
                - 'hand_state': [context_length, 20] tensor (current hand configuration)
                - 'target_pose': [6] tensor (target arm end-effector pose)
        """
        # Context window: indices from idx to idx + context_length
        context_indices = range(idx, idx + self.context_length)

        # Target index: context_length + action_horizon steps ahead
        target_idx = idx + self.context_length + self.action_horizon - 1

        # Load trajectory
        traj = self.loader.get_trajectory(idx, target_idx)

        # Extract context
        context_rgb = []
        context_depth = []
        context_hand = []

        for i, idx_i in enumerate(context_indices):
            sample = self.loader.get_sample(idx_i)

            # RGB: BGR [H, W, 3] uint8 -> RGB [3, H, W] float [0, 1]
            if sample.get("camera/rgb") is not None:
                rgb = sample["camera/rgb"][:, :, ::-1]  # BGR to RGB
                rgb = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
            else:
                # Black frame if missing
                rgb = torch.zeros(3, 480, 640)

            context_rgb.append(rgb)

            # Depth: [H, W] uint16 (mm) -> [H, W] float32 (meters, normalized)
            if sample.get("camera/depth") is not None:
                depth = sample["camera/depth"].astype(np.float32) / 1000.0  # mm to m
                # Normalize to [0, 1] assuming typical depth range 0.1-2.0 meters
                depth = torch.from_numpy(depth).clamp(0.1, 2.0) / 2.0
            else:
                depth = torch.zeros(480, 640)

            context_depth.append(depth)

            # Hand state: 20D joint angles
            hand = sample["hand/manus_joints"]
            hand = torch.from_numpy(hand).float()
            context_hand.append(hand)

        # Get target action (6D arm pose to reach)
        target_sample = self.loader.get_sample(target_idx)
        target_pose = torch.from_numpy(
            target_sample["arm/smoothed_pose"]
        ).float()

        return {
            "rgb": torch.stack(context_rgb),  # [T, 3, H, W]
            "depth": torch.stack(context_depth),  # [T, H, W]
            "hand_state": torch.stack(context_hand),  # [T, 20]
            "target_pose": target_pose,  # [6]
        }


class VisuomotorPolicy(nn.Module):
    """
    Simple CNN + MLP visuomotor policy for imitation learning.
    
    Processes RGB observations through a CNN, combines with hand state,
    and predicts robot arm poses.
    """

    def __init__(self, input_channels=3, hand_dim=20, output_dim=6):
        """
        Args:
            input_channels: Number of input channels (3 for RGB)
            hand_dim: Dimensionality of hand state
            output_dim: Dimensionality of output (6 for arm pose)
        """
        super().__init__()

        # CNN for RGB (simple ResNet-style architecture)
        self.conv1 = nn.Sequential(
            nn.Conv2d(input_channels, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((8, 8)),
        )

        # Calculate flattened size after conv: 64 * 8 * 8
        self.conv_output_size = 64 * 8 * 8

        # MLP for decision making
        self.mlp = nn.Sequential(
            nn.Linear(self.conv_output_size + hand_dim, 256),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, output_dim),
        )

    def forward(self, rgb, hand_state):
        """
        Args:
            rgb: [B, 3, H, W] RGB image tensor
            hand_state: [B, 20] hand joint angles

        Returns:
            [B, 6] predicted arm poses
        """
        # Process image
        conv_features = self.conv1(rgb)  # [B, 64, 8, 8]
        conv_flat = conv_features.view(conv_features.size(0), -1)  # [B, 4096]

        # Concatenate with hand state
        combined = torch.cat([conv_flat, hand_state], dim=1)  # [B, 4096 + 20]

        # Predict arm pose
        output = self.mlp(combined)  # [B, 6]
        return output


def train_epoch(model, train_loader, optimizer, criterion, device):
    """Train for one epoch."""
    model.train()
    total_loss = 0.0

    for batch_idx, batch in enumerate(train_loader):
        rgb = batch["rgb"][:, -1].to(device)  # Use last frame [B, 3, H, W]
        hand_state = batch["hand_state"][:, -1].to(device)  # Use last state [B, 20]
        target_pose = batch["target_pose"].to(device)  # [B, 6]

        # Forward pass
        predicted_pose = model(rgb, hand_state)
        loss = criterion(predicted_pose, target_pose)

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

        if (batch_idx + 1) % 100 == 0:
            print(f"  Batch {batch_idx + 1}/{len(train_loader)}: "
                  f"Loss = {loss.item():.4f}")

    return total_loss / len(train_loader)


def evaluate(model, val_loader, criterion, device):
    """Evaluate model on validation set."""
    model.eval()
    total_loss = 0.0
    pose_error = 0.0

    with torch.no_grad():
        for batch in val_loader:
            rgb = batch["rgb"][:, -1].to(device)
            hand_state = batch["hand_state"][:, -1].to(device)
            target_pose = batch["target_pose"].to(device)

            predicted_pose = model(rgb, hand_state)
            loss = criterion(predicted_pose, target_pose)

            total_loss += loss.item()
            # L2 distance error
            pose_error += torch.norm(predicted_pose - target_pose, dim=1).mean().item()

    avg_loss = total_loss / len(val_loader)
    avg_pose_error = pose_error / len(val_loader)

    return avg_loss, avg_pose_error


def main():
    parser = argparse.ArgumentParser(
        description="Train a simple visuomotor imitation learning policy."
    )
    parser.add_argument(
        "hdf5_path",
        help="Path to HDF5 teleoperation log.",
    )
    parser.add_argument(
        "--context-length", type=int, default=5,
        help="Number of past frames for context (default: 5)",
    )
    parser.add_argument(
        "--batch-size", type=int, default=32,
        help="Batch size for training (default: 32)",
    )
    parser.add_argument(
        "--epochs", type=int, default=50,
        help="Number of training epochs (default: 50)",
    )
    parser.add_argument(
        "--learning-rate", type=float, default=1e-3,
        help="Learning rate (default: 1e-3)",
    )
    parser.add_argument(
        "--val-split", type=float, default=0.2,
        help="Validation split fraction (default: 0.2)",
    )
    parser.add_argument(
        "--output-dir", type=str, default="./model_outputs",
        help="Directory to save model and logs (default: ./model_outputs)",
    )

    args = parser.parse_args()

    # Setup device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Load dataset
    print("\nLoading dataset...")
    dataset = TeleoperationImitationDataset(
        args.hdf5_path,
        context_length=args.context_length,
    )

    # Split into train/val
    val_size = max(1, int(len(dataset) * args.val_split))
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = random_split(
        dataset,
        [train_size, val_size],
    )

    print(f"  Training samples: {len(train_dataset)}")
    print(f"  Validation samples: {len(val_dataset)}")

    # Create data loaders
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=4,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=4,
    )

    # Initialize model
    print("\nInitializing model...")
    model = VisuomotorPolicy(
        input_channels=3,
        hand_dim=20,
        output_dim=6,
    ).to(device)

    total_params = sum(p.numel() for p in model.parameters())
    print(f"  Total parameters: {total_params:,}")

    # Optimizer and loss
    optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)
    criterion = nn.MSELoss()

    # Training loop
    print("\nTraining...")
    best_val_loss = float("inf")
    best_epoch = 0

    for epoch in range(args.epochs):
        print(f"\nEpoch {epoch + 1}/{args.epochs}")

        # Train
        train_loss = train_epoch(
            model, train_loader, optimizer, criterion, device
        )
        print(f"  Train loss: {train_loss:.4f}")

        # Validate
        val_loss, val_pose_error = evaluate(
            model, val_loader, criterion, device
        )
        print(f"  Val loss: {val_loss:.4f}, Pose error: {val_pose_error:.4f}")

        # Save if best
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_epoch = epoch
            model_path = os.path.join(args.output_dir, "best_model.pth")
            torch.save(model.state_dict(), model_path)
            print(f"  Saved best model to {model_path}")

    print(f"\nTraining complete!")
    print(f"Best model at epoch {best_epoch + 1} (val loss: {best_val_loss:.4f})")

    # Save final model
    final_path = os.path.join(args.output_dir, "final_model.pth")
    torch.save(model.state_dict(), final_path)
    print(f"Saved final model to {final_path}")

    # Save model metadata
    metadata = {
        "context_length": args.context_length,
        "best_epoch": best_epoch,
        "best_val_loss": float(best_val_loss),
        "hdf5_source": args.hdf5_path,
    }
    import json
    metadata_path = os.path.join(args.output_dir, "model_metadata.json")
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"Saved metadata to {metadata_path}")


if __name__ == "__main__":
    main()
