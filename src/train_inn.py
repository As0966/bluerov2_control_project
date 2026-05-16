import os
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import joblib

class INNControllerNet(nn.Module):
    def __init__(self, input_dim=16):
        super().__init__()
        # Matches Section 8: e_eta(4), e_nu(4), nu_d(4), nu(4) = 16
        self.net = nn.Sequential(
            nn.Linear(20, 64),
            nn.Tanh(),
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, 64),
            nn.Tanh(),
            nn.Linear(64, 4),
        )

    def forward(self, x):
        return self.net(x)

def train():
    os.makedirs("data", exist_ok=True)
    os.makedirs("figures", exist_ok=True)

    print("Loading NEMESIS-INN Training Data...")
    df = pd.read_csv("data/inn_training_data.csv")

    input_cols = [f"f{i}" for i in range(20)]
    output_cols = ["tau_u", "tau_v", "tau_w", "tau_r"]

    X = df[input_cols].values.astype(np.float32)
    Y = df[output_cols].values.astype(np.float32)

    X_train, X_test, Y_train, Y_test = train_test_split(
        X, Y, test_size=0.2, random_state=42
    )

    x_scaler = StandardScaler()
    y_scaler = StandardScaler()

    X_train_s = x_scaler.fit_transform(X_train)
    X_test_s = x_scaler.transform(X_test)
    Y_train_s = y_scaler.fit_transform(Y_train)
    Y_test_s = y_scaler.transform(Y_test)

    X_train_t = torch.tensor(X_train_s, dtype=torch.float32)
    Y_train_t = torch.tensor(Y_train_s, dtype=torch.float32)
    X_test_t = torch.tensor(X_test_s, dtype=torch.float32)
    Y_test_t = torch.tensor(Y_test_s, dtype=torch.float32)

    model = INNControllerNet(input_dim=len(input_cols))
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    epochs = 1500
    train_losses = []
    test_losses = []

    print(f"Starting Training ({epochs} epochs)...")
    for epoch in range(epochs):
        model.train()
        optimizer.zero_grad()
        pred = model(X_train_t)
        loss = loss_fn(pred, Y_train_t)
        loss.backward()
        optimizer.step()

        train_losses.append(loss.item())

        if epoch % 50 == 0:
            model.eval()
            with torch.no_grad():
                test_pred = model(X_test_t)
                test_loss = loss_fn(test_pred, Y_test_t)
                test_losses.append(test_loss.item())
            print(f"Epoch {epoch:04d} | train_loss={loss.item():.6f} | val_loss={test_loss.item():.6f}")
        else:
            if len(test_losses) > 0:
                test_losses.append(test_losses[-1])
            else:
                test_losses.append(0.0)

    # Save models
    torch.save(model.state_dict(), "data/inn_model.pth")
    joblib.dump(x_scaler, "data/inn_x_scaler.pkl")
    joblib.dump(y_scaler, "data/inn_y_scaler.pkl")

    # Generate Figures
    plt.figure(figsize=(10, 6))
    plt.plot(train_losses, label='Train Loss')
    plt.plot(test_losses, label='Val Loss')
    plt.yscale('log')
    plt.title("INN Cost Plot (Training Progress)")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss")
    plt.legend()
    plt.grid(True, which="both", alpha=0.3)
    plt.savefig("figures/inn_cost_plot.png")
    
    print("\nTraining Complete. Model and Figures saved.")

if __name__ == "__main__":
    train()
