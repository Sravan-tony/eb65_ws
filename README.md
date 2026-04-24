# Installation Guide

# System Requirements

- **Operating System**  : Recommended Ubunutu 18.04 or later

## 1. Creating a Virtual Environment
It is recommended to run training or deployment programs in a virtual environment. Conda is recommended for creating virtual environments. If Conda is already installed on your system, you can skip step 1.1.

### 1.1 Download and Install MiniConda
MiniConda is a lightweight distribution of Conda, suitable for creating and managing virtual environments. Use the following commands to download and install:

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```
After installation, initialize Conda:

```bash
~/miniconda3/bin/conda init --all
source ~/.bashrc
```

### 1.2 Create a New Environment

Use the following command to create a virtual environment:

```bash
conda create -n eb65-env python=3.8
```

### 1.3 Activate the Virtual Environment

```bash
conda activate eb65-env
```
### 1.4 Install mujoco and pygame
```bash
pip3 install mujoco

pip3 install pygame
```
---
