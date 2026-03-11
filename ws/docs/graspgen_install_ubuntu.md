# GraspGen install (Ubuntu, inside Docker container)

This guide assumes you run **inside the same Docker container** where ROS and the workspace live. Only `ws/deps` and `ws/src` are mounted, so anything you install under `deps/` (including the venv) **persists** across container restarts; the rest of the image is ephemeral.

The container must have **GPU access** (compose `deploy.resources.reservations.devices` for nvidia, and nvidia-container-toolkit on the host). Run `nvidia-smi` inside the container to confirm.

The image must also include the **CUDA toolkit** (nvcc, headers, libs) and `CUDA_HOME` so that step 5 (`install_pointnet.sh`) can build pointnet2_ops. If `which nvcc` is empty, add the CUDA toolkit to your Dockerfile as in the section below.

---

## 1. Create venv with Python 3.10 (under mounted `deps`)

GraspGen uses **torch 2.1.0**, which has no wheels for Python 3.12 (cp312). The venv **must** use **Python 3.10** so the cp310 wheels in the next step are accepted. The image must have `python3.10` installed (e.g. via Dockerfile and deadsnakes PPA).

```bash
cd /home/ros/ws/deps/GraspGen
python3.10 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

Because `deps` is volumed, `.venv` persists. In new shells, activate with:

```bash
source /home/ros/ws/deps/GraspGen/.venv/bin/activate
```

---

## 2. Install PyTorch 2.1.0 + torchvision 0.16.0 (per GraspGen README)

GraspGen officially uses **torch==2.1.0** and **torchvision==0.16.0**. The default pip index URL for cu118/cu121 no longer lists 2.1.0, but the **wheels are still available** from PyTorch’s direct directory. Install by **wheel URL** so you stay on the versions GraspGen documents.

The venv from step 1 must be Python 3.10; the wheels below are **cp310**. (PyTorch 2.1.0 does not provide cp312 wheels.)

**Option A – CUDA 12.1 (cu121):**

```bash
pip install https://download.pytorch.org/whl/cu121/torch-2.1.0%2Bcu121-cp310-cp310-linux_x86_64.whl
pip install https://download.pytorch.org/whl/cu121/torchvision-0.16.0%2Bcu121-cp310-cp310-linux_x86_64.whl
```

**Option B – CUDA 11.8 (cu118):**

```bash
pip install https://download.pytorch.org/whl/cu118/torch-2.1.0%2Bcu118-cp310-cp310-linux_x86_64.whl
pip install https://download.pytorch.org/whl/cu118/torchvision-0.16.0%2Bcu118-cp310-cp310-linux_x86_64.whl
```

If a torchvision URL 404s, check [PyTorch previous versions](https://pytorch.org/get-started/previous-versions/) for the exact path. Use the same CUDA variant (cu121 or cu118) for the next step.

---

## 3. Install PyG (torch-cluster, torch-scatter)

GraspGen’s README uses the PyG wheel index for **torch 2.1.0**. Match the CUDA variant you used above:

**For cu121:**

```bash
pip install torch-cluster torch-scatter -f https://data.pyg.org/whl/torch-2.1.0+cu121.html
```

**For cu118:**

```bash
pip install torch-cluster torch-scatter -f https://data.pyg.org/whl/torch-2.1.0+cu118.html
```

---

## 4. Install GraspGen (editable)

From the repo root (with venv active):

```bash
cd /home/ros/ws/deps/GraspGen
pip install -e .
```

Pip will see `torch==2.1.0` already satisfied and install the rest from PyPI.

---

## 5. Install PointNet (pointnet2_ops)

Requires a C++ compiler and the **CUDA toolkit** in the container (`nvcc`, headers, libs; `CUDA_HOME` set). If your image doesn’t have it, add the CUDA 12.1 toolkit to your Dockerfile (see **Dockerfile: CUDA toolkit** below). From `deps/GraspGen`:

```bash
cd /home/ros/ws/deps/GraspGen
./install_pointnet.sh
```

The script sets `TORCH_CUDA_ARCH_LIST="8.6"` (Ampere). For a different GPU, set the arch and run again, e.g.:

```bash
export TORCH_CUDA_ARCH_LIST="7.5"   # e.g. RTX 20xx
./install_pointnet.sh
```

---

## 6. Verify

```bash
pip install pytest
cd /home/ros/ws/deps/GraspGen
python tests/test_inference_installation.py
```

If you see generated grasps and no errors, the install is good.

For **Franka Panda** (ROS + Isaac Sim grasp eval), use the gripper config from the checkpoints repo: `deps/GraspGenModels/checkpoints/graspgen_franka_panda.yml` (e.g. when running `demo_object_mesh.py` or your integration).

---

## Dockerfile: CUDA toolkit (for step 5)

If the container has no CUDA toolkit (`which nvcc` empty), add the following **after** the Python 3.10 block and **before** the ROS deps. Base image is assumed **Ubuntu 24.04** (e.g. `osrf/ros:jazzy-desktop-full`). For Ubuntu 22.04, change `ubuntu2404` to `ubuntu2204` in the keyring URL.

```dockerfile
# CUDA 12.1 toolkit for pointnet2_ops (GraspGen); driver comes from host via nvidia-container-toolkit
RUN apt-get update && apt-get install -y wget \
    && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && rm -f cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    && apt-get install -y cuda-toolkit-12-1 \
    && rm -rf /var/lib/apt/lists/*
ENV CUDA_HOME=/usr/local/cuda
ENV PATH="${CUDA_HOME}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"
```

Then rebuild the image and run step 5 (`./install_pointnet.sh`) inside the GraspGen venv. Your **RTX 3060 Ti** (Ampere 8.6) is already the default in the install script; no need to set `TORCH_CUDA_ARCH_LIST`.

---

## Summary

| Step | Command / action |
|------|-------------------|
| 1 | `cd /home/ros/ws/deps/GraspGen` → `python3.10 -m venv .venv` → `source .venv/bin/activate` → `pip install --upgrade pip` |
| 2 | Install torch 2.1.0 + torchvision 0.16.0 via direct wheel URLs (cu121 or cu118; match cp3X to your Python). |
| 3 | `pip install torch-cluster torch-scatter -f https://data.pyg.org/whl/torch-2.1.0+cu121.html` (or +cu118) |
| 4 | `cd /home/ros/ws/deps/GraspGen` → `pip install -e .` |
| 5 | `./install_pointnet.sh` (set `TORCH_CUDA_ARCH_LIST` if not Ampere) |
| 6 | `pip install pytest` → `python tests/test_inference_installation.py` |

All of this is under `deps/`, so it survives container restarts. Use the venv whenever you run GraspGen scripts.
