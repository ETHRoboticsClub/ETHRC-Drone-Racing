# Getting Started

This guide will help you set up the ETHRC Drone Racing environment using Docker.

## Prerequisites

- **Host OS:** Ubuntu/Linux recommended.
- **Software:** Docker Engine + Docker Compose v2 in Rootless mode. ([Installation guide](https://docs.docker.com/engine/install/))

## Quick Start

!!! note
    After step 3, the container will be running in the background. You can stop it with `docker compose down`.

### Step 0: Set up SSH for GitHub

We need SSH so `git clone` works smoothly without typing passwords. If you haven't done this, follow [GitHub's guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

### Step 1: Clone the Repository

Clone the repository with submodules to include PX4:

```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
```

### Step 2: Build the Docker Image

Build the development environment:

```bash
docker compose build
```

### Step 3: Start Services

Enable GUI access, start the container, and open a shell:

```bash
# Allow container GUI apps to open windows on your desktop (X11)
xhost +local:root

# Start the container (detached) and open a shell inside
docker compose up -d
docker exec -it erc-aeronautics bash
```

### Step 4: Run Simulation

Inside the container, run the default simulation:

```bash
# Default x500 in the default world
cd /ws/px4/PX4-Autopilot
make px4_sitl gz_x500
```
