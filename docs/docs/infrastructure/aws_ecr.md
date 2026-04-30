# AWS Elastic Container Registry (ECR)

**Page Owners:** [Mate Tirpak](https://github.com/matetirpak)

---

AWS ECR is used to store and distribute Docker images for all ETHRC Drone Racing environments. Images are built automatically by CI and can also be pushed manually via the Makefile.

---

## Important (Temporary)

If you haven't configured AWS yet, you can use this access key to grant yourself read-only access to ECR. This will be removed in the future, please make sure you have access to the AWS user of your respective team (this will also grant you write access).

Access Key: [AWS ACCESS KEY]

Secret Access Key: [AWS SECRET ACCESS KEY]


## Repository Structure

There are two categories of repositories, both rooted under `aeronautics/droneracing/`:

| Category | Path Pattern | Purpose |
|----------|-------------|---------|
| CI builds | `aeronautics/droneracing/<branch>/<target>` | Automatically built and pushed by GitHub Actions on every push |
| Manual builds | `aeronautics/droneracing` | Manually built and pushed using the Makefile |

### CI Repository Examples

```
aeronautics/droneracing/main/dev
aeronautics/droneracing/main/perception
aeronautics/droneracing/main/mpc
aeronautics/droneracing/main/rl
aeronautics/droneracing/main/docs
aeronautics/droneracing/feature/my-branch/dev
```

### Image Targets

| Target | Description |
|--------|-------------|
| `dev` | Full environment — ROS 2, Gazebo, perception, MPC, RL |
| `perception` | Gate Detection, OpenVINS, Vicon receiver |
| `mpc` | MAVSDK, ACADOS, drone control stack |
| `rl` | MPC stack + reinforcement learning (stable-baselines3, PyTorch) |
| `docs` | MkDocs documentation server |

---

## Registry Details

| Property | Value |
|----------|-------|
| Registry | `916780037007.dkr.ecr.us-east-2.amazonaws.com` |
| Region | `us-east-2` |
| Encryption | AES-256 |
| Scan on push | Enabled |

---

## Image Tags

### CI Images
Each CI build produces two tags per image:

| Tag | Example | Description |
|-----|---------|-------------|
| Short SHA | `a1b2c3` | Pinned to a specific commit |
| `latest` | `latest` | Always points to the most recent build |

### Manual Images
Manual builds use `<target>-<version>` tags to avoid collisions within the shared repo:

| Tag | Example | Description |
|-----|---------|-------------|
| Versioned | `dev-0.1.0` | Pinned release |
| Latest | `dev-latest` | Most recent manual push |

---

## Lifecycle Policy

Each CI repository has an automated cleanup policy to control storage costs:

- **BuildCache**: only 1 image retained (used by Docker BuildKit layer caching)
- **All other images**: only the 4 most recent retained (1 buildcache + 3 real images)

---

## Authentication

CI uses OIDC (OpenID Connect) to assume an AWS IAM role — no long-lived credentials are stored.
The role ARN is stored as the `AWS_ROLE_ARN` repository secret.

For local use, authenticate with:

```bash
make login
```

This runs:

```bash
aws ecr get-login-password --region us-east-2 | docker login --username AWS --password-stdin 916780037007.dkr.ecr.us-east-2.amazonaws.com
```

Authentication tokens are valid for **12 hours**.

---

## Working with Images Locally

All operations are available through the Makefile. Run `make help` for a full list.

### List all repositories

```bash
make repos
```

### List all tags in a repository

```bash
# Manual repo
make tags

# CI repo
make tags REPO=aeronautics/droneracing/main/dev
```

### Pull an image

```bash
# From CI (latest dev build on main)
make pull REPO=aeronautics/droneracing/main/dev TAG=latest

# From manual repo
make pull REPO=aeronautics/droneracing TAG=dev-latest
```

### Run an image interactively

```bash
make run REPO=aeronautics/droneracing/main/dev TAG=latest

# With port forwarding (e.g. for docs)
make run REPO=aeronautics/droneracing/main/docs TAG=latest PORTS=8000:8000
```

---

## Using ECR Images with Docker Compose

If you want to use a pre-built image from ECR with `docker compose` (e.g., to skip the slow local build of the `dev` target), you can use the **Tag Trick**.

### The "Tag Trick"
Docker Compose identifies images locally by the project name and service name (e.g., `ethrc-drone-racing-dev`). To force Compose to use an ECR image:

1.  **Pull the ECR image**:
    ```bash
    make pull REPO=aeronautics/droneracing/main/dev TAG=latest
    ```
2.  **Tag it locally** to match the Compose name:
    ```bash
    docker tag 916780037007.dkr.ecr.us-east-2.amazonaws.com/aeronautics/droneracing/main/dev:latest ethrc-drone-racing-dev:latest
    ```
3.  **Start Compose** as usual:
    ```bash
    docker compose up -d dev
    ```
Now, `docker compose exec -it dev bash` will put you inside a container running the official CI build.

### Offline / Local Building (AWS Unreachable)

If you are working offline or AWS ECR is unreachable, your `docker compose build dev` command will fail because it cannot pull the `shared-perception-controls-deps` base image from AWS.

To resolve this, you can build the base image locally. `docker-compose.yaml` includes a special `base` service configured exactly for this:

1. **Build the base image locally:**
   ```bash
   docker compose build base
   ```
   *This automatically builds `Dockerfile.base` and tags it with the AWS URL.*

2. **Build the dev container (without pull):**
   ```bash
   docker compose build dev
   ```
   *(Ensure you do **not** use the `--pull` flag, otherwise Docker will ignore the local tag and try to contact AWS anyway).*

### Build and push manually

```bash
# Build
make build TARGET=perception VERSION=0.2.0 DOCKERFILE=docker/Dockerfile

# Push (will fail if version already exists)
make push TARGET=perception VERSION=0.2.0
```

---

## CI Workflow

The `aws_ecr.yml` GitHub Actions workflow handles automated builds. It:

1. Detects which targets have changed by diffing against the last successful run's commit SHA
2. If no prior successful run exists (new branch), **all targets are built**
3. Builds only affected targets in parallel
4. Pushes each image with a short SHA tag and `latest`
5. Uses Docker BuildKit registry cache stored in ECR to accelerate incremental builds

### Triggering a full rebuild

If new targets are added to `auto_build_targets.json`, trigger a forced rebuild via the CLI:

```bash
gh workflow run aws_ecr.yml --ref <your-branch> -f force_all=true
```

---

## Adding a New Image Target

1. Add a new stage to [docker/Dockerfile](../../docker/Dockerfile) or create a new Dockerfile
2. Add a new entry to [.github/workflows/auto_build_targets.json](../../.github/workflows/auto_build_targets.json):

```json
{
  "image_name": "my-target",
  "dockerfile": "docker/Dockerfile",
  "target": "my-target",
  "trigger_paths": ["docker/Dockerfile", "src/my-module/"]
}
```
3. Trigger a forced rebuild to push the new image for the first time
