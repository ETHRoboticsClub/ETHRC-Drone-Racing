##############################################################
# Makefile for building and pushing Docker images to AWS ECR #
# Manual builds use a single repo: aeronautics/droneracing   #
# Tags follow the pattern: <target>-<version>                #
# CI builds use per-branch repos (see aws_ecr.yml).          #
##############################################################

# ─── Config ───────────────────────────────────────────────────────────
ECR_REGISTRY  = 916780037007.dkr.ecr.us-east-2.amazonaws.com
ECR_REPO      = aeronautics/droneracing
REGION        = us-east-2
DOCKERFILE    = docker/Dockerfile

# Defaults — override on the command line as needed:
#   make build TARGET=perception VERSION=0.2
#   make push  TARGET=mpc VERSION=0.2
TARGET        ?=
VERSION       ?=
TAG           ?= latest
AWS_PROFILE   ?=

# Derived — image tag is <target>-<version> to avoid collisions in the shared repo
IMAGE_TAG     = $(TARGET)-$(VERSION)
FULL_IMAGE    = $(ECR_REGISTRY)/$(ECR_REPO)
# For pull/run from CI repos: make run REPO=aeronautics/droneracing/main/dev TAG=latest
REPO          ?= $(ECR_REPO)
RUN_IMAGE     = $(ECR_REGISTRY)/$(REPO):$(TAG)
PROFILE_FLAG  = $(if $(AWS_PROFILE),--profile $(AWS_PROFILE),)

# ─── Guards ───────────────────────────────────────────────────────────
guard-%:
	@if [ -z '$($(*))'  ]; then \
	  echo "ERROR: $* is required. Usage: make <target> $*=<value>"; \
	  exit 1; \
	fi

# ─── Targets ──────────────────────────────────────────────────────────
.PHONY: login build check-version push pull run repos list tags all help

help: ## Show this help
	@echo "Usage: make <target> [TARGET=dev|perception|mpc|rl|docs] [VERSION=x.y] [TAG=latest]"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*##' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*## "}; {printf "  make %-18s %s\n", $$1, $$2}'
	@echo ""
	@echo "Repo:  $(FULL_IMAGE)"
	@echo "Tag:   $(IMAGE_TAG)"

login: ## Authenticate Docker to ECR (valid 12h)
	aws ecr get-login-password --region $(REGION) $(PROFILE_FLAG) \
	  | docker login --username AWS --password-stdin $(ECR_REGISTRY)

build: ## Build TARGET image locally (uses docker/Dockerfile by default)
	docker build -f $(DOCKERFILE) --target $(TARGET) -t $(TARGET):$(VERSION) .

check-version: guard-VERSION ## Fail if <target>-<version> tag already exists in ECR
	@echo "Checking if $(ECR_REPO):$(IMAGE_TAG) already exists in ECR..."
	@if aws ecr describe-images \
	    --repository-name $(ECR_REPO) \
	    --image-ids imageTag=$(IMAGE_TAG) \
	    --region $(REGION) \
	    $(PROFILE_FLAG) > /dev/null 2>&1; then \
	  echo "ERROR: $(ECR_REPO):$(IMAGE_TAG) already exists. Bump VERSION to push a new image."; \
	  exit 1; \
	fi
	@echo "OK — tag is available."

push: guard-VERSION check-version ## Tag and push TARGET image to ECR (blocks if version exists)
	docker tag $(TARGET):$(VERSION) $(FULL_IMAGE):$(IMAGE_TAG)
	docker tag $(TARGET):$(VERSION) $(FULL_IMAGE):$(TARGET)-latest
	docker push $(FULL_IMAGE):$(IMAGE_TAG)
	docker push $(FULL_IMAGE):$(TARGET)-latest
	@echo ""
	@echo "Pushed:"
	@echo "  $(FULL_IMAGE):$(IMAGE_TAG)"
	@echo "  $(FULL_IMAGE):$(TARGET)-latest"

pull: ## Pull image — uses REPO+TAG (e.g. make pull REPO=aeronautics/droneracing/main/dev TAG=latest)
	docker pull $(RUN_IMAGE)

run: pull ## Pull and run image interactively (same REPO+TAG as pull, optional PORTS=8000:8000)
	docker run -it --rm $(if $(PORTS),-p $(PORTS),) $(RUN_IMAGE)

repos: ## List all ECR repositories under aeronautics/droneracing (CI + manual)
	@aws ecr describe-repositories \
	    --region $(REGION) \
	    $(PROFILE_FLAG) \
	    --query "repositories[*].repositoryName" \
	    --output table

list: ## List all tags in the shared manual ECR repo (aeronautics/droneracing)
	@aws ecr describe-images \
	    --repository-name $(ECR_REPO) \
	    --region $(REGION) \
	    $(PROFILE_FLAG) \
	    --query 'sort_by(imageDetails,&imagePushedAt)[?imageTags!=`null`].{Tags:join(`, `,imageTags),Pushed:imagePushedAt,SizeMB:to_string(imageSizeInBytes)}' \
	    --output table

tags: ## List all tags in any REPO (default: shared manual repo, override with REPO=...)
	@aws ecr describe-images \
	    --repository-name $(REPO) \
	    --region $(REGION) \
	    $(PROFILE_FLAG) \
	    --query 'sort_by(imageDetails,&imagePushedAt)[?imageTags!=`null`].{Tags:join(`, `,imageTags),Pushed:imagePushedAt,SizeMB:to_string(imageSizeInBytes)}' \
	    --output table

all: login build push ## Login, build, and push in one go (requires VERSION=x.y)
