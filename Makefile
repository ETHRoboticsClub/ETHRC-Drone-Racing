##############################################################
# Makefile for building and pushing Docker images to AWS ECR #
##############################################################

# ─── Config ───────────────────────────────────────────────────────────
ECR_REGISTRY  = 916780037007.dkr.ecr.us-east-2.amazonaws.com
ECR_REPO      = aeronautics/droneracing
REGION        = us-east-2

# Required for build/push — must be provided on the command line:
#   make build push IMAGE=base-mavsdk-acados VERSION=0.1 DOCKERFILE=Dockerfile.mavsdk_acados
IMAGE         ?=
VERSION       ?=
DOCKERFILE    ?=
AWS_PROFILE   ?=

# Derived
ECR_TAG       = $(ECR_REGISTRY)/$(ECR_REPO):$(IMAGE)-$(VERSION)
ECR_LATEST    = $(ECR_REGISTRY)/$(ECR_REPO):$(IMAGE)-latest
PROFILE_FLAG  = $(if $(AWS_PROFILE),--profile $(AWS_PROFILE),)

# ─── Guards ───────────────────────────────────────────────────────────
guard-%:
	@if [ -z '$($(*))'  ]; then \
	  echo "ERROR: $* is required. Usage: make <target> $*=<value>"; \
	  exit 1; \
	fi

# ─── Targets ──────────────────────────────────────────────────────────
.PHONY: login build push check-version list run help

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*##' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*## "}; {printf "  make %-18s %s\n", $$1, $$2}'

login: ## Authenticate Docker to ECR (valid 12h)
	aws ecr get-login-password --region $(REGION) $(PROFILE_FLAG) \
	  | docker login --username AWS --password-stdin $(ECR_REGISTRY)

build: guard-IMAGE guard-VERSION guard-DOCKERFILE ## Build the image locally
	docker build -f $(DOCKERFILE) -t $(IMAGE):$(VERSION) .

check-version: guard-IMAGE guard-VERSION ## Fail if this version already exists in ECR
	@echo "Checking if $(IMAGE)-$(VERSION) already exists in ECR..."
	@if aws ecr describe-images \
	    --repository-name $(ECR_REPO) \
	    --image-ids imageTag=$(IMAGE)-$(VERSION) \
	    --region $(REGION) \
	    $(PROFILE_FLAG) > /dev/null 2>&1; then \
	  echo "ERROR: $(IMAGE)-$(VERSION) already exists. Bump VERSION to push a new image."; \
	  exit 1; \
	fi
	@echo "OK — tag is available."

push: guard-IMAGE guard-VERSION check-version ## Tag and push to ECR (blocks if version exists)
	docker tag $(IMAGE):$(VERSION) $(ECR_TAG)
	docker tag $(IMAGE):$(VERSION) $(ECR_LATEST)
	docker push $(ECR_TAG)
	docker push $(ECR_LATEST)
	@echo ""
	@echo "Pushed:"
	@echo "  $(ECR_TAG)"
	@echo "  $(ECR_LATEST)"

list: ## List all image tags in the ECR repo
	@aws ecr describe-images \
	    --repository-name $(ECR_REPO) \
	    --region $(REGION) \
	    $(PROFILE_FLAG) \
	    --query 'sort_by(imageDetails,&imagePushedAt)[*].{Tags:join(`, `,imageTags),Pushed:imagePushedAt,Size:imageSizeInBytes}' \
	    --output table

run: guard-IMAGE guard-VERSION ## Run an image from ECR interactively
	docker run -it --rm $(ECR_TAG)

all: login build push ## Login, build, and push in one go
