# Dockerfile Notes

## Important
The purpose of this README is to avoid confusion when building Dockerfiles. Whenever you change the structure or add new dependencies, please note these down here! ECR should make our life easier but it also introduces complexity to which we have to pay attention.

## Typical Issues
For any issues with AWS ECR please contact Máté.
The goal is to have as much of our images in the cloud as possible to save us a huge amount of time which would otherwise be wasted building them repeatedly. However, this might get confusing and in case something doesn't work as planned, feel free to rebuild the necessary images locally. An example would be a missing ECR image referenced as the base. In this case you can rebuild this base locally and compile your desired Dockerfile by overwriting it to the local one.


## Structure
`untracked/` was previously used for heavy persistent dependencies. As of the recent modularization, these are now ideally hosted in AWS ECR and referenced directly from the main `Dockerfile`.

`Dockerfile` is designed to be lightweight and modular, pulling pre-built heavy dependency images (like `shared-perception-controls-deps`) from ECR by default to optimize build times and CI/CD performance.