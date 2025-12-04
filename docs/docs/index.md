# ETHRC Drone Racing

<p align="center">
  <img src="images/ETHRC-Aeronautics_transparent.png" alt="ETHRC Aeronautics Logo" width="300"/>
</p>

Welcome to the ETHRC Drone Racing project documentation.

## Quick Links

- [**Getting Started**](getting-started.md): Set up your environment and run your first simulation.
- [**Guidelines**](guidelines.md): Learn about our documentation standards and development workflow.

## Project Layout

The project is organized as follows:

- **`docker/`**: Contains the Docker configuration, including the `Dockerfile` used to build the development and simulation environment.
- **`px4/`**: Contains the PX4 Autopilot source code. This is where the flight control logic resides.
- **`docs/`**: The source for this documentation site.
    - **`docs/`**: Markdown content files.
    - **`mkdocs.yml`**: Configuration for the MkDocs site generator.
- **`docker-compose.yaml`**: Defines the services and volumes for running the simulation environment.

## Getting Started

Check out the `README.md` in the root of the repository for instructions on how to build and run the simulation.
