# Project Guidelines

This document outlines the guidelines for contributing to the ETHRC Drone Racing project, including documentation standards and development workflow.

## Documentation Guidelines

We use [MkDocs](https://www.mkdocs.org/) with the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme.

### Editing Documentation

Documentation source files are located in `docs/docs/`. They are written in Markdown.

- **Headers**: Use `#` for page titles and `##` for section headers.
- **Code Blocks**: Use triple backticks (\`\`\`) for code blocks. Specify the language for syntax highlighting (e.g., \`\`\`bash).
- **Images**: Place images in `docs/docs/images/` and reference them as `![Alt Text](images/filename.png)`.
- **Admonitions**: Use admonitions for notes, warnings, and tips:
    ```markdown
    !!! note
        This is a note.
    ```

### Running Locally

To preview your changes locally, run the following command from the `docs/` directory (or root if using the Docker container):

```bash
mkdocs serve
```

The site will be available at `http://localhost:8000`.

## Project Workflow

### Git Workflow

We follow a standard feature-branch workflow:

1.  **Create a Branch**: Create a new branch for your feature or fix.
    ```bash
    git checkout -b feature/my-new-feature
    ```
2.  **Commit Changes**: Make your changes and commit them with clear, descriptive messages.
    ```bash
    git commit -m "Add new flight controller logic"
    ```
3.  **Push and PR**: Push your branch to GitHub and open a Pull Request (PR) against `main`.
4.  **Review and Merge**: After review, merge the PR. Delete the branch after merging.

### Docker Environment

The project relies heavily on Docker to ensure a consistent environment.

- **Rebuilding**: If you modify `docker/Dockerfile` or `requirements.txt` (if applicable), rebuild the image:
    ```bash
    docker compose build
    ```
- **Running**: Start the environment with:
    ```bash
    docker compose up -d
    ```
