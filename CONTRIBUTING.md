# Contributing to Optimus Prime G1 Simulation

Thank you for considering contributing to this project! We welcome contributions of all kinds — bug reports, feature requests, documentation improvements, and code changes.

## How to Contribute

### 1. Reporting Bugs
- Open a [GitHub Issue](https://github.com/itsPremkumar/Optimus_Prime/issues/new?template=bug_report.md)
- Include:
  - Python version and Fusion 360 version
  - Full error output from `output/logs/optimus_fusion_log_*.txt`
  - Steps to reproduce

### 2. Suggesting Enhancements
- Open a [Feature Request](https://github.com/itsPremkumar/Optimus_Prime/issues/new?template=feature_request.md)
- Describe the problem you're solving and your proposed solution

### 3. Pull Requests

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes
4. Test your changes by running `python run_simulation.py --module <module_name>`
5. Commit with a clear message: `git commit -m "feat: add X capability"`
6. Push: `git push origin feature/my-feature`
7. Open a Pull Request

### Development Setup
```bash
git clone https://github.com/itsPremkumar/Optimus_Prime.git
cd Optimus_Prime
# No dependencies required — standard library only
# Ensure Autodesk Fusion 360 with MCP server is running
```

### Code Style
- Follow PEP 8 conventions
- Use descriptive variable names matching the existing style (e.g., `snake_case`)
- Keep functions focused and single-purpose
- Add docstrings for public functions
- Avoid external dependencies — the project uses only the Python standard library

### Commit Convention
We use [Conventional Commits](https://www.conventionalcommits.org/):
- `feat:` — new feature
- `fix:` — bug fix
- `docs:` — documentation
- `refactor:` — code restructuring
- `perf:` — performance improvement

## Code of Conduct
Please read and follow our [Code of Conduct](CODE_OF_CONDUCT.md).
