# Security Policy

## Supported Versions

| Version | Supported          |
|---------|--------------------|
| 6.0.x   | :white_check_mark:  |
| < 6.0   | :x:                 |

## Reporting a Vulnerability

This project interfaces with Autodesk Fusion 360 via its MCP server on `http://127.0.0.1:27182/mcp` and writes to `C:\opt_fusion_log.txt` and `C:\OptimusPrime_STL\`.

If you discover a security vulnerability:

1. **Do not** open a public GitHub Issue
2. Email the maintainers directly or open a [draft security advisory](https://github.com/itsPremkumar/Optimus_Prime/security/advisories)
3. Include a detailed description, reproduction steps, and potential impact

We will acknowledge receipt within 48 hours and provide a timeline for a fix.

## Security Considerations

- The MCP connection is localhost-only — do not expose it to untrusted networks
- Script payloads are executed with the full privileges of Fusion 360 — only run trusted scripts
- The stop flag file (`C:\opt_fusion_stop.flag`) should not be written by untrusted processes
