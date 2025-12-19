# Citation Format for Official ROS 2 Documentation

## Purpose
This document establishes the standard format for citing official ROS 2 documentation and other authoritative sources to ensure compliance with the "source-grounded content" principle.

## Standard Citation Format

### In-text Citations
For inline references to official documentation:

```markdown
According to the official ROS 2 documentation [^1], the concept works as follows...
```

### Reference List Format
At the end of each document, include a "References" section:

```markdown
## References

[^1]: ROS 2 Documentation. (2024). *ROS 2 Basics*. Retrieved from https://docs.ros.org/en/rolling/
[^2]: ROS 2 Documentation. (2024). *Nodes and Topics*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Topics/
[^3]: ROS 2 Working Group. (2024). *Design Documents*. Retrieved from https://design.ros2.org/
```

## Citation Elements

Each citation should include:

1. **Author/Organization**: ROS 2 Documentation, ROS 2 Working Group, etc.
2. **Year**: Publication or last updated year (use current year if not specified)
3. **Title**: Specific document or section title
4. **Source**: URL to the official documentation
5. **Access Date**: Optional but recommended for frequently updated documentation

## Alternative Citation Formats

### Direct Link with Context
```markdown
The publisher-subscriber pattern is fundamental to ROS 2 (see [official ROS 2 topics documentation](https://docs.ros.org/en/rolling/Concepts/About-Topics/)).
```

### Block Quote Citations
```markdown
> "ROS 2 introduces a data-centric approach to robotics middleware using DDS." [^1]

[^1]: ROS 2 Documentation. (2024). *Middleware Implementation*. Retrieved from https://docs.ros.org/en/rolling/Concepts/Middleware/Overview/
```

## Best Practices

1. **Always link to the specific section** rather than the general documentation page
2. **Use version-specific links** when available (e.g., rolling, humble, etc.)
3. **Verify link accuracy** before publishing
4. **Include the documentation version** in citations when relevant
5. **Distinguish between different ROS 2 distributions** (Rolling, Humble, etc.)

## Examples

### Basic Citation
```markdown
ROS 2 uses a client library abstraction to provide APIs in multiple languages [^1].

[^1]: ROS 2 Documentation. (2024). *Client Libraries*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Client-Libraries/
```

### Multiple Citations
```markdown
Communication in ROS 2 is handled through topics [^1] and services [^2], with additional support for actions [^3].

[^1]: ROS 2 Documentation. (2024). *Topics*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Topics/
[^2]: ROS 2 Documentation. (2024). *Services*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Services/
[^3]: ROS 2 Documentation. (2024). *Actions*. Retrieved from https://docs.ros.org/en/rolling/Concepts/About-Actions/
```

## Verification Process

Before finalizing any content:
- [ ] Verify all links are valid and accessible
- [ ] Confirm citations point to official ROS 2 documentation
- [ ] Ensure all technical claims are supported by cited sources
- [ ] Check that no speculative information is presented without proper attribution