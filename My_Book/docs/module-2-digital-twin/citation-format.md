# Citation Format for Simulation Technologies Documentation

## Purpose
This document establishes the standard format for citing official Gazebo, Unity, and sensor simulation documentation to ensure compliance with the "source-grounded content" principle.

## Standard Citation Format

### In-text Citations
For inline references to official documentation:

```markdown
According to the official Gazebo documentation [^1], the physics engine...
```

### Reference List Format
At the end of each document, include a "References" section:

```markdown
## References

[^1]: Gazebo Documentation. (2024). *Physics Simulation*. Retrieved from http://gazebosim.org/
[^2]: Unity Documentation. (2024). *Scripting API*. Retrieved from https://docs.unity3d.com/
[^3]: Official Sensor Documentation. (2024). *LiDAR Simulation*. Retrieved from [official source]
```

## Citation Elements by Technology

### Gazebo Citations
Each Gazebo citation should include:
1. **Author/Organization**: Gazebo Documentation, Open Robotics, etc.
2. **Year**: Publication or last updated year
3. **Title**: Specific document or section title
4. **Source**: URL to the official Gazebo documentation
5. **Version**: Gazebo version if relevant (e.g., Garden, Fortress)

### Unity Citations
Each Unity citation should include:
1. **Author/Organization**: Unity Documentation, Unity Technologies, etc.
2. **Year**: Publication or last updated year
3. **Title**: Specific document or section title
4. **Source**: URL to the official Unity documentation
5. **Version**: Unity version if relevant

### Sensor Citations
Each sensor citation should include:
1. **Author/Organization**: Manufacturer documentation, ROS documentation, etc.
2. **Year**: Publication or last updated year
3. **Title**: Specific sensor model or concept
4. **Source**: URL to the official documentation
5. **Model/Type**: Specific sensor model or type if relevant

## Alternative Citation Formats

### Direct Link with Context
```markdown
The physics properties in Gazebo are configured as described in the [official Gazebo documentation](http://gazebosim.org/).
```

### Block Quote Citations
```markdown
> "Gazebo provides a high-fidelity physics simulation environment." [^1]

[^1]: Gazebo Documentation. (2024). *Physics Simulation*. Retrieved from http://gazebosim.org/
```

## Best Practices

1. **Always link to the specific section** rather than the general documentation page
2. **Use version-specific links** when available
3. **Verify link accuracy** before publishing
4. **Include the software version** in citations when relevant
5. **Distinguish between different software versions** (Unity 2022 vs 2023, etc.)

## Examples

### Gazebo Citation
```markdown
Gazebo uses ODE as its default physics engine [^1].

[^1]: Gazebo Documentation. (2024). *Physics Engine*. Retrieved from http://gazebosim.org/docs/garden/physics/
```

### Unity Citation
```markdown
Unity's rendering pipeline can be customized for high-fidelity visualization [^1].

[^1]: Unity Documentation. (2024). *Scriptable Render Pipeline*. Retrieved from https://docs.unity3d.com/Manual/render-pipelines.html
```

### Sensor Citation
```markdown
LiDAR sensors produce point cloud data that can be processed in real-time [^1].

[^1]: ROS Documentation. (2024). *LiDAR Processing*. Retrieved from https://wiki.ros.org/laser_pipeline
```

## Verification Process

Before finalizing any content:
- [ ] Verify all links are valid and accessible
- [ ] Confirm citations point to official documentation
- [ ] Ensure all technical claims are supported by cited sources
- [ ] Check that no speculative information is presented without proper attribution
- [ ] Validate that all simulation examples are grounded in official documentation