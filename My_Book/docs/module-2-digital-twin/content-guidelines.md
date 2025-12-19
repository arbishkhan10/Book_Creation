# Content Creation Guidelines for Simulation Technologies (Gazebo & Unity)

## Constitution Compliance

All content in this module must adhere to the project constitution principles:

### 1. Source-Grounded Content
- All content must be traceable to official Gazebo, Unity, or sensor simulation documentation
- No speculative or hypothetical content allowed
- Citations required for all technical claims
- Updates must reference current official documentation

### 2. Zero Hallucination Tolerance
- Strict grounding required: no extrapolation, inference, or generation beyond indexed data
- Clear refusal statements for out-of-scope queries
- Explicit citation of source material when answering

### 3. Production-Ready Standards
- All content must meet production quality standards
- Comprehensive error handling in examples (where applicable)
- Performance considerations addressed
- Security best practices implemented by default
- Documentation included with all features

## Content Structure

### Frontmatter Requirements
Each documentation file must include the following frontmatter:

```yaml
---
title: [Clear, descriptive title]
description: [Brief description of the content]
tags: [relevant tags like gazebo, unity, sensors, etc.]
authors: [author information]
---
```

### Content Format
- Use clear headings and subheadings to organize content
- Include practical examples with code/configurations
- Provide explanations suitable for students with varying technical backgrounds
- Include diagrams or illustrations where helpful (with proper attribution)

## Simulation-Specific Guidelines

### Gazebo Content
- Include version information for Gazebo (e.g., Garden, Fortress)
- Provide URDF model examples and explanations
- Cover physics properties, collision detection, and joint constraints
- Explain environment setup and lighting considerations

### Unity Content
- Include version information for Unity
- Provide step-by-step scene setup instructions
- Cover 3D asset integration and optimization
- Explain Human-Robot Interaction (HRI) interface design

### Sensor Simulation Content
- Specify sensor types (LiDAR, depth cameras, IMU)
- Include configuration parameters and expected outputs
- Cover validation methods for sensor data
- Explain integration with perception systems

## Citation Format

When referencing official documentation:

### Gazebo Documentation
```markdown
## References

[^1]: Gazebo Documentation. (2024). *Physics Simulation*. Retrieved from http://gazebosim.org/
[^2]: Gazebo Documentation. (2024). *Model Format*. Retrieved from http://gazebosim.org/
```

### Unity Documentation
```markdown
## References

[^1]: Unity Documentation. (2024). *Scripting API*. Retrieved from https://docs.unity3d.com/
[^2]: Unity Documentation. (2024). *3D Objects*. Retrieved from https://docs.unity3d.com/
```

### Sensor Documentation
```markdown
## References

[^1]: Official Sensor Documentation. (2024). *LiDAR Simulation*. Retrieved from [official source]
[^2]: Official Sensor Documentation. (2024). *Camera Simulation*. Retrieved from [official source]
```

## Quality Assurance

Before publishing any content:
- Verify all information against official Gazebo, Unity, and sensor documentation
- Test all examples to ensure they work as described
- Ensure accessibility for students with different technical backgrounds
- Confirm all content adheres to the "Source-Grounded Content" principle
- Validate that all examples follow simulation best practices and conventions