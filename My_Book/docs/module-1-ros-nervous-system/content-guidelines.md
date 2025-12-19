# Content Creation Guidelines for ROS 2 Educational Module

## Constitution Compliance

All content in this module must adhere to the project constitution principles:

### 1. Source-Grounded Content
- All content must be traceable to official ROS 2 documentation, authoritative sources, or verified technical references
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
tags: [relevant tags like ros2, basics, dds, etc.]
authors: [author information]
---
```

### Content Format
- Use clear headings and subheadings to organize content
- Include practical examples with code snippets
- Provide explanations suitable for students with varying technical backgrounds
- Include diagrams or illustrations where helpful (with proper attribution)

## Citation Format

When referencing official ROS 2 documentation or other authoritative sources:

- Use proper Markdown links to official documentation
- Include version information where relevant
- Attribute all external content appropriately
- Clearly distinguish between official documentation and supplementary material

## Quality Assurance

Before publishing any content:
- Verify all information against official ROS 2 documentation
- Test all code examples to ensure they work as described
- Ensure accessibility for students with different technical backgrounds
- Confirm all content adheres to the "Source-Grounded Content" principle
- Validate that all examples follow ROS 2 best practices and conventions