---
id: 1
title: create-project-constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,spec-first,ai-book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constituion /sp.constituion

Project: AI/Spec-Driven Technical Book with Embedded RAG Chatbot

Purpose:
Create and deploy a Docusaurus-based technical book using Spec-Kit Plus and Claude Code, published on GitHub Pages, with an embedded RAG chatbot that answers questions strictly from the book content and user-selected text.

Core Principles:
- Spec-first, source-grounded development
- Accuracy via official documentation
- Clarity for developers and AI engineers
- Reproducibility and modular design
- Zero hallucination tolerance

Tech Stack:
- Authoring: Claude Code, Spec-Kit Plus
- Docs: Docusaurus (MD/MDX)
- Deploy: GitHub Pages
- Backend: FastAPI
- AI: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres

Standards:
- All content traceable to specs and sources
- Production-ready code only
- Chapters map 1:1 to spec sections
- RAG answers grounded only in indexed content
- Support full-book and selected-text retrieval
- Refuse unsupported queries with explanation
```

## Response snapshot

Created the project constitution file (.specify/memory/constitution.md) with the following core principles for the AI/Spec-Driven Technical Book project:

1. Spec-First Development: All development begins with comprehensive specifications using Spec-Kit Plus
2. Source-Grounded Content: All book content must be traceable to official documentation
3. Zero Hallucination Tolerance: RAG chatbot must only respond with information directly sourced from indexed book content
4. Modular, Reusable Architecture: Book components designed for modularity and reuse
5. Production-Ready Code Standards: All code must meet production deployment requirements
6. Traceability and Grounding: Every chapter maps 1:1 to specification sections

The constitution also includes sections on Technology Stack Requirements, Development Workflow, and Governance with version 1.0.0 ratified on 2025-12-17.

## Outcome

- ✅ Impact: Created initial project constitution with 6 core principles aligned to project goals
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Proceed with creating spec document based on constitution
- 🧠 Reflection: Constitution properly captures the core values of spec-first development, source-grounded content, and zero hallucination tolerance for the AI book project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - constitution aligns with project requirements
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create project specification document