# Physical AI & Humanoid Robotics - Module Structure

## Chapter Organization

This document explains how to organize chapter markdown files for the book.

### Directory Structure

```
docs/
├── intro.md                    # Introduction page
├── module-1-ros2/              # Module 1 chapters
│   ├── chapter-1-ros2-basics.md
│   ├── chapter-2-python-agents-rclpy.md
│   └── chapter-3-urdf-humanoids.md
├── module-2-digital-twin/      # Module 2 chapters
│   ├── chapter-1-gazebo-basics.md
│   ├── chapter-2-unity-integration.md
│   └── chapter-3-sensor-simulation.md
├── module-3-ai-brain/          # Module 3 chapters
│   ├── chapter-1-isaac-sim-basics.md
│   ├── chapter-2-perception-navigation.md
│   └── chapter-3-nav2-path-planning.md
├── module-4-vla/               # Module 4 chapters
│   ├── chapter-1-whisper-voice-commands.md
│   ├── chapter-2-llm-cognitive-planning.md
│   └── chapter-3-natural-language-actions.md
└── capstone/                   # Capstone project chapters
    ├── introduction.md
    ├── integration.md
    └── deployment.md
```

### Adding New Chapters

To add a new chapter:

1. Create the markdown file in the appropriate module directory
2. Add the file path to the corresponding module page in `src/pages/module/`
3. The chapter will automatically appear as a card on the module page

### Chapter File Format

Each chapter file should start with frontmatter:

```markdown
---
sidebar_position: 1  # Position in the module
---

# Chapter Title

Chapter content here...
```

### Module Pages

Each module page is located at:
- `src/pages/module/module-1-ros2.js`
- `src/pages/module/module-2-digital-twin.js`
- `src/pages/module/module-3-ai-brain.js`
- `src/pages/module/module-4-vla.js`
- `src/pages/module/capstone.js`

To add a new chapter to a module page, add a new `ModuleCard` component with:
- `title`: The chapter title
- `description`: Brief description of the chapter
- `to`: Path to the chapter file (e.g., `/docs/module-1-ros2/chapter-name`)

### Sidebar Configuration

The sidebar is configured in `sidebars.js` and only shows module names as links to their respective pages.