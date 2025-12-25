// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Manual sidebar for the Physical AI & Humanoid Robotics book
  bookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/chapter-1-ros2-basics',
        'module-1-ros2/chapter-2-python-agents-rclpy',
        'module-1-ros2/chapter-3-urdf-humanoids',
        'module-1-ros2/chapter-4-advanced-ros2-concepts',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/chapter-1-gazebo-basics',
        'module-2-digital-twin/chapter-2-unity-integration',
        'module-2-digital-twin/chapter-3-sensor-simulation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-ai-brain/chapter-1-isaac-sim-basics',
        'module-3-ai-brain/chapter-2-perception-navigation',
        'module-3-ai-brain/chapter-3-nav2-path-planning',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/chapter-1-whisper-voice-commands',
        'module-4-vla/chapter-2-llm-cognitive-planning',
        'module-4-vla/chapter-3-natural-language-actions',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/introduction',
        'capstone/integration',
        'capstone/deployment',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
