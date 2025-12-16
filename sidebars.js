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
      type: 'link',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      href: '/module/module-1-ros2',
    },
    {
      type: 'link',
      label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
      href: '/module/module-2-digital-twin',
    },
    {
      type: 'link',
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac™)',
      href: '/module/module-3-ai-brain',
    },
    {
      type: 'link',
      label: 'Module 4 - Vision-Language-Action (VLA)',
      href: '/module/module-4-vla',
    },
    {
      type: 'link',
      label: 'Capstone Project',
      href: '/module/capstone',
    },
  ],
};

export default sidebars;
