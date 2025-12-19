// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros-nervous-system/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-1-ros-nervous-system/ros2-basics',
            'module-1-ros-nervous-system/nodes-topics-services',
            'module-1-ros-nervous-system/urdf-python-ros-integration'
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-2-digital-twin/physics-simulation-gazebo',
            'module-2-digital-twin/digital-twins-hri-unity',
            'module-2-digital-twin/sensor-simulation-validation'
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-ai-robot-brain/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-3-ai-robot-brain/nvidia-isaac-sim',
            'module-3-ai-robot-brain/isaac-ros-vslam-navigation',
            'module-3-ai-robot-brain/nav2-path-planning-humanoid'
          ],
        }
      ],
    },
  ],
};

export default sidebars;