// @ts-check
// `@type` JSDoc annotations allow IDEs and type-checking tools to autocomplete
// and validate type information. For more information, visit:
// https://docusaurus.io/docs/jsdoc
// Note: These types are for Docusaurus v3.
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI/Spec-Driven Technical Book',
  tagline: 'ROS 2 Educational Module',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-book.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use '/' for root deployment
  baseUrl: '/',

  // Vercel deployment configuration
  organizationName: 'arbishkhan10', // Your GitHub org/user name.
  projectName: 'My-Book', // Your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/arbishkhan10/My-Book/edit/main/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/arbishkhan10/My-Book/edit/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Humanoid Robotics Book',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/robotics-logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/facebook/docusaurus',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Module 1: ROS 2 Basics',
                to: '/docs/module-1-ros-nervous-system',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/module-2-digital-twin',
              },
              {
                label: 'Module 3: NVIDIA Isaac',
                to: '/docs/module-3-ai-robot-brain/nvidia-isaac-sim',
              },
              {
                label: 'Module 4: VLA Integration',
                to: '/docs/module-4-vla-integration',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/rolling/',
              },
              {
                label: 'NVIDIA Isaac',
                href: 'https://developer.nvidia.com/isaac',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/arbishkhan10/My-Book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Arbish Khan, MetaLog Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;