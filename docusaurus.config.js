//
import { themes as prismThemes } from 'prism-react-renderer';

/** {import('@docusaurus/types').Config} */
const config = {
  title: 'AI/Spec-Driven Technical Book',
  tagline: 'ROS 2 Educational Module',
  favicon: 'img/favicon.ico',

  // GitHub Pages URL
  url: 'https://arbishkhan10.github.io',
  baseUrl: '/', // GitHub repo name

  organizationName: 'arbishkhan10',
  projectName: 'My-Book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/arbishkhan10/My-Book/edit/main/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/arbishkhan10/My-Book/edit/main/blog/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Humanoid Robotics Book',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo.svg', // Add dark mode logo if available
      },
      items: [
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Module' },
        { to: '/blog', label: 'Blog', position: 'left' },
        { href: 'https://github.com/arbishkhan10/My-Book', label: 'GitHub', position: 'right' },
      ],
      style: 'primary', // Use primary color for navbar
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'json', 'python', 'typescript', 'docker', 'yaml'], // Add more languages
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 5,
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [{ label: 'Modules', to: '/docs/intro' }],
        },
        {
          title: 'Community',
          items: [
            { label: 'Stack Overflow', href: 'https://stackoverflow.com/questions/tagged/docusaurus' },
            { label: 'Discord', href: 'https://discordapp.com/invite/docusaurus' },
            { label: 'Twitter', href: 'https://twitter.com/docusaurus' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'Blog', to: '/blog' },
            { label: 'GitHub', href: 'https://github.com/arbishkhan10/My-Book' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Arbish Khan. Built with Docusaurus.`,
    },
  },
};

export default config;
