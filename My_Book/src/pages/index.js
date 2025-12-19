import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={clsx('hero__title', styles.heroTitle)}>
            Ready to Build Humanoid Robots?
          </Heading>
          <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
            Start your journey into the world of humanoid robotics with practical, hands-on guidance.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.heroButton)}
              to="/docs/intro">
              Get Started Now
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
