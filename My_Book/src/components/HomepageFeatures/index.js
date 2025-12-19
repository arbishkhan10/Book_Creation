import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Fundamentals',
    icon: '⚡', // Lightning bolt icon
    description: 'Master the fundamentals of ROS 2 for robotics development.',
  },
  {
    title: 'Simulation & Testing',
    icon: '🧪', // Test tube icon
    description: 'Comprehensive simulation and testing environments.',
  },
  {
    title: 'NVIDIA Isaac Platform',
    icon: '🤖', // Robot icon
    description: 'Leverage NVIDIA Isaac for advanced robotics applications.',
  },
  {
    title: 'Vision-Language-Action',
    icon: '👁️', // Eye icon
    description: 'Integrate cutting-edge VLA models to enable robots to understand and act on natural language commands.',
  },
];

function Feature({title, icon, description}) {
  return (
    <div className={clsx('col col--3')}>
      <div className={`${styles.featureCard}`}>
        <div className={styles.featureCardContent}>
          <div className={styles.featureIcon}>{icon}</div>
          <Heading as="h3">{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
