import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const ModuleList = [
  {
    title: 'Module 1: The Robotic Nervous System',
    icon: '🧠', // Brain emoji representing the nervous system
    description: (
      <>
        Learn how humanoid robots communicate, move, and sense using ROS 2, rclpy, and URDF.
        Understand nodes, topics, services with simple analogies and basic CLI commands.
      </>
    ),
    to: '/docs/module-1-ros2/chapter-1-ros2-basics',
  },
  {
    title: 'Module 2: The Digital Twin',
    icon: '🔄', // Loop arrows representing simulation and twin
    description: (
      <>
        Master physics simulation, environment building, and high-fidelity visualization
        using Gazebo and Unity for your humanoid robots.
      </>
    ),
    to: '/docs/module-2-digital-twin/intro',
  },
  {
    title: 'Module 3: The AI-Robot Brain',
    icon: '🤖', // Robot emoji representing the AI brain
    description: (
      <>
        Explore Isaac Sim for photorealistic synthetic data, Isaac ROS for perception,
        navigation, and Nav2 for real humanoid path planning.
      </>
    ),
    to: '/docs/module-3-ai-robot-brain/intro',
  },
  {
    title: 'Module 4: Vision-Language-Action',
    icon: '👁️', // Eye emoji representing vision and perception
    description: (
      <>
        Implement Whisper for voice-to-command, LLM-based cognitive planning,
        and natural language to ROS 2 actions conversion.
      </>
    ),
    to: '/docs/module-4-vla/intro',
  },
];

function ModuleCard({ title, icon, description, to }) {
  return (
    <div className={clsx('col col--3')}>
      <Link
        to={to}
        className={clsx('card', styles.moduleCard)}
      >
        <div className="card__header text--center">
          <span className={styles.moduleIcon}>{icon}</span>
          <Heading as="h3">{title}</Heading>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer text--center">
          <span className={styles.learnMore}>Click to Open →</span>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className="text--center padding-bottom--lg">
              <Heading as="h2" className={styles.sectionTitle}>
                Course Modules
              </Heading>
              <p className={styles.sectionSubtitle}>
                Explore our comprehensive curriculum designed for humanoid robotics
              </p>
            </div>
          </div>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
