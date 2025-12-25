import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './module-page.module.css';

function ModuleCard({ title, description, to }) {
  return (
    <div className={clsx('col col--4', styles.moduleCard)}>
      <Link to={to} className={styles.cardLink}>
        <div className="card">
          <div className="card__body">
            <Heading as="h3">{title}</Heading>
            <p>{description}</p>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function Module3Page() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Module 3 - The AI-Robot Brain (NVIDIA Isaac™)"
      description="Learn about Isaac Sim, perception, navigation, and Nav2 for humanoid robots">
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <Heading as="h1" className={clsx(styles.moduleTitle)}>
                Module 3 - The AI-Robot Brain (NVIDIA Isaac™)
              </Heading>
              <p className={styles.moduleDescription}>
                This module covers Isaac Sim for photorealistic synthetic data, Isaac ROS for VSLAM, perception, navigation, and Nav2 for real humanoid path planning.
              </p>
            </div>
          </div>

          <div className="row">
            <ModuleCard
              title="Chapter 1: Isaac Sim Basics"
              description="Introduction to NVIDIA Isaac Sim, photorealistic synthetic data generation, and high-fidelity simulation for robotics."
              to="/docs/module-3-ai-brain/chapter-1-isaac-sim-basics"
            />
            <ModuleCard
              title="Chapter 2: Perception and Navigation"
              description="Robot perception systems using ROS 2 and Isaac tools, implementing perception for humanoid robots."
              to="/docs/module-3-ai-brain/chapter-2-perception-navigation"
            />
            <ModuleCard
              title="Chapter 3: Nav2 Path Planning"
              description="Path planning using Nav2 for humanoid robots, implementing navigation algorithms."
              to="/docs/module-3-ai-brain/chapter-3-nav2-path-planning"
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}