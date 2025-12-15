import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './module-page.module.css';

function ModuleCard({ title, description, to, moduleType }) {
  return (
    <div className={clsx('col col--4', styles.moduleCard, styles[`moduleCard${moduleType}`])}>
      <Link to={to} className={styles.cardLink}>
        <div className={clsx("card", styles[`card${moduleType}`])}>
          <div className="card__body">
            <Heading as="h3" className={styles[`cardTitle${moduleType}`]}>{title}</Heading>
            <p>{description}</p>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function Module2Page() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Module 2 - The Digital Twin (Gazebo & Unity)"
      description="Learn about Gazebo physics simulation, Unity integration, and sensor simulation for humanoid robots">
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <Heading as="h1" className={clsx(styles.moduleTitle)}>
                Module 2 - The Digital Twin (Gazebo & Unity)
              </Heading>
              <p className={styles.moduleDescription}>
                This module covers physics simulation, gravity and collisions, building environments in Gazebo, high-fidelity visualization in Unity, and sensor simulation.
              </p>
            </div>
          </div>

          <div className="row">
            <ModuleCard
              title="Chapter 1: Gazebo Basics"
              description="Introduction to Gazebo physics simulation environment, setting up environments, gravity and collisions for humanoid robots."
              to="/docs/module-2-digital-twin/chapter-1-gazebo-basics"
              moduleType="Module2"
            />
            <ModuleCard
              title="Chapter 2: Unity Integration"
              description="Unity for high-fidelity visualization and simulation, integrating Unity with robotics applications."
              to="/docs/module-2-digital-twin/chapter-2-unity-integration"
              moduleType="Module2"
            />
            <ModuleCard
              title="Chapter 3: Sensor Simulation"
              description="Simulating robot sensors including LiDAR, depth cameras, and IMU for humanoid robots."
              to="/docs/module-2-digital-twin/chapter-3-sensor-simulation"
              moduleType="Module2"
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}