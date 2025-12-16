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

export default function Module1Page() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Module 1 - The Robotic Nervous System (ROS 2)"
      description="Learn about ROS 2 basics, Python agents with rclpy, and URDF for humanoids">
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <Heading as="h1" className={clsx(styles.moduleTitle)}>
                Module 1 - The Robotic Nervous System (ROS 2)
              </Heading>
              <p className={styles.moduleDescription}>
                This module teaches how humanoid robots communicate, move, and sense using ROS 2, rclpy, and URDF.
              </p>
            </div>
          </div>

          <div className="row">
            <ModuleCard
              title="Chapter 1: ROS 2 Basics"
              description="What ROS 2 is, why humanoid robots need it, nodes, topics, services with simple analogies, and basic CLI commands."
              to="/docs/module-1-ros2/chapter-1-ros2-basics"
              moduleType="Module1"
            />
            <ModuleCard
              title="Chapter 2: Python Agents with rclpy"
              description="AI agent → ROS 2 control pipeline, publisher, subscriber, simple controller, example: move humanoid joint using Python."
              to="/docs/module-1-ros2/chapter-2-python-agents-rclpy"
              moduleType="Module1"
            />
            <ModuleCard
              title="Chapter 3: URDF for Humanoids"
              description="What URDF is and how it defines robot links/joints, mini humanoid URDF example, visualizing in RViz/Gazebo."
              to="/docs/module-1-ros2/chapter-3-urdf-humanoids"
              moduleType="Module1"
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}