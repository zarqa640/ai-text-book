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

export default function CapstonePage() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Capstone Project"
      description="Bringing it all together - autonomous humanoid robot project">
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <Heading as="h1" className={clsx(styles.moduleTitle)}>
                Capstone Project
              </Heading>
              <p className={styles.moduleDescription}>
                The capstone project integrates all four modules to create an autonomous humanoid robot that can understand natural language commands and execute them in the real world.
              </p>
            </div>
          </div>

          <div className="row">
            <ModuleCard
              title="Introduction"
              description="Overview of the capstone project requirements and objectives for the autonomous humanoid robot."
              to="/docs/capstone/introduction"
            />
            <ModuleCard
              title="Integration"
              description="How to integrate all modules together for the capstone project implementation."
              to="/docs/capstone/integration"
            />
            <ModuleCard
              title="Deployment"
              description="Deploying your autonomous humanoid robot system in real-world scenarios."
              to="/docs/capstone/deployment"
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}