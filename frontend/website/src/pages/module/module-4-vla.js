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

export default function Module4Page() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Module 4 - Vision-Language-Action (VLA)"
      description="Learn about Whisper, LLM-based cognitive planning, and natural language to ROS 2 actions">
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <Heading as="h1" className={clsx(styles.moduleTitle)}>
                Module 4 - Vision-Language-Action (VLA)
              </Heading>
              <p className={styles.moduleDescription}>
                This module covers Whisper for voice-to-command, LLM-based cognitive planning, and natural language to ROS 2 actions conversion.
              </p>
            </div>
          </div>

          <div className="row">
            <ModuleCard
              title="Chapter 1: Whisper for Voice Commands"
              description="Whisper ASR system for converting human speech into text, forming the first part of the voice command processing pipeline."
              to="/docs/module-4-vla/chapter-1-whisper-voice-commands"
            />
            <ModuleCard
              title="Chapter 2: LLM-Based Cognitive Planning"
              description="Using Large Language Models for high-level cognitive planning in humanoid robots."
              to="/docs/module-4-vla/chapter-2-llm-cognitive-planning"
            />
            <ModuleCard
              title="Chapter 3: Natural Language to Actions"
              description="Converting natural language commands to executable ROS 2 actions for humanoid robots."
              to="/docs/module-4-vla/chapter-3-natural-language-actions"
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}