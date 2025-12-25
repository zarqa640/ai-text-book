import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './chatbot.module.css';

function ChatBotPage() {
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! How can I help you today?', sender: 'bot', timestamp: new Date() }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsTyping(true);

    // Simulate bot response after delay
    setTimeout(() => {
      const botResponse = {
        id: Date.now() + 1,
        text: getBotResponse(inputValue),
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, botResponse]);
      setIsTyping(false);
    }, 1000);
  };

  const getBotResponse = (userInput) => {
    const lowerInput = userInput.toLowerCase();

    if (lowerInput.includes('hello') || lowerInput.includes('hi') || lowerInput.includes('hey')) {
      return 'Hello there! How can I assist you with Physical AI & Humanoid Robotics today?';
    } else if (lowerInput.includes('module') || lowerInput.includes('course')) {
      return 'We have 4 modules: Module 1 (ROS 2), Module 2 (Digital Twin), Module 3 (AI-Robot Brain), and Module 4 (Vision-Language-Action). Which one would you like to know more about?';
    } else if (lowerInput.includes('ros') || lowerInput.includes('robot')) {
      return 'ROS 2 is the robotic nervous system! It handles communication between different parts of the robot. Would you like to learn more about rclpy or URDF?';
    } else if (lowerInput.includes('gazebo') || lowerInput.includes('unity') || lowerInput.includes('simulation')) {
      return 'The Digital Twin module covers Gazebo physics simulation and Unity integration for high-fidelity visualization. Very important for humanoid robotics!';
    } else if (lowerInput.includes('ai') || lowerInput.includes('isaac') || lowerInput.includes('brain')) {
      return 'The AI-Robot Brain module uses NVIDIA Isaac Sim for perception and navigation. Nav2 provides advanced path planning capabilities.';
    } else if (lowerInput.includes('vision') || lowerInput.includes('language') || lowerInput.includes('action') || lowerInput.includes('whisper') || lowerInput.includes('llm')) {
      return 'The VLA module combines Whisper for voice commands, LLMs for cognitive planning, and natural language to ROS 2 actions. Powerful for human-robot interaction!';
    } else if (lowerInput.includes('thank')) {
      return 'You\'re welcome! Is there anything else I can help you with?';
    } else if (lowerInput.includes('help')) {
      return 'I can help you with information about the Physical AI & Humanoid Robotics course modules. Try asking about specific modules, ROS 2, Gazebo, Unity, Isaac Sim, or Vision-Language-Action systems.';
    } else {
      return 'That\'s interesting! For more detailed information, I recommend checking out the specific modules in our course. Would you like to know about any particular topic?';
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <Layout title="Physical AI Chatbot" description="WhatsApp-style chat interface for Physical AI & Humanoid Robotics">
      <div className={styles.chatContainer}>
        {/* WhatsApp-style chat header */}
        <div className={styles.chatHeader}>
          <div className={styles.headerLeft}>
            <div className={styles.avatar}>
              <div className={styles.botIcon}>🤖</div>
            </div>
            <div className={styles.headerInfo}>
              <h3>Physical AI Assistant</h3>
              <p className={styles.status}>Online</p>
            </div>
          </div>
          <div className={styles.headerRight}>
            <button className={clsx(styles.headerButton, styles.infoButton)}>ℹ️</button>
            <button className={clsx(styles.headerButton, styles.menuButton)}>⋮</button>
          </div>
        </div>

        {/* Chat messages area */}
        <div className={styles.messagesArea}>
          {messages.map((message) => (
            <div
              key={message.id}
              className={clsx(
                styles.message,
                message.sender === 'user' ? styles.userMessage : styles.botMessage
              )}
            >
              <div className={styles.messageContent}>
                <p>{message.text}</p>
                <span className={styles.timestamp}>{formatTime(message.timestamp)}</span>
              </div>
            </div>
          ))}

          {isTyping && (
            <div className={clsx(styles.message, styles.botMessage)}>
              <div className={styles.messageContent}>
                <div className={styles.typingIndicator}>
                  <div className={styles.typingDot}></div>
                  <div className={styles.typingDot}></div>
                  <div className={styles.typingDot}></div>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        {/* Message input area */}
        <form onSubmit={handleSendMessage} className={styles.inputArea}>
          <div className={styles.inputContainer}>
            <button type="button" className={styles.emojiButton}>😊</button>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Type a message..."
              className={styles.messageInput}
              ref={inputRef}
            />
            <button type="submit" className={styles.sendButton}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </button>
          </div>
        </form>
      </div>
    </Layout>
  );
}

export default ChatBotPage;