---
sidebar_position: 3
---

# Chapter 3: Natural Language to Actions

## Converting Natural Language to ROS 2 Actions

This chapter focuses on the critical integration between natural language processing and ROS 2 action execution. We'll explore how to transform voice commands and text inputs into executable ROS 2 actions for humanoid robots.

## Architecture Overview

The natural language to action pipeline consists of three main components:
1. **Input Processing**: Voice-to-text or direct text input
2. **Intent Recognition**: Understanding the user's intent
3. **Action Mapping**: Converting intent to ROS 2 actions

## Natural Language Action Mapping Node

Here's a comprehensive ROS 2 node that maps natural language to ROS 2 actions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
import re
from typing import Dict, List, Tuple

class NaturalLanguageActionMapper(Node):
    def __init__(self):
        super().__init__('natural_language_action_mapper')

        # Subscriptions for voice commands and text commands
        self.voice_subscription = self.create_subscription(
            String,
            'voice_commands',
            self.voice_callback,
            10
        )

        self.text_subscription = self.create_subscription(
            String,
            'text_commands',
            self.text_callback,
            10
        )

        # Publishers for different action types
        self.navigation_publisher = self.create_publisher(String, 'navigation_goals', 10)
        self.manipulation_publisher = self.create_publisher(String, 'manipulation_goals', 10)
        self.speech_publisher = self.create_publisher(String, 'speech_commands', 10)

        # Define action patterns
        self.action_patterns = {
            'navigation': [
                r'move to (.+)',
                r'go to (.+)',
                r'walk to (.+)',
                r'go (.+)',
                r'move (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'get (.+)',
                r'pick (.+)',
                r'lift (.+)'
            ],
            'greeting': [
                r'hello',
                r'hi',
                r'hey',
                r'greetings'
            ],
            'speech': [
                r'say (.+)',
                r'tell (.+)'
            ]
        }

    def voice_callback(self, msg):
        """Process voice commands"""
        self.process_command(msg.data, 'voice')

    def text_callback(self, msg):
        """Process text commands"""
        self.process_command(msg.data, 'text')

    def process_command(self, command: str, source: str):
        """Process a natural language command and map to ROS 2 actions"""
        self.get_logger().info(f"Processing {source} command: {command}")

        # Normalize the command
        normalized_command = command.lower().strip()

        # Determine action type based on patterns
        action_type, extracted_data = self.classify_intent(normalized_command)

        if action_type:
            self.execute_action(action_type, extracted_data, normalized_command)
        else:
            self.get_logger().warn(f"Could not classify command: {command}")
            self.speak_response(f"Sorry, I didn't understand the command: {command}")

    def classify_intent(self, command: str) -> Tuple[str, str]:
        """Classify the intent of the command"""
        for action_type, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command)
                if match:
                    extracted_data = match.group(1) if len(match.groups()) > 0 else ''
                    return action_type, extracted_data

        return None, None

    def execute_action(self, action_type: str, extracted_data: str, original_command: str):
        """Execute the appropriate action based on type"""
        if action_type == 'navigation':
            self.handle_navigation(extracted_data)
        elif action_type == 'manipulation':
            self.handle_manipulation(extracted_data)
        elif action_type == 'greeting':
            self.handle_greeting()
        elif action_type == 'speech':
            self.handle_speech(extracted_data)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")

    def handle_navigation(self, location: str):
        """Handle navigation commands"""
        # Map natural language locations to coordinates
        location_map = {
            'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            'living room': {'x': -1.0, 'y': 1.5, 'theta': 1.57},
            'bedroom': {'x': 2.0, 'y': -1.0, 'theta': 3.14},
            'dining room': {'x': 0.5, 'y': -2.0, 'theta': -1.57},
            'office': {'x': -2.0, 'y': 0.0, 'theta': 0.0}
        }

        if location in location_map:
            nav_goal = location_map[location]
            goal_msg = String()
            goal_msg.data = f"navigate_to:{nav_goal['x']}:{nav_goal['y']}:{nav_goal['theta']}"
            self.navigation_publisher.publish(goal_msg)
            self.get_logger().info(f"Sending navigation goal to {location}")
            self.speak_response(f"Okay, I'm going to the {location}")
        else:
            self.get_logger().warn(f"Unknown location: {location}")
            self.speak_response(f"Sorry, I don't know where {location} is")

    def handle_manipulation(self, object_name: str):
        """Handle manipulation commands"""
        # For simplicity, we'll publish a manipulation goal
        manipulation_msg = String()
        manipulation_msg.data = f"manipulate:{object_name}"
        self.manipulation_publisher.publish(manipulation_msg)
        self.get_logger().info(f"Sending manipulation goal for {object_name}")
        self.speak_response(f"Okay, I'll try to get the {object_name}")

    def handle_greeting(self):
        """Handle greeting commands"""
        responses = [
            "Hello! How can I help you?",
            "Hi there! What can I do for you?",
            "Greetings! How are you today?"
        ]
        import random
        response = random.choice(responses)
        self.speak_response(response)

    def handle_speech(self, text: str):
        """Handle speech commands"""
        self.speak_response(text)

    def speak_response(self, text: str):
        """Publish speech response"""
        speech_msg = String()
        speech_msg.data = text
        self.speech_publisher.publish(speech_msg)
        self.get_logger().info(f"Speaking: {text}")
```

## Advanced Intent Recognition with LLM Integration

For more sophisticated intent recognition, integrate with LLMs:

```python
import openai

class AdvancedNaturalLanguageMapper(NaturalLanguageActionMapper):
    def __init__(self):
        super().__init__()

        # Initialize LLM client for advanced intent recognition
        self.llm_client = openai.OpenAI(api_key="your-api-key-here")

    def advanced_classify_intent(self, command: str) -> Dict:
        """Use LLM for advanced intent classification"""
        try:
            response = self.llm_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": """You are a natural language understanding system for a humanoid robot.
                        Classify the user command into one of these categories:
                        - navigation: Commands to move to locations
                        - manipulation: Commands to interact with objects
                        - speech: Commands to speak text
                        - greeting: Greeting commands
                        - system: System commands
                        - unknown: Commands that don't fit other categories

                        Return the classification in JSON format: {"action_type": "...", "parameters": {...}}
                        """
                    },
                    {
                        "role": "user",
                        "content": command
                    }
                ],
                response_format={"type": "json_object"}
            )

            import json
            result = json.loads(response.choices[0].message.content)
            return result

        except Exception as e:
            self.get_logger().error(f"LLM classification failed: {e}")
            # Fallback to regex-based classification
            action_type, extracted_data = self.classify_intent(command)
            return {"action_type": action_type, "parameters": {"data": extracted_data}}
```

## Action Execution with Feedback

Implement action execution with feedback and status reporting:

```python
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ActionMapperWithFeedback(Node):
    def __init__(self):
        super().__init__('action_mapper_with_feedback')

        # Use a reentrant callback group to handle multiple callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscription for commands
        self.command_subscription = self.create_subscription(
            String,
            'commands',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )

        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'action_status', 10)

    def command_callback(self, msg):
        """Process command with async execution"""
        import threading
        thread = threading.Thread(target=self.process_command_async, args=(msg.data,))
        thread.start()

    async def process_command_async(self, command: str):
        """Process command asynchronously with status updates"""
        self.publish_status("processing", f"Processing command: {command}")

        try:
            # Classify and execute command
            result = await self.execute_command_with_status(command)
            self.publish_status("completed", f"Command completed: {command}")
        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")
            self.publish_status("failed", f"Command failed: {command} - {str(e)}")

    def publish_status(self, status: str, message: str):
        """Publish action status"""
        status_msg = String()
        status_msg.data = f"{status}:{message}"
        self.status_publisher.publish(status_msg)
```

## Integration with Robot State

Maintain awareness of robot state for context-aware command execution:

```python
class ContextAwareActionMapper(NaturalLanguageActionMapper):
    def __init__(self):
        super().__init__()

        # Robot state tracking
        self.robot_state = {
            'location': 'unknown',
            'carrying': None,
            'battery_level': 100,
            'current_task': None
        }

        # Timer to periodically update state
        self.state_timer = self.create_timer(1.0, self.update_robot_state)

    def update_robot_state(self):
        """Update robot state from various sources"""
        # In a real implementation, this would subscribe to robot state topics
        # For now, we'll just log the current state
        self.get_logger().debug(f"Robot state: {self.robot_state}")

    def process_command(self, command: str, source: str):
        """Process command with state awareness"""
        # Check if robot can execute the command given current state
        if not self.can_execute_command(command):
            self.speak_response("I can't execute that command right now")
            return

        super().process_command(command, source)

    def can_execute_command(self, command: str) -> bool:
        """Check if the robot can execute the command given current state"""
        # Example: Can't navigate if carrying fragile object
        if 'carry' in command.lower() and self.robot_state['carrying'] == 'fragile_item':
            return False

        # Example: Can't execute if battery is low
        if self.robot_state['battery_level'] < 10:
            return False

        return True
```

## Exercise: Create a Voice Command Pipeline

1. Implement the NaturalLanguageActionMapper node on your humanoid robot
2. Test with various voice commands like "go to kitchen", "pick up the red cup", "say hello world"
3. Add more location mappings to the navigation system
4. Implement error handling for ambiguous commands
5. Add a feedback system to report action completion status
6. Integrate with your existing Whisper voice recognition system

---
This completes Module 4 and the Physical AI & Humanoid Robotics book. You now have a complete system that can understand voice commands, process them with cognitive planning, and execute them as ROS 2 actions on your humanoid robot.