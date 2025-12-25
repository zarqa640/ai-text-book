---
sidebar_position: 2
---

# Chapter 2: LLM-Based Cognitive Planning

## Cognitive Planning with Large Language Models

Large Language Models (LLMs) serve as the cognitive engine for humanoid robots, enabling high-level reasoning, planning, and decision-making. In this chapter, we'll explore how to integrate LLMs with ROS 2 to create intelligent humanoid robots that can understand complex tasks and generate appropriate action sequences.

## Setting Up LLM Integration with ROS 2

First, install the required packages for LLM integration:

```bash
pip install openai anthropic transformers torch
```

## Basic LLM Node for Cognitive Planning

Here's a ROS 2 node that uses an LLM for cognitive planning:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
from typing import List, Dict

class LLMCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Subscription to receive high-level commands
        self.command_subscription = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            10
        )

        # Publisher for generated action plans
        self.plan_publisher = self.create_publisher(String, 'action_plans', 10)

        # Initialize LLM client
        # For OpenAI GPT:
        openai.api_key = "your-api-key-here"  # Set in environment variable

        # For local models, you can use Hugging Face transformers
        # self.model = AutoModelForCausalLM.from_pretrained("microsoft/DialoGPT-medium")
        # self.tokenizer = AutoTokenizer.from_pretrained("microsoft/DialoGPT-medium")

    def command_callback(self, msg):
        """Process high-level command and generate action plan"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Generate cognitive plan using LLM
        plan = self.generate_plan(command)

        # Publish the plan
        plan_msg = String()
        plan_msg.data = plan
        self.plan_publisher.publish(plan_msg)
        self.get_logger().info(f"Published action plan: {plan}")

    def generate_plan(self, command: str) -> str:
        """Generate an action plan using LLM"""
        try:
            # Define the system prompt for humanoid cognitive planning
            system_prompt = """
            You are a cognitive planning system for a humanoid robot. Your role is to take high-level commands
            and break them down into sequences of executable actions that the robot can perform.

            Available actions:
            - move_to(location)
            - pick_up(object)
            - place_down(object, location)
            - speak(text)
            - gesture(action)
            - detect_object(object)
            - grasp(object)
            - release(object)
            - look_at(location)

            Respond with a sequence of actions in the format:
            1. action1
            2. action2
            3. action3

            Be specific about locations, objects, and parameters.
            """

            # Call the LLM to generate the plan
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command}
                ],
                max_tokens=256,
                temperature=0.3
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            self.get_logger().error(f"Error generating plan: {e}")
            return f"ERROR: Could not generate plan for command '{command}'"
```

## Advanced Planning with Memory and Context

For more sophisticated cognitive planning, implement memory and context awareness:

```python
class AdvancedLLMPlanner(Node):
    def __init__(self):
        super().__init__('advanced_llm_planner')

        # Subscription and publisher setup
        self.command_subscription = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            10
        )
        self.plan_publisher = self.create_publisher(String, 'action_plans', 10)

        # Maintain conversation history for context
        self.conversation_history = []

        # Robot state tracking
        self.robot_state = {
            'location': 'starting_position',
            'carrying': None,
            'last_action': 'idle'
        }

    def generate_contextual_plan(self, command: str) -> str:
        """Generate a plan considering the robot's current state and history"""

        context = f"""
        Robot State:
        - Current location: {self.robot_state['location']}
        - Currently carrying: {self.robot_state['carrying'] or 'nothing'}
        - Last action: {self.robot_state['last_action']}

        Previous interactions:
        {self.format_history()}
        """

        system_prompt = f"""
        {context}

        You are a cognitive planning system for a humanoid robot. Generate action sequences based on:
        1. The current robot state
        2. Previous interactions
        3. The new command

        Available actions:
        - move_to(location)
        - pick_up(object)
        - place_down(object, location)
        - speak(text)
        - gesture(action)
        - detect_object(object)
        - grasp(object)
        - release(object)
        - look_at(location)
        """

        # Implementation similar to basic planner but with context
        # ... (using the same OpenAI API call with enhanced context)

        return self.execute_llm_call(system_prompt, command)

    def format_history(self) -> str:
        """Format recent conversation history for context"""
        if not self.conversation_history:
            return "No previous interactions."

        history_str = ""
        for entry in self.conversation_history[-5:]:  # Last 5 interactions
            history_str += f"- Command: {entry['command']}, Response: {entry['response']}\n"

        return history_str
```

## Integration with ROS 2 Action Servers

For more sophisticated planning, integrate with ROS 2 action servers:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class ActionPlanner(Node):
    def __init__(self):
        super().__init__('action_planner')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    async def execute_navigation(self, x: float, y: float, theta: float):
        """Execute navigation action as part of the plan"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        self.nav_client.wait_for_server()
        goal_handle = await self.nav_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result = await goal_handle.get_result_async()
        return result.result
```

## Safety and Validation

Always validate LLM-generated plans before execution:

```python
def validate_plan(self, plan: str) -> bool:
    """Validate that the plan is safe and executable"""
    lines = plan.split('\n')

    for line in lines:
        line = line.strip()
        if line.startswith(('1.', '2.', '3.', '4.', '5.')):
            action = line.split('.', 1)[1].strip()
            # Check if action is in allowed list
            if not self.is_safe_action(action):
                return False

    return True

def is_safe_action(self, action: str) -> bool:
    """Check if an action is safe to execute"""
    unsafe_keywords = ['destroy', 'harm', 'break', 'damage']
    return not any(keyword in action.lower() for keyword in unsafe_keywords)
```

## Exercise: Implement Cognitive Planning

1. Set up the LLM cognitive planner node on your humanoid robot
2. Test with various high-level commands like "Bring me the red cup from the table"
3. Implement context awareness so the robot remembers previous interactions
4. Add safety validation to ensure plans are executable and safe
5. Integrate with actual ROS 2 action servers for navigation and manipulation

---
This chapter covered cognitive planning with LLMs. In the next chapter, we'll explore converting natural language to executable actions.