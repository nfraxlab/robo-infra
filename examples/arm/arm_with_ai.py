#!/usr/bin/env python3
"""LLM-controlled robot arm using ai-infra Agent.

This example demonstrates AI-powered robot control:
- Generating function tools from the arm controller
- Natural language control with ai-infra Agent
- Streaming responses for real-time feedback
- Provider fallbacks for reliability

Requirements:
    pip install ai-infra robo-infra

Run:
    # Set your LLM provider API key
    export OPENAI_API_KEY=your-key-here
    # Or: export ANTHROPIC_API_KEY=your-key-here

    python arm_with_ai.py
"""

import asyncio

from ai_infra import Agent

from robo_infra.controllers.joint_group import JointGroup, JointGroupConfig
from robo_infra.core.actuator import SimulatedActuator
from robo_infra.core.types import Limits
from robo_infra.integrations.ai_infra import controller_to_tools


def create_arm() -> JointGroup:
    """Create a 4-DOF robot arm with simulated servos.

    Returns:
        Configured and enabled JointGroup controller.
    """
    # Create simulated servo actuators for each joint
    joints = {
        "base": SimulatedActuator(
            name="base",
            limits=Limits(min=0, max=360, default=180),
            unit="degrees",
        ),
        "shoulder": SimulatedActuator(
            name="shoulder",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
        "elbow": SimulatedActuator(
            name="elbow",
            limits=Limits(min=0, max=150, default=75),
            unit="degrees",
        ),
        "wrist": SimulatedActuator(
            name="wrist",
            limits=Limits(min=0, max=180, default=90),
            unit="degrees",
        ),
    }

    # Configure the arm with home and named positions
    config = JointGroupConfig(
        name="robot_arm",
        home_positions={
            "base": 180,
            "shoulder": 90,
            "elbow": 75,
            "wrist": 90,
        },
        named_positions={
            "ready": {"base": 180, "shoulder": 45, "elbow": 90, "wrist": 90},
            "rest": {"base": 0, "shoulder": 0, "elbow": 0, "wrist": 0},
            "pickup": {"base": 90, "shoulder": 60, "elbow": 120, "wrist": 45},
        },
    )

    # Create the arm controller
    arm = JointGroup(name="robot_arm", joints=joints, config=config)

    return arm


def demo_tool_generation(arm: JointGroup) -> list:
    """Demonstrate generating AI tools from the arm controller.

    Args:
        arm: The robot arm controller.

    Returns:
        List of generated callable tools.
    """
    print("\n--- Generating AI Tools ---")

    # Generate function tools for AI agent
    # These are plain Python functions compatible with ai-infra Agent
    tools = controller_to_tools(arm)

    print(f"Generated {len(tools)} tools for AI agent:")
    for tool in tools:
        doc = tool.__doc__ or "No description"
        # Truncate long docstrings for display
        doc_preview = doc.split("\n")[0][:60]
        print(f"  - {tool.__name__}: {doc_preview}...")

    return tools


def demo_basic_control(agent: Agent, arm: JointGroup) -> None:
    """Demonstrate basic natural language control.

    Args:
        agent: The AI agent with arm tools.
        arm: The robot arm controller.
    """
    print("\n--- Basic Natural Language Control ---")

    # Natural language robot control
    commands = [
        "Move the shoulder to 45 degrees and the elbow to 90 degrees",
        "What is the current arm status?",
        "Home all joints",
    ]

    for command in commands:
        print(f"\nCommand: {command}")
        try:
            result = agent.run(command)
            print(f"Response: {result}")
            print(f"Positions: {arm.get_positions()}")
        except Exception as e:
            print(f"Error: {e}")
            print("(This is expected if no API key is configured)")
            break


async def demo_streaming(agent: Agent) -> None:
    """Demonstrate streaming responses for real-time feedback.

    Args:
        agent: The AI agent with arm tools.
    """
    print("\n--- Streaming Control ---")
    print("Command: Move shoulder to 60 degrees")
    print("Response: ", end="")

    try:
        async for event in agent.astream("Move shoulder to 60 degrees"):
            if event.type == "token":
                print(event.content, end="", flush=True)
            elif event.type == "tool_start":
                print(f"\n[Calling {event.tool}...]", end="")
            elif event.type == "tool_end":
                print(f" [Done]", end="")
        print()  # Final newline
    except Exception as e:
        print(f"\nStreaming error: {e}")
        print("(This is expected if no API key is configured)")


def demo_provider_fallbacks(agent: Agent) -> None:
    """Demonstrate provider fallbacks for reliability.

    Args:
        agent: The AI agent with arm tools.
    """
    print("\n--- Provider Fallbacks ---")
    print("Trying multiple providers in order: OpenAI -> Anthropic -> Google")

    try:
        result = agent.run_with_fallbacks(
            messages=[{"role": "user", "content": "Get the current arm status"}],
            candidates=[
                ("openai", "gpt-4o"),
                ("anthropic", "claude-sonnet-4-20250514"),
                ("google_genai", "gemini-2.0-flash"),
            ],
        )
        print(f"Response: {result}")
    except Exception as e:
        print(f"All providers failed: {e}")
        print("(This is expected if no API keys are configured)")


def main() -> None:
    """Demonstrate AI-controlled robot arm."""
    print("=" * 60)
    print("Robot Arm Example - AI Control with ai-infra Agent")
    print("=" * 60)

    # Create and enable the arm
    arm = create_arm()
    arm.enable()
    arm.home()
    print(f"\nCreated arm: {arm.name}")
    print(f"Initial positions: {arm.get_positions()}")

    # Generate tools from the controller
    tools = demo_tool_generation(arm)

    # Create ai-infra Agent with the generated tools
    print("\n--- Creating AI Agent ---")
    agent = Agent(tools=tools)
    print("Agent created with arm control tools")

    # Demonstrate basic control
    demo_basic_control(agent, arm)

    # Demonstrate streaming (async)
    print("\n" + "-" * 40)
    asyncio.run(demo_streaming(agent))

    # Demonstrate provider fallbacks
    print("\n" + "-" * 40)
    demo_provider_fallbacks(agent)

    # Cleanup
    print("\n--- Cleanup ---")
    arm.disable()
    print("Arm disabled")

    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
