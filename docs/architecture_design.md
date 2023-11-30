# Architecture and Design

In our autonomous robot project, we have adopted a modular architecture inspired by the principles of decentralization. Our robot's design divides functionalities into distinct nodes or modules, each encapsulating specific tasks, such as perception, decision-making, motion control, and others.

The following diagram illustrates the high-level structure of the project and the communication between its different components:

![System Architecture](../images/architecture.jpeg)

## Perception

**Communication:**
- Gazebo Subscriber: Gathering knowledge about the nearby objects.
- Classifier Client: Object classification.
- Update Client: Update object mapping

## Classification

**Communication:**
- Classifier Service: Classifies objects based on their characteristics.

## Reasoning

**Communication:**
- Cocktail Service: Responsible to process the cocktail request.
- Prolog Client: Interacts with prolog knowledge base.
- Goto Client: Informs the controller which object to move to.

## Mapping

**Communication:**
- Rviz Publisher: Broadcast object position to rviz node.
- Get Object Service: Provides the requested object(s) poses.
- Update Service: Updates the map with the new object and respective pose.

## Control

**Communication:**
- Goto Service: Responsible for updating the actions in order to move to the given 
object.
- Client Get Object: Requests information about the object pose.
- Gazebo Subscriber: Receives information about the current robot position.
- Action Publisher: Send movement commands to the robot.
