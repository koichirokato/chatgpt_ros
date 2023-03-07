# chatgpt_ros
ROS wrapper for ChatGPT API

## Dependencies
Python requests modeule.

```
$ sudo apt-get install python3-requests
```

## Setup
You have to set ChatGPT API for environment value of `ChatGPT_API`.

## Usage
### Publish and Subscribe
`chatgpt_ros` node subscribe ` /input_text` topic and publish `/output_text` topic.

### Run

```
$ ros2 run chatgpt_ros chatgpt_ros
```

## ROS Service
`chatgpt_ros_service` node is the server of ros service of `chat_gpt_service`.
You can see srv file is in `chatgpt_ros_interfaces/srv`.

### Run

```
$ ros2 run chatgpt_ros chatgpt_ros_service
```

## Referene
This node was created based on a question to ChatGPT (creating a ROS node using the ChatGPT API).
