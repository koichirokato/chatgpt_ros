# chatgpt_ros
ROS wrapper for ChatGPT API

## Dependencies
Python requests modeule.

```
$ sudo apt-get install python3-requests
```

## Setup
You have to set ChatGPT API for environment value of `ChatGPT_API`.

## Publish and Subscribe
This node subscribe ` /input_text` topic and publish `/output_text` topic.

## Usage

```
$ ros2 run chatgpt_ros chatgpt_ros
```

## Referene
This node was created based on a question to ChatGPT (creating a ROS node using the ChatGPT API).
