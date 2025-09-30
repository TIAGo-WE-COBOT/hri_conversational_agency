# Conversational agents in ROS

This package is a container for ROS modules, developed for easy integration of conversational agents in ROS.

---

## Table of contents
- [Installation](#installation)
    - [Ollama](#ollama)
    - [Ollama and Langchain](#ollama-and-langchain)
- [How to use](#how-to-use)
    - [Simple chat](#simple-chat)
        - [Dummy backend](#dummy-backend)
        - [Ollama backend](#ollama-backend)
    - [Advanced chat (Langchain-based agents)](#advanced-chat-langchain-based-agents)
        - [Multiprompt chain](#multiprompt-chain)
        - [Configuration](#configuration)
- [ROS interface](#ros-interface)
    - [Topics](#topics)
    - [Services](#services)

---

## Installation

* Open a terminal and move to the `src` folder of your workspace (here assumed to be `~/hri_ws`)
    ```
    cd `~/hri_ws/src`
    ```
* Clone the repo with
    ```
    git clone https://github.com/TIAGo-WE-COBOT/hri_conversational_agency.git
    ```
* Go back to the root of the workspace and build the package
    ```
    cd ..
    catkin build hri_conversational_agency
    ```

> [!IMPORTANT]
> The above command will not install the requirements for any specific backend. As such, you will only be able to run the `chat_simple.py`node implementing a pre-scripted bot (`backend=dummy`). <br>
> See the following section to install specific backend(s).

### Ollama

> [!IMPORTANT]
> This installation is required only if you intend to run `chat_advanced.py` or `chat_simple.py` with `ollama` backend.

* Install Ollama as described in the [docs](https://github.com/ollama/ollama/blob/main/docs/linux.md) with
    ```
    curl -fsSL https://ollama.com/install.sh | sh
    ```
* Install the Python bindings for Ollama with
    ```
    pip install ollama-python
    ```
* Pull the model you will use with `ollama pull <model>`. To be able to run the `chat_simple.py` node out-of-the-box, you have to pull the model used as default first. To do so:
    * Start Ollama
    ```
    ollama serve
    ```
    and let it run
    * In a different terminal, run
    ```
    ollama pull llama3.2:3b
    ```

### Langchain

> [!IMPORTANT]
> This installation is required only if you intend to run `chat_advanced.py`.

* Install Ollama as per instructions in [Installation/Backend/Ollama](#ollama) section.
* Install Langchain and its required components
    ```
    pip install -qU langchain-core langchain-ollama langgraph langchain-text-splitters langchain-huggingface langchain-community

    ```
* Pull the model you will use with `ollama pull <model>`. To be able to run the `chat_advanced.py` node out-of-the-box, you have to pull the model used as default first. To do so:
    * Start Ollama
    ```
    ollama serve
    ```
    and let it run
    * In a different terminal, run
    ```
    ollama pull qwen3:4b
    ```

---

## How to use

### Simple chat

The `chat_simple.py` provides the ROS interface for a simple conversational agent (either based on LLM or on a simpler implementation). Through the ROS interface is it also possible to control some agent's configurations such as conversation history and system prompt.

#### _Dummy_ backend

* Open a terminal (let's call it *T1*), launch `chat_simple.launch` with `backend:=dummy` (default)
    ```
    roslaunch hri_conversational_agency chat_simple.launch
    ```
* Open another terminal (*T2*) and start listening to the `/conversational_agent/response` topic
    ```
    rostopic echo /conversational_agent/response
    ```
* Open another terminal (*T3*) and ask something to the conversational agent, by publishing on topic 
    ```
    rostopic pub -1 /conversational_agent/request std_msgs/String "data: 'Ciao! Parlami di te'" 
    ```
    where `'Ciao! Parlami di te'` might be substituted with anything you would like to ask to the agent.
    
    In *T2* you should soon see the response from the conversational agent. Keep chatting as you like!

#### _Ollama_ backend

* Run Ollama with
    ```
    ollama serve
    ```
    and let it run.

* In another terminal, launch `chat_simple.launch` with `backend:=ollama`
    ```
    roslaunch hri_conversational_agency chat_simple.launch backend:=ollama
    ```

* You can now check the model behavior as done for the [dummy backend](#dummy).

### Advanced chat (Langchain-based agents)

The `chat_advanced.py` provides the ROS interface for agents implemented leveraging on the [Langchain](https://www.langchain.com/) framework to implement more complex agent's behavior. The agent configuration is set via YAML file (see [`cfg` folder](./cfg/)), either passed at launch or via ROS service.

> [!NOTE]
> At the time of writing this note, only the `multiprompt.py` implementation is available for the `chat_advanced.py` script.

#### Multiprompt chain
The multiprompt chain implements an **intelligent routing system** that automatically directs user queries to specialized expert chains based on the content and context of the question.

<details><summary>Click here for insight into the `multiprompt` implementation.</summary>

**How it works:**
1. **User input** → **Router** analyzes the query and selects the best _expert_ (sometimes referred to as _destinations_ in the code).
2. **Router** → **Expert chain** processes the query using specialized knowledge/tools.
3. **Expert chain** → **Response** generated using the appropriate method (RAG, chat, or tool).

**Available expert types:**
- **RAG chains**: Use custom context and document retrieval (e.g., historical knowledge, technical documentation)
- **Chat chains**: Simple conversational agents with specific personalities or domains
- **Tool chains**: Execute external functions (weather API, datetime, calculations, etc.)

**Example scenario:**

- User asks *"What's the weather like today?"* → Routes to **weather tool chain**
- User asks *"Tell me about Napoleon"* → Routes to **historian RAG chain** 
- User asks *"How are you?"* → Routes to **general chat chain**

The above behavior can be reproduced using the [`multiprompt_demo_historian.yaml`](./cfg/multiprompt_demo_historian.yaml) configuration file.
</details>

* Run Ollama with
    ```
    ollama serve
    ```
    and let it run.

* In another terminal, launch the `chat_advanced` node
    ```
    roslaunch hri_conversational_agency chat_advanced.launch
    ```

* You can now check the model behavior as done for the [dummy backend](#dummy).

---

## ROS interface

### Topics

* Subscribed:
    * `request` (`std_msgs/String`)
        <br>Input messages for the conversational agent.
* Published:
    * `response` (`std_msgs/String`)
        <br>Response message generated by the conversational agent.

### Services
>[!IMPORTANT]
> The below services are currently implemented for Ollama backend only. If called with a different backend, the `success` field in the returned response will be `False`.

* `set_sys_prompt` (`hri_conversational_agency/SetAgentContent.srv`)
    <br>Set system prompt for the model. Implemented for `chat_simple` only.
* `set_history` (`hri_conversational_agency/SetHistory.srv`) 
    <br>Set chat history for the model. If called with an empty `history` field, resets the model "memory".
* `set_context` (`hri_conversational_agency/SetAgentContent.srv`)
    <br>Set context for the model. Implemented for `chat_simple` only.
* `set_config` (`hri_conversation_agency/SetAgentConfig.srv`)
    <br>Set filepath to a YAML file with the configuration for the `chat_advanced` agent. Implemented for `chat_advanced` only.

---

Author
* Luca :envelope: [luca6.pozzi@mail.polimi.it](mailto:luca6.pozzi@mail.polimi.it)