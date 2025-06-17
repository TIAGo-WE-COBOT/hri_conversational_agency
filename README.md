# Conversational agents in ROS

This package is a container for ROS modules, developed for easy integration of conversational agents in ROS.

## Installation

* Open a terminal and move to the `src` folder of your workspace (here assumed to be `~/hri_ws`)
    ```
    cd `~/hri_ws/src`
    ```
* Clone the repo with
    ```
    git clone https://github.com/TIAGo-WE-COBOT/hri_conversational_agency.git
    ```
<!--* Install the common requirements with
    ```
    pip install -r requirements.txt
    ```
-->
* Go back to the root of the workspace and build the package
    ```
    cd ..
    catkin build hri_conversational_agency
    ```

> [!IMPORTANT]
> The above command will not install the requirements for any specific backend. As such, you will only be able to run a pre-scripted bot (`backend=dummy`). <br>
> See the following section to install specific backend(s).

### Backend
#### Ollama

* Install Ollama as described in the [docs](https://github.com/ollama/ollama/blob/main/docs/linux.md) with
    ```
    curl -fsSL https://ollama.com/install.sh | sh
    ```
* Install the Python bindings for Ollama with
    ```
    pip install ollama-python
    ```
* Pull the model you will use with `ollama pull <model>`. To be able to run the `chat.py` node out-of-the-box, you have to pull the model used as default first. To do so:
    * Start Ollama
    ```
    ollama serve
    ```
    and let it run
    * In a different terminal, run
    ```
    ollama pull llama3.2:3b
    ```

#### Ollama and Langchain
* Install Ollama as per instructions in [Installation/Backend/Ollama](#ollama) section.
* Install Langchain and its required components
    ```
    TODO.
    ```
* Pull the model you will use with `ollama pull <model>`. To be able to run the `chat_extended.py` node out-of-the-box, you have to pull the model used as default first. To do so:
    * Start Ollama
    ```
    ollama serve
    ```
    and let it run
    * In a different terminal, run
    ```
    ollama pull qwen3:4b
    ```

<!--
#### GPT4All

TODO. See Issues.

#### OpenAI

- Get your API key from [this link](https://platform.openai.com/account/api-keys) and copy it to your clipboard.
- Create a file named `.env` in the package root, if you do not already have one.
- Open the `.env` file and add the following line
    ```
    OPENAI_API_KEY=<your_api_key>
    ```
    where `<your_api_key>` has to be susbstituted with the actual API key obtained at the first step.

> [!IMPORTANT]
> Make sure that the `.gitignore` file includes a `.env` or `*.env` entry to avoid the file with the API key being tracked by Git. 
-->

## How to use

#### _Dummy_ backend

* Open a terminal (let's call it *T1*), launch `chat.launch` with `backend:=dummy` (default)
    ```
    roslaunch hri_conversational_agency chat.launch
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

* In another terminal, launch `chat.launch` with `backend:=ollama`
    ```
    roslaunch hri_conversational_agency chat.launch backend:=ollama
    ```

* You can now check the model behavior as done for the [dummy backend](#dummy).

#### _Ollama + Langchain_ backend
* Run Ollama with
    ```
    ollama serve
    ```
    and let it run.

* In another terminal, run `chat_extended.py`
    ```
    rosrun hri_conversational_agency chat_extended.py
    ```

* You can now check the model behavior as done for the [dummy backend](#dummy). 
Specifically the `chat_extended.py` script imports the [`multiprompt.py`](./src/hri_conversational_agency/langchain/multiprompt.py) which in turn implements a multi-prompt chain, _i.e._ a chain that call an LLM repeatedly, first to route the user input to the proper downstream chain, and then to generate the answer.
<br>The two downstream chains are implemented as a Retrieval-Augmented Generation model, and a chat model respectively. The behavior of the two chains is controlled via their prompts and context, which can be modified by settin a ROS paramete with the filepath of a YAML file with the same structure as [`multiprompt_default.yaml`](./cfg/multiprompt_default.yaml).

<!--
## Troubleshooting

### - openai.error.AuthenticationError: \<empty message\>
The error is tracked in this [Github issue](https://github.com/openai/openai-python/issues/464). Try to generate a new API key and replace the existing one.
-->

### ROS interface

#### Topics

* Subscribed:
    * `request` (`std_msgs/String`)
        <br>Input messages for the conversational agent.
* Published:
    * `response` (`std_msgs/String`)
        <br>Response message generated by the conversational agent.

#### Services
>[!IMPORTANT]
> The below services are currently implemented for Ollama backend only. If called with a different backend, the `success` field in the returned response will be `False`.

* `set_sys_prompt` (`hri_conversational_agency/SetAgentContent.srv`)
    <br>Set system prompt for the model.
* `set_history` (`hri_conversational_agency/SetHistory.srv`) 
    <br>Set chat history for the model. If called with an empty `history` field, resets the model "memory".
* `set_context` (`hri_conversational_agency/SetAgentContent.srv`)
    <br>Set context for the model.

