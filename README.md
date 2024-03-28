# Conversational agents in ROS

This package is a container for ROS modules, developed for easy integration of conversational agents in ROS.

## Installation

1. Install prerequirements. TODO.

2. Clone the repo

3. Build (e.g. with `caktin build`)

4. 
## Setup

### OpenAI

- Get your API key from [this link](https://platform.openai.com/account/api-keys)

#### Alternatives

1. 

- Set it in `src/conversational_agency/openai_utils/cfg.py`

:warning: This will set your personal API key for your local version of the code. If you push it on Github, your private API will be visible to any user having access to your repo. To prevent this, follow the instructions below.

- In a terminal, move to this repo and run
```
git update-index --skip-worktree src/conversational_agency/openai_utils/cfg.py
```

to prevent the file with your API key to be uploaded in Github on push. 

- If, for any reason, you are willing to save in Git/Github changes made to the `cfg.py` file revert the previous command with
```
git update-index --skip-worktree src/conversational_agency/openai_utils/cfg.py
```
2. Actually employed:

- Create a `.env` file and set the API key in as OPENAI_API_KEY = <your_API_key>

- Check that `.env` extension is included in the `.gitignore` file. In this way it will not be pushed on Github.

## How to run

TODO.

## How to use
It's possible to launch the package in four possible configurations:
1. Default: launch the conversational agent script, whisper_node, audio_capture and soundplay in all at once
Depending on where you want to launch audio_capture and soundplay you can: 
2. launch the conversational agent script, whisper_node and soundplay in one terminal using the command:
   ```
   roslaunch hri_conversational_agency chat_vocal.launch audio_capture:=false
   ```
   and then launch audio_capture in another terminal (E.g. TIAGo)
4. launch the conversational agent script, whisper_node and audio_capture in one terminal using the command:
   ```
   roslaunch hri_conversational_agency chat_vocal.launch tts:=false
   ```
   and then launch soundplay in another terminal (E.g. TIAGo)
6. launch the conversational agent script, whisper_node in one terminal using the command:
   ```
   roslaunch hri_conversational_agency chat_vocal.launch audio_capture:=false tts:=false
   ```
   and then launch audio_capture and soundplay in different terminals (in a separate way)

## Troubleshooting

### - openai.error.AuthenticationError: \<empty message\>
The error is tracked in this [Github issue](https://github.com/openai/openai-python/issues/464). Try to generate a new API key and replace the existing one.

## TODO

- [ ] modify the package name to `hri_conversational_agency` in the README
- [ ] set the API key in a different way. As of now, one cannot commit changes to (e.g.) `PROMPT_TEMPLATE`
    - [ ] env variable
    - [ ] different file
