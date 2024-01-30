# Conversational agents in ROS

This package is a container for ROS modules, developed for easy integration of conversational agents in ROS.

## Installation

TODO.

## Setup

### OpenAI

- Get your API key from [this link](https://platform.openai.com/account/api-keys)
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

## How to run

TODO.

## How to use

TODO.

## Troubleshooting

### - openai.error.AuthenticationError: \<empty message\>
The error is tracked in this [Github issue](https://github.com/openai/openai-python/issues/464). Try to generate a new API key and replace the existing one.

## TODO

- [ ] modify the package name to `hri_conversational_agency` in the README
- [ ] set the API key in a different way. As of now, one cannot commit changes to (e.g.) `PROMPT_TEMPLATE`
    - [ ] env variable
    - [ ] different file
