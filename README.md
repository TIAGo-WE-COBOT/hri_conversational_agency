# Conversational agents in ROS

This package is a container for ROS modules, developed for easy integration of conversational agents in ROS.

## Installation

TODO.

## Setup

### OpenAI

- Get your API key from [this link](https://platform.openai.com/account/api-keys)
- Set it in `src/conversational_agency/openai_utils/cfg.py`
- In a terminal, move to this repo and run
```
git update-index --assume-unchanged src/conversational_agency/openai_utils/cfg.py
```

This will set your personal API key for your local version of the code, but prevent it to be uploaded in Github on push. 

## How to run

TODO.

## How to use

TODO.

## Troubleshooting

### - openai.error.AuthenticationError: \<empty message\>
The error is tracked in this [Github issue](https://github.com/openai/openai-python/issues/464). Try to generate a new API key and replace the existing one.