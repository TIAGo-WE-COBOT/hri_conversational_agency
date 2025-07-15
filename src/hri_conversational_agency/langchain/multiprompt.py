"""
A conversational agent that generates responses to user input based on Ollama models. The integration with Langchain allows for the implementation of advanced functionalities such as document retrieval, and multi-prompt chains.
"""

import os
import re
import yaml
from operator import itemgetter
from typing import Literal
from typing_extensions import TypedDict

from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_huggingface import HuggingFaceEmbeddings
from langchain_community.vectorstores import FAISS
from langchain_core.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnableConfig
from langgraph.graph import END, START, StateGraph

from hri_conversational_agency.base import BaseChatter
from hri_conversational_agency.logger import Logger
import hri_conversational_agency.langchain.tools as tools_utils
            

class LangchainChatter(BaseChatter):
    SYSTEM_COMPONENTS = ['llm', 'router'] # these components might have associated configurations (i.e. `_cfg` attributes) or chains (i.e. `_chain` attributes) but they are not considered "expert" chains, and treated in a special way.
    # Define schema for multi-prompt chain output
    class RouteQuery(TypedDict):
        """Route query to destination expert."""
        destination: str  # Dynamic - will be updated based on available chains
    
    # For LangGraph, we will define the state of the graph to hold the query,
    # destination, and final answer.
    class State(TypedDict):
        query: str
        destination: 'LangchainChatter.RouteQuery'
        answer: str
        history: list[dict] # list of messages in the format 
                            # {
                            #   'role': 'system' | 'user' |'assistant', 
                            #   'content': 'message content'
                            # }

    def __init__(self, config):
        """Initialize the LangchainChatter object.

        Args:
            config (str): Path to YAML configuration file
        """
        # Initialize the agent (empty) memory
        self.set_history([])
        # Load the configuration from the YAML file
        self.set_config(config)
        # Initialize the logger to store the dialogue history
        self.log = Logger(
            agent_name='langchain - ' + self.llm_cfg["model"])
        self.log.logfile_open()
        print("Ready to chat!")

    #region ROS interface methods
    def generate_response(self, request):
        """Generate a response to the given request using the multi-prompt chain.

        Args:
            request (str): The input query to the agent. It can be a question or a statement that the agent will respond to.

        Returns:
            string: The next line in the script.
        """
        # Log the input for later inspection of the dialogue
        self.log.log_input(request)
        # Generate the response
        state = self.app.invoke({"query": request, "history": self.messages})
        response = self._format_answer(state["answer"])
        # Add the response to the messages to maintain the chat history
        self.messages += [
            {'role': 'user', 'content': request},
            {'role': 'assistant', 'content': response}
        ]
        # Log the output for later inspection of the dialogue
        self.log.log_output(f"[{state['destination']['destination'].lower()}] {response}")
        return response

    def set_config(self, yaml_path):
        """Receive filepath to a YAML file containing the configuration of the agent."""
        with open(yaml_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        # 1. Set/Update LLM config (always required)
        self._update_cfg(cfg.pop("model", {}), "llm", self._build_llm)
        # 2. Get chains configuration
        chains_dict = cfg.pop("chains", [])
        # 3. Set/Update router config with extracted experts
        router_cfg = cfg.pop("router", {})
        experts_dict = {name: chain_cfg["description"] 
                       for name, chain_cfg in chains_dict.items() 
                       if "description" in chain_cfg}
        router_cfg["experts"] = experts_dict
        self._update_cfg(router_cfg, "router", self._build_router_chain)
        # 4. Set/Update each chain configuration
        for chain_name, chain_config in chains_dict.items():
            type_to_builder = {
                'rag': self._build_rag_chain,
                'chat': self._build_chat_chain,
                'tool': lambda: self._build_tool_chain(chain_name),
            }
            builder_method = type_to_builder.get(chain_config.get('type', None), None)
            if builder_method:
                self._update_cfg(chain_config, chain_name, builder_method)
            else:
                print("Warning: No builder method found for chain '{}'".format(chain_name))
        # 5. Set/Update RouteQuery destinations and build the app
        self._update_route_query_destinations()
        self._build_app()
        print(dir(self))
    
    def set_history(self, history):
        """Receive a list of tuples (role, content) to set the history of the agent. Any previous history will be overwritten.

        Args:
            history ([(str, str)]): List of tuples (role, content) to set the history of the agent.
        """
        self.messages = [{'role': role, 'content': content} 
                         for role, content in history
                         if role in ['user', 'assistant', 'system']]
    #endregion

    # region StateGraph nodes definition
    def router_query(self, state: State, config: RunnableConfig):
        """Invoke the router chain to determine the destination expert for the input query.
        
        Args:
            state (State): The current state of the graph containing the input query and history.
            config (RunnableConfig): Runtime configuration for the router chain invocation.
        
        Returns:
            dict: A dictionary with the destination expert for the query.
        """
        destination = self.router_chain.invoke(state["query"], config)
        return {"destination": destination}
    
    def chat_query(self, state: State, config: RunnableConfig):
        """Invoke the chat chain to generate a response based on the input query and history.

        Args:
            state (State):The current state of the graph containing the input query and history.
            config (RunnableConfig): Runtime configuration for the chat chain invocation.

        Returns:
            dict: A dictionary with the generated answer from the chat chain.
        """
        destination = state["destination"]["destination"].lower()
        chain = getattr(self, f"{destination}_chain")
        return {"answer": chain.invoke({"input": state["query"],
                                         "history": state["history"]}, config)}

    def rag_query(self, state: State, config: RunnableConfig):
        """Invoke the RAG chain to retrieve relevant information from context and generate an answer based on the input query and history.

        Args:
            state (State): The current state of the graph containing the input query and history.
            config (RunnableConfig): Runtime configuration for the RAG chain invocation.

        Returns:
            dict: A dictionary with the generated answer from the RAG chain.
        """
        destination = state["destination"]["destination"].lower()
        chain = getattr(self, f"{destination}_chain")
        return {"answer": chain.invoke({"input": state["query"],
                                         "history": state["history"]}, config)}

    def tool_query(self, state: State, config: RunnableConfig):
        """Invoke a tool chain based on the destination specified in the state.

        Args:
            state (State): The current state of the graph containing the input query and destination.
            config (RunnableConfig): Runtime configuration for the tool chain invocation.
        
        Returns:
            dict: A dictionary with the answer from the invoked tool chain.
        """
        destination = state["destination"]["destination"].lower()
        
        # Get the appropriate tool chain
        tool_chain = getattr(self, f"{destination}_chain", None)
        if tool_chain:
            return {"answer": tool_chain()}
        else:
            return {"answer": f"Tool '{destination}' not available."}

    def router_select_node(self, state: State):
        """Route to available chains dynamically.
        
        Args:
            state (State): The current state of the graph containing the destination.

        Returns:
            str: The name of the next node to route to based on the destination.
        """
        destination = state["destination"]["destination"].lower()
        # Check if the requested chain actually exists
        print("Routing to destination:", destination)
        if hasattr(self, f"{destination}_chain"):
            return f"{destination}_query"
        else:
            raise ValueError(
                f"Chain '{destination}' not available. Available chains: {[attr_name for attr_name in dir(self) if attr_name.endswith('_chain')]}"
            )
    #endregion
    
    #region Chain objects definition
    def _build_app(self):
        """Build the multi-prompt chain with only available chains."""
        print("Building the multi-prompt chain...", end=' ', flush=True)
        graph = StateGraph(self.State)
        # Always add router
        graph.add_node("router_query", self.router_query)
        graph.add_edge(START, "router_query")
        # Discover all available chains dynamically
        available_chains = []
        # Find all attributes ending with '_chain'
        for attr_name in dir(self):
            if attr_name.endswith('_chain') and not attr_name.startswith('_') and not attr_name[:-6] in LangchainChatter.SYSTEM_COMPONENTS:
                chain_name = attr_name[:-6] # remove suffix '_chain'
                # Determine the query method based on chain type from config
                chain_cfg = getattr(self, f"{chain_name}_cfg", {})
                chain_type = chain_cfg.get("type")
                query_method = getattr(self, f"{chain_type}_query", None)
                if not query_method:
                    print(f"Unknown chain type for '{chain_name}', skipping")
                    continue
                # Add the chain node and edge
                graph.add_node(f"{chain_name}_query", query_method)
                graph.add_edge(f"{chain_name}_query", END)
                available_chains.append(chain_name)
        # Add conditional routing
        graph.add_conditional_edges("router_query", self.router_select_node)
        # Build the app with the defined graph
        self.app = graph.compile()
        print(f"Done. Available chains: {available_chains}")
    
    def _build_llm(self):
        """Build the LLM instance from configuration."""
        llm_cfg = self.llm_cfg.copy()
        self.llm = ChatOllama(
            model=llm_cfg.pop("model"),
            **llm_cfg,
        )

    def _build_router_chain(self):
        """Build the router chain for the agent. The chain will be used to route the input query to the appropriate expert based on the router prompt and the available experts."""
        print("Building router chain...", end=' ', flush=True)
        # Set prompt and available "experts" for the routing chain. The experts are defined as a dictionary where the keys are the expert names and the values are their descriptions.
        self.router_chain = (
            self._format_chat_prompt(
                self._format_router_prompt(
                    self.router_cfg["prompt"], 
                    self.router_cfg["experts"]
                )
            ) 
            | self.llm.with_structured_output(self.RouteQuery)
        )
        print("Done.")
    
    def _build_rag_chain(self):
        """Build the RAG chain for the agent. The chain will be used to retrieve relevant context from the vector store and generate a response based on the context. 
        Saved embeddings can be loaded from disk if a `faiss_path` is provided in the `rag_kwargs`.
        """
        # Get the current chain name being built
        current_chain = getattr(self, '_current_chain_name', 'rag')
        print(f"Building {current_chain} (RAG) chain...", end=' ', flush=True)
        # Use the specific chain's config
        chain_cfg = getattr(self, f"{current_chain}_cfg")
        # Unpack kwargs for RAG chain elements
        text_splitter_kwargs = chain_cfg.get('text_splitter', {})
        embedding_model_kwargs = chain_cfg.get('embedding_model', {})
        faiss_kwargs = chain_cfg.get('faiss', {})
        # Create embeddings for the context
        embedding_model = embedding_model_kwargs.pop(
            'model_name',
            "sentence-transformers/all-mpnet-base-v2"
        )
        embeddings = HuggingFaceEmbeddings(
            model_name=embedding_model,
            model_kwargs=embedding_model_kwargs,
        )
        # Get path to dir to save/load the FAISS index
        embeddings_dir = os.path.join( 
            os.path.dirname(os.path.abspath(__file__)),
            '../../../embeddings'
        )
        index_dir = faiss_kwargs.get('index_dir', None)
        index_path = os.path.join(embeddings_dir, index_dir) if index_dir else None
        if index_dir and os.path.exists(index_path):
            # If a FAISS path is provided, load the vector store from disk
            vector_store = FAISS.load_local(
                index_path,
                embeddings,
                allow_dangerous_deserialization=True
            )
        else: # Otherwise, create a new vector store (and save it)
            # Split the context into chunks
            text_splitter = RecursiveCharacterTextSplitter(**text_splitter_kwargs)
            texts = text_splitter.split_text(chain_cfg["context"]["content"])
            # Create a vector store from the context chunks
            vector_store = FAISS.from_texts(texts, embeddings)
            # Save the vector store to disk
            if index_path:
                vector_store.save_local(index_path)
        # Create a retriever from the vector store
        retriever = vector_store.as_retriever(
            search_type="similarity",
            search_kwargs=faiss_kwargs.get('search', {}),
        )
        # Build the RAG chain using the retriever and the prompt
        rag_chain = (
            {
                "context": itemgetter("input") | retriever,
                "input": itemgetter("input"),
                "history": itemgetter("history")          
            }
            | self._format_chat_prompt(chain_cfg["prompt"])
            | self.llm
            | StrOutputParser()
        )
        # Store the chain with the specific name
        setattr(self, f"{current_chain}_chain", rag_chain)
        print("Done.")

    def _build_chat_chain(self):
        """Build a chat chain for the agent.
        """
        # Get the current chain name being built
        current_chain = getattr(self, '_current_chain_name', 'fallback')
        print(f"Building {current_chain} (chat) chain...", end=' ', flush=True)
        # Use the specific chain's config
        chain_cfg = getattr(self, f"{current_chain}_cfg")
        # Get the prompt and handle optional variables
        prompt = chain_cfg["prompt"]
        # Check if {interests} is in the prompt and substitute if not provided
        if "{interests}" in prompt:
            interests_list = chain_cfg.get("interests", [])
            if interests_list:
                interests_str = "Your interests include: " + ", ".join(interests_list) + "."
            else:
                interests_str = ""
            prompt = prompt.replace("{interests}", interests_str)
        # Build the chat chain
        chat_chain = (
            self._format_chat_prompt(prompt) 
            | self.llm 
            | StrOutputParser()
        )
        # Store the chain with the specific name
        setattr(self, f"{current_chain}_chain", chat_chain)
        print("Done.")
    
    def _build_tool_chain(self, tool_name):
        """Build a tool chain for a tool identified by name in config."""
        print(f"Building {tool_name} (tool) chain...", end=' ', flush=True)
        # Get the tool configuration
        tool_cfg = getattr(self, f"{tool_name}_cfg", {})
        # Get tool class from the tool name
        tool_class = tools_utils.get_tool_class(tool_name)
        # If the tool exists, build the chain
        if tool_class:
            # Build the tool chain using the tool class
            tool_chain = tool_class.build_chain(tool_cfg)
            # Store the chain as class attribute
            setattr(self, f"{tool_name}_chain", tool_chain)
            print("Done.")
        else:
            print("Failed - tool class not found for {}".format(tool_name))
        
    #endregion
    
    # region Helper methods for configuration and chain building
    def _update_cfg(self, new_cfg, cfg_key, builder_method):
        """Helper method to check, merge, and rebuild config sections.
        
        Args:
            new_cfg (dict): The new config dictionary to merge.
            cfg_key (str): The config key (e.g., "model", "router", "rag").
            builder_method (callable): The method to call if config changed (e.g., self._build_llm).
        
        Returns:
            bool: True if the config was updated and rebuilt. False otherwise.
        """
        cfg_var_name = cfg_key + "_cfg" # name to be assigned to config variable
        # Check if the config variable exists, if not create it
        merged_cfg = self._merge_dicts_recursive(
            getattr(self, cfg_var_name, {}),
            new_cfg
        )
        # # If the config has changed, update the config variable and rebuild the chain
        if not hasattr(self, cfg_var_name) or merged_cfg != getattr(self, cfg_var_name, {}):
            # Update the config variable
            setattr(self, cfg_var_name, merged_cfg)
            # Set the current chain name for the builder to use
            self._current_chain_name = cfg_key
            builder_method()
            # Clean up
            delattr(self, '_current_chain_name')
            return True
        return False
    
    def _update_route_query_destinations(self):
        """Update RouteQuery destinations based on available chains."""
        available_chains = []
        # Find chains dynamically by checking attributes ending with '_cfg'
        for attr_name in dir(self):
            if attr_name.endswith('_cfg') and not attr_name.startswith('_'):
                chain_name = attr_name[:-4]  # Remove '_cfg' suffix
                # Skip system configs (as `llm_cfg`, or `router_cfg`)
                if chain_name in LangchainChatter.SYSTEM_COMPONENTS:
                    continue    
                # Check if the actual chain object exists too
                if hasattr(self, f"{chain_name}_chain"):
                    available_chains.append(chain_name)
        # Update the Literal type dynamically
        if available_chains:
            self.RouteQuery.__annotations__["destination"] = Literal[tuple(available_chains)]
            print(f"Updated RouteQuery destinations: {available_chains}")
        else:
            print("WARNING: No available chains found for RouteQuery!")
    
    @staticmethod
    def _merge_dicts_recursive(old, new):
        """Recursively merge two dictionaries.
        Behavior:
        * a key exists in both dictionaries: the value from `new` will overwrite the value from `old`.
        * a key exists in `old` but not in `new`: the value from `old` will be kept.
        * a key exists in `new` but not in `old`: the value from `new` will be added to `old`.
        The behavior is applied recursively for nested dictionaries.

        Args:
            old (dict): The existing dictionary to merge into.
            new (dict): The new dictionary to merge from.

        Returns:
            dict: The merged dictionary.
        """
        merged = old.copy()
        for k, v in new.items():
            if (
                k in merged
                and isinstance(merged[k], dict)
                and isinstance(v, dict)
            ):
                merged[k] = LangchainChatter._merge_dicts_recursive(merged[k], v)
            else:
                merged[k] = v
        return merged
    # endregion

    #region Static methods for string/prompt formatting
    @staticmethod
    def _format_router_prompt(prompt, experts):
        """Format the router prompt with the experts.
        
        Args:
            prompt (str): The prompt to format.
            experts (dict): A dictionary of experts where keys are expert names and values are their descriptions.
        Returns:
            str: The formatted prompt with the experts listed.
        """
        experts_str = "\n".join([f"- {name}: {desc}" 
                             for name, desc in experts.items()])
        return prompt.format(experts=experts_str)
    
    @staticmethod
    def _format_answer(text: str) -> str:
        """Remove <think>...</think> blocks, emojis, and trailing newlines.
        
        Args:
            text (str): The text to format.
        
        Returns:
            str: The formatted text with <think> blocks and emojis removed, and trailing newlines.
        """
        # Remove all <think>...</think> blocks (including newlines inside)
        text = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL)
        # Remove emojis
        emoj = re.compile("["
        u"\U0001F600-\U0001F64F"  # emoticons
        u"\U0001F300-\U0001F5FF"  # symbols & pictographs
        u"\U0001F680-\U0001F6FF"  # transport & map symbols
        u"\U0001F1E0-\U0001F1FF"  # flags (iOS)
        u"\U000024C2-\U0001F251"
        u"\U0001f926-\U0001f937"
        u"\U00010000-\U0010ffff"
        u"\u2600-\u2B55"
        u"\u200d"
        u"\u23cf"
        u"\u23e9"
        u"\u231a"
        u"\ufe0f"  # dingbats
        u"\u3030"
                      "]+", re.UNICODE)
        text = re.sub(emoj, '', text)
        text = text.replace('*', ' ')  # remove asterisks
        # Remove leading/trailing whitespace and newlines
        return text.strip() + '\n\n'

    @staticmethod
    def _format_chat_prompt(prompt):
        """Format the chat prompt for the agent.
        This method creates a ChatPromptTemplate with the given prompt as system message.

        Args:
            prompt (str): The string to format as system message in the chat prompt.

        Returns:
            langchain_core.prompts.ChatPromptTemplate: The formatted chat prompt template.
        """
        return ChatPromptTemplate.from_messages(
            [
                ("system", prompt),
                ("human", "{input}"),
            ]
        )
    #endregion