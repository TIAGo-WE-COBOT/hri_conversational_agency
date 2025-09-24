"""
A conversational agent that generates responses to user input based on Ollama models. The integration with Langchain allows for the implementation of advanced functionalities such as document retrieval, and multi-prompt chains.
"""

import os
import re
import yaml
from operator import itemgetter
from typing import Literal
from typing_extensions import TypedDict
from functools import partial

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

# Optional import for tools
try:
    import hri_conversational_agency.langchain.tools as tools_utils
    TOOLS_AVAILABLE = True
except ImportError:
    TOOLS_AVAILABLE = False
    tools_utils = None


class LangchainChatter(BaseChatter):
    SYSTEM_COMPONENTS = ['llm', 'router']  # these components might have associated configurations but are not considered "expert" chains
    SYSTEM_FIELDS = ['context', 'input', 'history']  # fields that are managed by the system and should not be altered by parametric formatting (see `_format_parametric_field`)
    # Define schema for multi-prompt chain output
    class RouteQuery(TypedDict):
        """Route query to destination expert."""
        destination: str  # Dynamic - will be updated based on available chains
    
    # For LangGraph, we define the state of the graph to hold the query, destination, and final answer.
    class State(TypedDict):
        query: str
        destination: 'LangchainChatter.RouteQuery'
        answer: str
        history: list[dict]  # list of messages in the format 
                            # {
                            #   'role': 'system' | 'user' |'assistant', 
                            #   'content': 'message content'
                            # }

    def __init__(self, config, verbose=True):
        """Initialize the LangchainChatter object.

        Args:
            config (str): Path to YAML configuration file
        """
        # Initialize the chain registry
        self._chain_registry = {
            'configs': set(),       # track all _cfg attributes
            'chains': set(),        # track all _chain attributes  
            'destinations': set()   # track destination chains (non-system)
        } # NOTE. 'destinations' chains are sometimes referred to as 'experts' elsewhere in the code.
        # Initialize the agent (empty) memory
        self.set_history([])
        # Initialize the logger to store the dialogue history
        self.log = Logger(verbose=verbose)
        self.log.logfile_open()
        # Initialize an empty dict to track the app stats
        self.stats = {}
        # Load the configuration from the YAML file
        self.set_config(config)
        
    #region Chain registry methods
    def _register_config(self, config_name):
        """Register a configuration in the registry."""
        self._chain_registry['configs'].add(config_name)
        # If it is not a system component, it is a destination
        if config_name not in self.SYSTEM_COMPONENTS:
            self._chain_registry['destinations'].add(config_name)

    def _register_chain(self, chain_name):
        """Register a chain in the registry."""
        self._chain_registry['chains'].add(chain_name)

    def _unregister_config(self, config_name):
        """Unregister a configuration from the registry."""
        self._chain_registry['configs'].discard(config_name)
        self._chain_registry['destinations'].discard(config_name)

    def _unregister_chain(self, chain_name):
        """Unregister a chain from the registry."""
        self._chain_registry['chains'].discard(chain_name)

    def _get_destination_chains(self):
        """Get all destination chains (non-system chains that exist)."""
        return [name for name in self._chain_registry['destinations'] 
                if name in self._chain_registry['chains']]

    def debug_registry(self):
        """Debug method to inspect registry state."""
        print("=== CHAIN REGISTRY DEBUG ===")
        print(f"Configs: {sorted(self._chain_registry['configs'])}")
        print(f"Chains: {sorted(self._chain_registry['chains'])}")
        print(f"Destinations: {sorted(self._chain_registry['destinations'])}")
        print(f"Available destination chains: {self._get_destination_chains()}")
        print("============================")
    #endregion

    #region ROS interface methods
    def generate_response(self, request):
        """Generate a response to the given request using the multi-prompt chain.

        Args:
            request (str): The input query to the agent. It can be a question or a statement that the agent will respond to.

        Returns:
            string: The response from the agent.
        """
        # If agent seed is pending, return it and clear the flag
        if getattr(self, "seed_agent", None) is not None:
            print("Input {} received while seed pending, returning seed response".format(request))
            response = self.seed_agent
            self.seed_agent = None
            # Add to history as assistant message
            self.messages.append({'role': 'assistant', 'content': response})
            self.log.log_output(f"[seed] {response}")
            return response
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

    def get_agent_stats(self):
        """Return a copy of the router call statistics."""
        return dict(self.stats)

    def set_config(self, yaml_path):
        """Receive filepath to a YAML file containing the configuration of the agent."""
        with open(yaml_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        # Handle seeds
        seed_agent = cfg.pop("seed_agent", None)
        seed_user = cfg.pop("seed_user", None)
        # Check incremental flag.
        incremental = cfg.pop("incremental", False)
        if not incremental: 
            # If False (or not set), the agent is rebuilt from scratch.
            # Use registry to efficiently clear everything
            for config_name in list(self._chain_registry['configs']):
                self._clear_config_and_chain(config_name)
            # Clear the registry
            self._chain_registry = {
                'configs': set(),
                'chains': set(),
                'destinations': set()
            }
        # Track what changed for incremental updates
        updated_chains = []
        # 1. Set/Update LLM config (always required)
        llm_updated = self._update_cfg(cfg.pop("model", {}), "llm", self._build_llm)
        self.log.set_agent_name(f'langchain - {self.llm_cfg["model"]}')
        # 2. Get chains configuration
        chains_dict = cfg.pop("chains", {})
        # 3. Handle removed chains (only in incremental mode)
        if incremental:
            removed_chains = self._remove_deleted_chains(chains_dict)
            if removed_chains:
                updated_chains.extend(removed_chains)
        # 4. Extract router config
        router_cfg = cfg.pop("router", {})
        # 5. Set/Update each chain configuration
        for chain_name, chain_config in chains_dict.items():
            # Check if there is any parametric entry in the chain config and format it
            for key, value in chain_config.items():
                #if key in self.SYSTEM_ENTRIES: 
                #    continue  # skip system fields
                # NOTE. The above two lines are kept to remind the logic, but there is not reason to skip any entry here.
                chain_config[key] = self._resolve_parametric_entry(
                    value, skip_fields=self.SYSTEM_FIELDS
                )
            # The chain is named after the name field if present, otherwise use the key itself (and add the name field).
            if chain_config.get("name", None) is None:
                chain_config["name"] = chain_name
            else:
                chain_name = chain_config["name"]
            # Initialize stats tracking for the chain if not present
            if chain_name not in self.stats:
                self.stats[chain_name] = {"calls_count": 0}
            chain_type = chain_config.get('type')
            if chain_type == 'rag':
                builder_method = self._build_rag_chain
            elif chain_type == 'chat':
                builder_method = self._build_chat_chain
            elif chain_type == 'tool':
                builder_method = partial(self._build_tool_chain, chain_name)
            else:
                print(f"Warning: No builder method found for chain '{chain_name}' with type '{chain_type}'")
                continue
            # Update the chain and track if it changed
            if self._update_cfg(chain_config, chain_name, builder_method):
                updated_chains.append(chain_name)
        # 6. Set/Update router with available experts
        experts_dict = {chain_cfg["name"]: chain_cfg["description"] 
                       for __, chain_cfg in chains_dict.items() 
                       if "description" in chain_cfg}
        router_cfg["experts"] = experts_dict
        # 7. Set/Update RouteQuery destinations
        if updated_chains or not incremental:
            self.RouteQuery.__annotations__["destination"] = Literal[tuple(self._get_destination_chains())]
        # 8. Set/Update router
        router_updated = self._update_cfg(router_cfg, "router", self._build_router_chain)
        # 9. Rebuild app if anything changed
        if updated_chains or router_updated or llm_updated or not incremental:
            print(f"Rebuilding app -Updated chains: {updated_chains}, Router: {router_updated}, LLM: {llm_updated}")
            self._build_app()
        else:
            print("No changes detected, keeping existing app")
        # 10. Set the agent seed if present in the current config update
        if seed_agent:
            self.seed_agent = seed_agent
            #self.generate_response(request = None)

        if seed_user:
            self.generate_response(seed_user)

    def set_history(self, history):
        """Receive a list of tuples (role, content) to set the history of the agent. Any previous history will be overwritten.

        Args:
            history ([(str, str)]): List of tuples (role, content) to set the history of the agent.
        """
        self.messages = [{'role': role, 'content': content} 
                         for role, content in history
                         if role in ['user', 'assistant', 'system']]
    #endregion

    #region StateGraph nodes definition
    def router_query(self, state: State, config: RunnableConfig):
        """Invoke the router chain to determine the destination expert for the input query.
        
        Args:
            state (State): The current state of the graph containing the input query and history.
            config (RunnableConfig): Runtime configuration for the router chain invocation.
        
        Returns:
            dict: A dictionary with the destination expert for the query.
        """
        destination = self.router_chain.invoke({"input": state["query"],
                                                "history": state["history"]},
                                                config)
        return {"destination": destination}
    
    def chat_query(self, state: State, config: RunnableConfig):
        """Invoke the chat chain to generate a response based on the input query and history.

        Args:
            state (State): The current state of the graph containing the input query and history.
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
        print("Routing to destination:", destination)
        # Track call for the destination chain
        self.stats[destination]["calls_count"] += 1
        # Use registry to check if chain exists
        if destination in self._chain_registry['chains']:
            return f"{destination}_query"
        else:
            available = list(self._get_destination_chains())
            raise ValueError(f"Chain '{destination}' not available. Available: {available}")
    #endregion
    
    #region Chain objects definition
    def _build_app(self):
        """Build the multi-prompt chain with only available chains."""
        print("Building the multi-prompt chain...", end=' ', flush=True)
        graph = StateGraph(self.State)
        # Always add router
        graph.add_node("router_query", self.router_query)
        graph.add_edge(START, "router_query")
        # Use registry to get available destination chains
        available_chains = self._get_destination_chains()
        # Build nodes for each destination chain
        for chain_name in available_chains:
            # Get chain config and type
            chain_cfg = getattr(self, f"{chain_name}_cfg", {})
            chain_type = chain_cfg.get("type")
            # Get the appropriate query method
            query_method = getattr(self, f"{chain_type}_query", None)
            if not query_method:
                print(f"Unknown chain type '{chain_type}' for '{chain_name}', skipping")
                continue
            # Add the chain node and edge
            graph.add_node(f"{chain_name}_query", query_method)
            graph.add_edge(f"{chain_name}_query", END)
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
        self._register_chain('llm')

    def _build_router_chain(self):
        """Build the router chain for the agent."""
        print("Building router chain...", end=' ', flush=True)
        self.router_chain = (
            self._format_chat_prompt(
                self._format_router_prompt(
                    self.router_cfg["prompt"], 
                    self.router_cfg["experts"]
                )
            ) 
            | self.llm.with_structured_output(self.RouteQuery)
        )
        self._register_chain('router')
        print("Done.")
    
    def _build_rag_chain(self):
        """Build the RAG chain for the agent."""
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
        else:  # Otherwise, create a new vector store (and save it)
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
        self._register_chain(current_chain)
        print("Done.")

    def _build_chat_chain(self):
        """Build a chat chain for the agent."""
        current_chain = getattr(self, '_current_chain_name', 'fallback')
        print(f"Building {current_chain} (chat) chain...", end=' ', flush=True)
        # Use the specific chain's config
        chain_cfg = getattr(self, f"{current_chain}_cfg")
        # Update config dictionary if changed
        setattr(self, f"{current_chain}_cfg", chain_cfg)
        # Build the chat chain
        chat_chain = (
            self._format_chat_prompt(chain_cfg["prompt"]) 
            | self.llm 
            | StrOutputParser()
        )
        # Store the chain with the specific name
        setattr(self, f"{current_chain}_chain", chat_chain)
        self._register_chain(current_chain)
        print("Done.")
    
    def _build_tool_chain(self, tool_name):
        """Build a tool chain for a tool identified by name in config."""
        print(f"Building {tool_name} (tool) chain...", end=' ', flush=True)
        # Get the tool configuration
        tool_cfg = getattr(self, f"{tool_name}_cfg", {})
        # Check if tools are available
        if not TOOLS_AVAILABLE or not tools_utils:
            print("Failed - tools module not available")
            return
        # Get tool class from the tool name
        tool_class = tools_utils.get_tool_class(tool_name)
        # If the tool exists, build the chain
        if tool_class:
            # Build the tool chain using the tool class
            tool_chain = tool_class.build_chain(tool_cfg)
            # Store the chain as class attribute
            setattr(self, f"{tool_name}_chain", tool_chain)
            self._register_chain(tool_name)
            print("Done.")
        else:
            print(f"Tool class not found for {tool_name}")
    #endregion
    
    #region Helper methods for configuration and chain building
    def _clear_config_and_chain(self, name):
        """Clear both config and chain for a given name."""
        # Remove config
        config_attr = f"{name}_cfg"
        if hasattr(self, config_attr):
            delattr(self, config_attr)
            print(f"  Cleared {config_attr}")
        # Remove chain object
        chain_attr = f"{name}_chain"
        if hasattr(self, chain_attr):
            delattr(self, chain_attr)
            print(f"  Cleared {chain_attr}")
        # Unregister from registry
        self._unregister_config(name)
        self._unregister_chain(name)

    def _update_cfg(self, new_cfg, cfg_key, builder_method):
        """Helper method to check, merge, and rebuild config sections.
        
        Args:
            new_cfg (dict): The new config dictionary to merge.
            cfg_key (str): The config key (e.g., "model", "router", "rag").
            builder_method (callable): The method to call if config changed (e.g., self._build_llm).
        
        Returns:
            bool: True if the config was updated and rebuilt. False otherwise.
        """
        cfg_var_name = cfg_key + "_cfg"
        # Get existing config
        existing_cfg = getattr(self, cfg_var_name, {})
        # Merge configs
        merged_cfg = self._merge_dicts_recursive(existing_cfg, new_cfg)
        # Check if config actually changed
        if merged_cfg != existing_cfg:
            print(f"  Updating {cfg_key} configuration...")
            # Update the config variable and register it
            setattr(self, cfg_var_name, merged_cfg)
            self._register_config(cfg_key)
            # Set the current chain name for the builder to use
            self._current_chain_name = cfg_key
            try:
                builder_method()
                print(f"{cfg_key} updated successfully")
                return True
            except Exception as e:
                print(f"Failed to update {cfg_key}: {repr(e)}")
                return False
            finally:
                # Clean up
                if hasattr(self, '_current_chain_name'):
                    delattr(self, '_current_chain_name')
        else:
            print(f"  No changes in {cfg_key} configuration")
            return False

    def _remove_deleted_chains(self, new_chains_dict):
        """Remove chains that exist but are not in the new configuration."""
        # Get the set of chains that are not in the new configuration
        new_chains = set(new_chains_dict.keys())
        chains_to_remove = self._chain_registry['destinations'] - new_chains
        # Remove them
        removed_chains = []
        for chain_name in chains_to_remove:
            print(f"  Removing deleted chain: {chain_name}")
            self._clear_config_and_chain(chain_name)
            removed_chains.append(chain_name)
        return removed_chains
    
    @staticmethod
    def _merge_dicts_recursive(old, new):
        """Recursively merge two dictionaries.
        
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
    #endregion

    #region Static methods for string/prompt formatting
    @staticmethod
    def _format_answer(text: str) -> str:
        """Remove <think>...</think> blocks, emojis, and trailing newlines."""
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
        return text.strip()

    @staticmethod
    def _format_chat_prompt(prompt):
        """Format the chat prompt for the agent."""
        return ChatPromptTemplate.from_messages(
            [
                ("system", prompt),
                ("human", "{input}"),
            ]
        )
    
    @staticmethod
    def _format_router_prompt(prompt, experts):
        """Format the router prompt with the experts."""
        experts_str = "\n".join([f"- {name}: {desc}" 
                               for name, desc in experts.items()])
        # TODO. Re-adding {history} to the prompt seems to work, but its not really elegant.
        return prompt.format(experts=experts_str, history="{history}")

    @staticmethod
    def _resolve_parametric_entry(entry, skip_fields=None, list_join=", "):
        """
        Resolve a parametric entry in the config.
        If entry is a dict with 'template' and 'params', format it substituting each {field} with the `field` value in params. Otherwise, return as is.

        Args:
            entry (dict | list |str): The entry to resolve.
            skip_fields (list, optional): List of fields to skip inside formatting.
            list_join (str, optional): String to join list values.

        Returns:
            The resolved entry value.
        """
        if skip_fields is None:
            skip_fields = []
        # Handle list of entries
        if isinstance(entry, list):
            entry_resolved = []
            for item in entry: # process items one-by-one
                entry_resolved.append(
                    LangchainChatter._resolve_parametric_entry(
                        item, skip_fields, list_join)
                )
        # Handle parametric entry
        elif isinstance(entry, dict) and "template" in entry and "params" in entry:
            # Use the template as the value to format
            template, params = entry["template"], entry["params"]
            entry_resolved = template
            # Find all {field} in the template and replace them with params values
            for field in re.findall("(?<={)(.*?)(?=})", template):
                if field in skip_fields:
                    continue
                replacement = params.get(field, "")
                if isinstance(replacement, list):
                    replacement = list_join.join(str(v) for v in replacement)
                entry_resolved = entry_resolved.replace(f"{{{field}}}", str(replacement))
        # Handle other cases (str, int, float, etc.)
        else:
            entry_resolved = entry # return as is
        return entry_resolved
        # endregion