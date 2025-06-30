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
from langchain_core.runnables import RunnableConfig, RunnablePassthrough, RunnableLambda
from langgraph.graph import END, START, StateGraph

from hri_conversational_agency.base import BaseChatter
from hri_conversational_agency.logger import Logger

class LangchainChatter(BaseChatter):
    # Define schema for multi-prompt chain output
    class RouteQuery(TypedDict):
        """Route query to destination expert."""
        destination: Literal["rag", "fallback", "weather", "datetime"]
    # For LangGraph, we will define the state of the graph to hold the query,
    # destination, and final answer.
    class State(TypedDict):
        query: str
        destination: 'LangchainChatter.RouteQuery'
        answer: str
        history: list[dict] # list of messages in the format 
                            # {
                            #   'role': 'system'/'user'/'assistant', 
                            #   'content': 'message content'
                            # }

    def __init__(self, config):
        """Initialize the LangchainChatter object.

        Args:
            model (str, optional): The name of the model to use for the Ollama agent. Defaults to 'qwen3:4b'.
            The model should be available in the Ollama server. You can check the available models with `ollama list` and retrieve the model with `ollama pull <model_name>` if needed.
        """
        # Initialize the agent "memory"
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
        # Add the reponse to the messages to maintain the chat history
        # Update history
        self.messages += [
            {'role': 'user', 'content': request},
            {'role': 'assistant', 'content': response}
        ]
        # Log the output for later inspection of the dialogue
        self.log.log_output("[{}] {}".format(
            state["destination"]["destination"].lower(),
            response)
        )
        return response

    def set_config(self, yaml_path):
        """Receive filepath to a YAML file containing the configuration of the agent. The YAML file should contain the model name, the router prompt, the RAG context, and the fallback prompt."""
        with open(yaml_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        # 1. Set/Update LLM config
        self._update_cfg(cfg.pop("model", {}), "llm", self._build_llm)
        # 2. Set/Update router config
        router_cfg = cfg.pop("router", {})
        experts_dict = {}
        for chain, kwargs in cfg.items(): 
            # Parse remaining entries in the config to see if they are "experts" for the router chain.
            if "description" not in kwargs:
                continue
            experts_dict[chain] = kwargs["description"]
        router_cfg["experts"] = experts_dict
        self._update_cfg(router_cfg, "router", self._build_router_chain)
        # 3. Set/Update configs for LLM-based chains
        self._update_cfg(cfg.pop("rag", {}), "rag", self._build_rag_chain)
        self._update_cfg(cfg.pop("fallback", {}), "fallback", self._build_fallback_chain)
        # 4. Set/Update configs for "tool" chains (i.e. Python functions not running on an LLM)
        self._update_cfg(cfg.pop("weather", {}), "weather", self._build_tool_weather_chain)
        self._update_cfg(cfg.pop("datetime", {}), "datetime", self._build_tool_datetime_chain)

        # Build the multi-prompt chain that will
        # 1) Select the "expert" via the router_chain, and collect the answer
        # alongside the input query.
        # 2) Route the input query to the proper chain, based on the
        # selection.
        self._build_app()
        return True 
    
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
            destination = self.router_chain.invoke(state["query"], config)
            return {"destination": destination}
    
    def rag_query(self, state: State, config: RunnableConfig):
        return {"answer": self.rag_chain.invoke({"input": state["query"],
                                                 "history": state["history"]}, config)}

    def fallback_query(self, state: State, config: RunnableConfig):
        return {"answer": self.fallback_chain.invoke({"input": state["query"],
                                                 "history": state["history"]}, config)}
    
    def weather_query(self, state: State, config: RunnableConfig):
        return {"answer": self.weather_chain()}

    def datetime_query(self, state: State, config: RunnableConfig):
        return {"answer": self.datetime_chain()}

    def router_select_node(self, state: State) -> Literal["rag_query", "fallback_query", "weather_query", "datetime_query"]:
        destination = state["destination"]["destination"].lower()
        if destination in [key for key in self.router_cfg["experts"].keys()]:
            return destination + '_query'
        else:
            print("No expert found for destination: {}".format(destination))
            return "fallback_query"
    #endregion
    
    #region Chain objects definition
    def _build_app(self):
        """Build the multi-prompt chain for the agent. The chain will be used to route the input query to the appropriate expert, and invoke a response from it.
        """
        print("Building the multi-prompt chain...", end=' ', flush=True)
        graph = StateGraph(self.State)
        graph.add_node("router_query", self.router_query)
        graph.add_node("rag_query", self.rag_query)
        graph.add_node("fallback_query", self.fallback_query)

        # Add tool nodes if they exist
        if hasattr(self, 'weather_chain'):
            graph.add_node("weather_query", self.weather_query)
            graph.add_edge("weather_query", END)

        if hasattr(self, 'datetime_chain'):
            graph.add_node("datetime_query", self.datetime_query)
            graph.add_edge("datetime_query", END)

        graph.add_edge(START, "router_query")
        graph.add_conditional_edges("router_query", self.router_select_node)
        graph.add_edge("rag_query", END)
        graph.add_edge("fallback_query", END)
        self.app = graph.compile()
        print("Done.")

    def _build_router_chain(self):
        """Build the router chain for the agent. The chain will be used to route the input query to the appropriate expert based on the router prompt and the available experts.
        """
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
        print("Building RAG chain...", end=' ', flush=True)
        # Unpack kwargs for RAG chain elements
        text_splitter_kwargs = self.rag_cfg.get('text_splitter', {})
        embedding_model_kwargs = self.rag_cfg.get('embedding_model', {})
        faiss_kwargs = self.rag_cfg.get('faiss', {})
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
            text_splitter_kwargs = self.rag_cfg.get('text_splitter',{})
            text_splitter = RecursiveCharacterTextSplitter(
                **text_splitter_kwargs
            )
            texts = text_splitter.split_text(self.rag_cfg["context"]["content"])
            # Create a vector store from the context chunks
            vector_store = FAISS.from_texts(texts, embeddings)
            # Save the vector store to disk
            vector_store.save_local(index_path)
            # TODO. Consider logging the context for later inspection of the dialogue
        self.rag_retriever = vector_store.as_retriever(
            search_type="similarity",
            search_kwargs=faiss_kwargs.get('search', {}),
        )
        # Build the RAG chain using the retriever and the prompt
        self.rag_chain = (
            {
                "context": itemgetter("input") | self.rag_retriever,
                "input": itemgetter("input"),
                "history": itemgetter("history")          
            }
            | self._format_chat_prompt(self.rag_cfg["prompt"])
            | self.llm
            | StrOutputParser()
        )
        print("Done.")

    def _build_fallback_chain(self):
        """Build the fallback chain for the agent. The chain will be used to generate a response when no relevant context is found."""
        print("Building fallback chain...", end=' ', flush=True)
        self.fallback_chain = (
            self._format_chat_prompt(self.fallback_cfg["prompt"]) 
            | self.llm 
            | StrOutputParser()
        )
        print("Done.")
    
    def _build_llm(self):
        llm_cfg = self.llm_cfg.copy()
        self.llm = ChatOllama(model=llm_cfg.pop("model"),
                              **llm_cfg,
                            )
        
    def _build_tool_weather_chain(self): # TODO. Implement translation
        """Build the weather chain for the agent. The chain will be used to retrieve current weather information for a specified location."""
        print("Building weather chain...", end=' ', flush=True)
        from hri_conversational_agency.langchain.tools import get_weather_info
        self.weather_chain = lambda: get_weather_info(
            location = self.weather_cfg.get('location', None),
            lang = self.weather_cfg.get('lang', 'en')
        )
        print("Done.")

    def _build_tool_datetime_chain(self):   # TODO. Implement translation
        """Build the datetime chain for the agent. The chain will be used to retrieve current date, month, and season information."""
        print("Building datetime chain...", end=' ', flush=True)
        from hri_conversational_agency.langchain.tools import get_datetime_info
        self.datetime_chain = lambda: get_datetime_info(
            lang = self.datetime_cfg.get('lang', 'en')
        )
        print("Done.")
    #endregion
    
    #region Helper methods for setting agent configuration
    def _merge_dicts_recursive(self, old, new):
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
                merged[k] = self._merge_dicts_recursive(merged[k], v)
            else:
                merged[k] = v
        return merged
    
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

        merged_cfg = self._merge_dicts_recursive(
            getattr(self, cfg_var_name, {}),
            new_cfg
        )
        
        if not hasattr(self, cfg_var_name) or merged_cfg != getattr(self, cfg_var_name, {}):
            setattr(self, cfg_var_name, merged_cfg)
            builder_method()
            return True
        return False

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
        experts_str = "\n".join(["- {}: {}".format(name, desc) 
                                 for name, desc in experts.items()
                                 ])
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
    
    @staticmethod
    def _remove_emojis(text: str) -> str:
        """Remove emojis from the given text.

        Args:
            text (str): A string that may contain emojis.

        Returns:
            str: The input string with emojis removed.
        """

    #endregion