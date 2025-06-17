"""
A conversational agent that generates responses to user input based on Ollama models. The integration with Langchain allows for the implementation of advanced functionalities such as document retrieval, and multi-prompt chains.
"""

import os
import re
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
        destination: Literal["rag", "fallback"]
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

    def __init__(self, model='qwen3:4b', prompts_dict=None,
     router_experts_dict=None, rag_context=None, **kwargs):
        """Initialize the LangchainChatter object.

        Args:
            model (str, optional): The name of the model to use for the Ollama agent. Defaults to 'qwen3:4b'.
            The model should be available in the Ollama server. You can check the available models with `ollama list` and retrieve the model with `ollama pull <model_name>` if needed.
        """
        # Initialize the logger to store the dialogue history
        self.log = Logger(agent_name='langchain - ' + model)
        self.log.logfile_open()
        # Initialize the LLMs
        self.llm = ChatOllama(model=model,
                              model_kwargs=kwargs.pop('model_kwargs', {}),
                              )
        self.messages = [] # chat history, to be used for model "memory"
        # Set prompt and available "experts" for the routing chain. The experts are defined as a dictionary where the keys are the expert names and the values are their descriptions.
        self.router_prompt = prompts_dict["router_prompt"]
        self.router_experts_dict = router_experts_dict
        self._build_router_chain()
        # Set prompt and context for the Retrieval-Augmented Generation (RAG) chain.
        self.rag_prompt = prompts_dict["rag_prompt"]
        self.rag_context = rag_context
        self.rag_kwargs = kwargs.pop('rag_kwargs', {})
        self._build_rag_chain(build_retriever=True)
        # Set the fallback prompt for an simple chat model that will be used when no relevant context is found.
        self.fallback_prompt = prompts_dict["fallback_prompt"]
        self._build_fallback_chain()
        # Build the multi-prompt chain that will
        # 1) Select the "expert" via the router_chain, and collect the answer
        # alongside the input query.
        # 2) Route the input query to the proper chain, based on the
        # selection.
        self._build_app()
        print("Ready to chat!")
    
    # region StateGraph nodes definition
    def router_query(self, state: State, config: RunnableConfig):
            destination = self.route_chain.invoke(state["query"], config)
            return {"destination": destination}
    
    def rag_query(self, state: State, config: RunnableConfig):
        return {"answer": self.rag_chain.invoke({"input": state["query"],
                                                 "history": state["history"]}, config)}

    def fallback_query(self, state: State, config: RunnableConfig):
        return {"answer": self.fallback_chain.invoke({"input": state["query"],
                                                 "history": state["history"]}, config)}

    def router_select_node(self, state: State) -> Literal["rag_query", "fallback_query"]:
        destination = state["destination"]["destination"].lower()
        if destination in [key for key in self.router_experts_dict.keys()]:
            return destination + '_query'
        else:
            print("No expert found for destination: {}".format(destination))
            return "fallback_query"
    #endregion

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

    def set_context(self, context):
        """Receive a context string to set the context of the agent. The context will be used to inform the agent's responses."""
        self.rag_context = context
        self._build_rag_chain(build_retriever=True)
        self._build_app()
    
    def set_rag_prompt(self, prompt):
        """Set the RAG prompt for the agent. The prompt will be used to format the input query into the respective prompt, run it through a chat model, and cast the result to a string."""
        self.rag_prompt = prompt
        self._build_rag_chain(build_retriever=False)
        self._build_app()

    def set_history(self, history):
        """Receive a list of tuples (role, content) to set the history of the agent. Any previous history will be overwritten.

        Args:
            history ([(str, str)]): List of tuples (role, content) to set the history of the agent.
        """
        self.messages = [{'role': role, 'content': content} 
                         for role, content in history
                         if role in ['user', 'assistant', 'system']]

    def set_sys_prompt(self, prompt):
        """Receive a prompt to set the router prompt of the agent. Any previous system prompt will be deleted.
        
        Args:
            prompt (str): String to set as the system prompt of the router chain.
        """
        self.router_prompt = prompt
        self._build_router_chain()
        self._build_app()

    def set_fallback_prompt(self, prompt):
        """Set the fallback prompt for the agent. The prompt will be used when the agent cannot find a relevant context.
        
        Args:
            prompt (str): The system prompt to set for the fallback agent.
        """
        self.fallback_prompt = prompt
        self._build_fallback_chain()
        self._build_app()
    
    def on_shutdown(self):
        self.log.logfile_close()
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
        self.route_chain = (
            self._format_chat_prompt(
                self._format_router_prompt(
                    self.router_prompt, 
                    self.router_experts_dict
                    )
                ) 
            | self.llm.with_structured_output(self.RouteQuery)
        )
        print("Done.")
    
    def _build_rag_chain(self, build_retriever=True):
        """Build the RAG chain for the agent. The chain will be used to retrieve relevant context from the vector store and generate a response based on the context. 
        The retriever is built from the context if `build_retriever` is True, otherwise it will use the existing retriever if available. This enables not to rebuild the retriever when only the prompt changes, but the context remains the same.
        Saved embeddings can be loaded from disk if a `faiss_path` is provided in the `rag_kwargs`.
        
        Args:
            build_retriever (bool): Whether to build the retriever from the context. If False, it will use the existing retriever if available.
            rag_kwargs (dict): Additional keyword arguments to pass to the RAG chain.
        """
        print("Building RAG chain...", end=' ', flush=True)
        # Unpack kwargs for RAG chain elements
        text_splitter_kwargs = self.rag_kwargs.get(
            'text_splitter_kwargs', {})
        embedding_model_kwargs = self.rag_kwargs.get(
            'embeddings_model_kwargs', {})
        faiss_kwargs = self.rag_kwargs.get(
            'faiss_kwargs', {})
        # If the retriever has to be built or does not exist, build it
        if build_retriever or not hasattr(self, 'rag_retriever'):
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
            else:
                # Split the context into chunks
                text_splitter_kwargs = self.rag_kwargs.get(
                    'text_splitter_kwargs',
                    {}
                )
                text_splitter = RecursiveCharacterTextSplitter(
                    **text_splitter_kwargs
                )
                texts = text_splitter.split_text(self.rag_context)
                # Create a vector store from the context chunks
                vector_store = FAISS.from_texts(texts, embeddings)
                # Save the vector store to disk
                vector_store.save_local(index_path)
                # TODO. Consider logging the context for later inspection of the dialogue
            self.rag_retriever = vector_store.as_retriever(
                search_type="similarity",
                search_kwargs=self.rag_kwargs.get('retriever_kwargs', {}),
            )
        # Build the RAG chain using the retriever and the prompt
        self.rag_chain = (
            {
                "context": itemgetter("input") | self.rag_retriever,
                "input": itemgetter("input"),
                "history": itemgetter("history")          
            }
            | self._format_chat_prompt(self.rag_prompt)
            | self.llm
            | StrOutputParser()
        )
        print("Done.")

    def _build_fallback_chain(self):
        """Build the fallback chain for the agent. The chain will be used to generate a response when no relevant context is found."""
        print("Building fallback chain...", end=' ', flush=True)
        self.fallback_chain = (
            self._format_chat_prompt(self.fallback_prompt) 
            | self.llm 
            | StrOutputParser()
        )
        print("Done.")
    #endregion
    
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