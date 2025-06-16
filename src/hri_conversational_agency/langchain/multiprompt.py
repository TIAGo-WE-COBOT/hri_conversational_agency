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

DEFAULT_ROUTER_PROMPT = """You are an assistant who has to choose which expert to route the question to. Experts are:
{}
Based on the question, choose the expert that is most relevant to answer it. Respond with the name of the expert only.
"""
DEFAULT_EXPERTS = {
                   "rag": "answers questions about history, historical events, periods, figuresm and culture.",
                   "fallback": "answers on general questions, when no expert is found.",
                   }
DEFAULT_CONTEXT = "This expert only answers in capital letters in the same language of the question."
DEFAULT_RAG_PROMPT = "You are a helpful assistant. Here is the conversation so far: \n{history}\n. Answer all questions to the best of your ability, using the provided context. Context: {context} /no_think"
DEFAULT_FALLBACK_PROMPT = "You are a helpful assistant. Answer all questions to the best of your ability Here is the conversation so far: \n{history}\n /no_think"

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

    def __init__(self, model='qwen3:4b', model_kwargs=None, router_prompt=DEFAULT_ROUTER_PROMPT, router_experts_dict=DEFAULT_EXPERTS, rag_prompt=DEFAULT_RAG_PROMPT, context=DEFAULT_CONTEXT, fallback_prompt=DEFAULT_FALLBACK_PROMPT, **kwargs):
        """Initialize the LangchainChatter object.

        Args:
            model (str, optional): The name of the model to use for the Ollama agent. Defaults to 'llama3.2:3b'.
            The model should be available in the Ollama server. You can check the available models with `ollama list` and retrieve the model with `ollama pull <model_name>` if needed.
        """
        # Initialize the logger to store the dialogue history
        self.log = Logger(agent_name='langchain - ' + model)
        self.log.logfile_open()
        # Initialize the LLMs
        self.llm = ChatOllama(model=model,
                              model_kwargs=model_kwargs
                              )
        self.messages = [] # chat history, to be used for model "memory"
        # Set prompt and available "experts" for the routing chain. The experts are defined as a dictionary where the keys are the expert names and the values are their descriptions.
        self.router_prompt = router_prompt
        self.router_experts_dict = router_experts_dict
        self._build_router_chain()
        # Set prompt and context for the Retrieval-Augmented Generation (RAG) chain.
        self.rag_prompt = rag_prompt
        self.rag_context = context
        self._build_rag_chain(build_retriever=True)
        # Set the fallback prompt for an simple chat model that will be used when no relevant context is found.
        self.fallback_prompt = fallback_prompt
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
        print(state["history"])
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
        self.log.log_output("[{} {}]".format(
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
        raise NotImplementedError()
        self.messages = [{'role': role, 'content': content} 
                         for role, content in history
                         if role in ['user', 'assistant', 'system']]
        # TODO. Return the history as set (?).

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
    
    def _build_rag_chain(self, build_retriever=True, **rag_kwargs):
        """Build the RAG chain for the agent. The chain will be used to retrieve relevant context from the vector store and generate a response based on the context.
        
        Args:
            build_retriever (bool): Whether to build the retriever from the context. If False, it will use the existing retriever if available.
            rag_kwargs (dict): Additional keyword arguments to pass to the RAG chain.
        """
        print("Building RAG chain...", end=' ', flush=True)
        if build_retriever or not hasattr(self, 'retriever'):
            # Split the context into chunks
            text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=100)
            texts = text_splitter.split_text(self.rag_context)
            # Create embeddings for the context
            embeddings = HuggingFaceEmbeddings(
                model_name="sentence-transformers/all-mpnet-base-v2",
                model_kwargs={"device": "cuda"}
                )
            # Create a vector store from the context chunks
            vector_store = FAISS.from_texts(texts, embeddings)
            # TODO. Consider logging the context for later inspection of the dialogue
            # TODO. Allow saving and loading the vector store to/from disk.
            self.rag_retriever = vector_store.as_retriever(
                search_type="similarity",
                search_kwargs={"k": 3}
            )
        # Build the RAG chain using the retriever and the prompt
        self.rag_chain = (
            {
                "context": self.rag_retriever,
                "input": RunnablePassthrough(),
            }
            | self._format_chat_prompt(self.rag_prompt)
            | self.llm
            | StrOutputParser()
        )
        print("Done.")

    def _build_fallback_chain(self):
        """Build the fallback chain for the agent. The chain will be used to generate a response when no relevant context is found."""
        print("Initializing fallback chain...", end=' ', flush=True)
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
        return prompt.format(experts_str) + experts_str

    @staticmethod
    def _format_answer(text: str) -> str:
        """Remove <think>...</think> blocks and trailing newlines.
        
        Args:
            text (str): The text to format.
        
        Returns:
            str: The formatted text with <think> blocks removed and trailing newlines.
        """
        # Remove all <think>...</think> blocks (including newlines inside)
        text = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL)
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