#!/usr/bin/env python3
"""
LLM configuration for Search and Rescue multi-robot ROSA Agent.
This module handles LLM initialization with configurations optimized for SAR operations.
"""

from langchain_ollama import ChatOllama
import os
import logging

# Optional: For fallback to cloud models if available
# from langchain_openai import ChatOpenAI

logger = logging.getLogger(__name__)

def initialize_llm(model_name: str) -> ChatOllama:
    """
    Initialize and configure the LLM for the SAR robot agent.
    
    Optimized for search and rescue missions with reliability and
    deterministic responses as priorities.
    
    Args:
        model_name (str): Name of the LLM model to use (e.g., 'qwen3:8b', 'llama3:8b', etc.)
        
    Returns:
        ChatOllama: Configured LLM instance
    """
    # For SAR operations, we prioritize:
    # 1. Reliability - multiple retries
    # 2. Deterministic responses - zero temperature
    # 3. Longer context - maximum available context window
    # 4. Fast responses - timeout settings
    
    logger.info(f"Initializing LLM for SAR operations with model: {model_name}")
    
    return ChatOllama(
        model=model_name,
        temperature=0.0,  
        max_retries=5,    
        num_ctx=8192,     
        timeout=30,       
    )