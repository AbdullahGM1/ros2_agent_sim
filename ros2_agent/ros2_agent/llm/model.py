#!/usr/bin/env python3
"""
LLM configuration for Search and Rescue multi-robot ROSA Agent.
This module handles LLM initialization with configurations optimized for SAR operations.
"""

import logging
from langchain_ollama import ChatOllama

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
    
    # Use the original parameters with slight modifications for SAR operations
    return ChatOllama(
        model=model_name,
        temperature=0.0,  # Deterministic responses for consistent operation
        max_retries=3,    # Increased retries for reliability in challenging environments
        num_ctx=8192,     # Maximum context for handling complex situations
    )