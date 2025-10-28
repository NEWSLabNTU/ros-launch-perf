"""Analysis and visualization tools for play_launch execution logs."""

from .plot_resource_usage import main as plot_main

__version__ = "0.1.0"

__all__ = ["plot_main"]


def main():
    """Entry point for plot_resource_usage command."""
    plot_main()
