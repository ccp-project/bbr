CCP Algorithm: BBR
==================

This repository provides a *simplified* implementation of Google's 
[BBR congestion control algorithm](https://ai.google/research/pubs/pub45646) using 
[CCP](https://github.com/ccp-project/portus).

To get started using this algorithm with CCP, please see our [guide](https://ccp-project.github.io/ccp-guide).

**NOTE**: So far, this implementation only covers the high-level concepts in BBR. It does not yet
handle all of the edge cases and thus will not perform exactly the same as the canonical implementation
in all scenarios. We are currently working on improving this implementation to match the canonical one.
