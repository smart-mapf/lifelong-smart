.. only:: not devmode

.. include:: ../../README.md
    :parser: myst_parser.sphinx_

.. toctree::
    :maxdepth: 1
    :caption: Getting Started
    :hidden:

    Overview <overview>
    Installation <install>


.. toctree::
    :maxdepth: 1
    :caption: API References
    :hidden:

    Planner-EM Communication <api_server/library_root>
    EM-Executor Communication <api_client/library_root>

.. only:: devmode

   API docs are disabled in dev mode. Build without DOCS_DEV=1 to see them.
