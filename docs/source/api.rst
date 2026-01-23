.. include:: ../../README.md
    :parser: myst_parser.sphinx_

.. only:: not devmode


   .. toctree::
      :maxdepth: 1
      :caption: API Reference
      :hidden:

      api_server/library_root
      api_client/library_root

.. only:: devmode

   API docs are disabled in dev mode. Build without DOCS_DEV=1 to see them.
