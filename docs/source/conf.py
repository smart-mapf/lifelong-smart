from __future__ import annotations
import os
import subprocess
from datetime import date

# -------------------------------------------------
# Project info
# -------------------------------------------------
project = "LSMART"
author = "Yulun Zhang"
copyright = f"{date.today().year}, {author}"
version = release = "0.1"

# -------------------------------------------------
# Extensions
# -------------------------------------------------
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "breathe",
    # "exhale",
]

# Enable Exhale only when generating API stubs
if os.environ.get("EXHALE_PROJECT", "").strip():
    extensions.append("exhale")

templates_path = ["_templates"]
exclude_patterns = []

# -------------------------------------------------
# Paths
# -------------------------------------------------
HERE = os.path.abspath(os.path.dirname(__file__))  # docs/source
DOCS = os.path.abspath(os.path.join(HERE, ".."))  # docs
REPO = os.path.abspath(os.path.join(DOCS, ".."))  # repo root

# -------------------------------------------------
# Doxygen
# -------------------------------------------------
DOXYFILES = {
    "server": os.path.join(DOCS, "Doxyfile.server"),
    "client": os.path.join(DOCS, "Doxyfile.client"),
}


def run_doxygen():
    for name, doxyfile in DOXYFILES.items():
        subprocess.check_call(["doxygen", doxyfile], cwd=DOCS)


def setup(app):
    app.add_css_file("custom.css")
    app.connect("builder-inited", lambda app: run_doxygen())


# -------------------------------------------------
# Breathe (register both XML trees)
# -------------------------------------------------
breathe_projects = {
    "server": os.path.join(DOCS, "doxygen", "server", "xml"),
    "client": os.path.join(DOCS, "doxygen", "client", "xml"),
}

breathe_default_project = "server"

# -------------------------------------------------
# Exhale (ONE project per run)
# -------------------------------------------------
EXHALE_PROJECT = os.environ.get("EXHALE_PROJECT", "").strip().lower()

if EXHALE_PROJECT:
    exhale_args = {
        "containmentFolder": f"./api_{EXHALE_PROJECT}",
        "rootFileName": "library_root.rst",
        "rootFileTitle": f"{EXHALE_PROJECT.capitalize()} API",
        "doxygenStripFromPath": REPO,  # whatever you set REPO root to
        "createTreeView": True,
        "generateBreatheFileDirectives": True,
        "exhaleExecutesDoxygen": False,
    }
else:
    exhale_args = {}

# -------------------------------------------------
# HTML
# -------------------------------------------------
html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_logo = "_static/lsmart-logo-black.png"

html_theme_options = {
    # Top bar links like pyribs (Paper / Documentation / GitHub)
    "external_links": [
        {
            "name": "Paper",
            "url": ""
        },
        {
            "name": "Documentation",
            "url": ""
        },
        {
            "name": "GitHub",
            "url": "https://github.com/lunjohnzhang/lifelong_mapf_argos"
        },
    ],
    # Right-side icons (optional)
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/lunjohnzhang/lifelong_mapf_argos",
            "icon": "fa-brands fa-github"
        },
    ],
    "navbar_end": ["navbar-icon-links"],
    "header_links_before_dropdown":
    6,
}
