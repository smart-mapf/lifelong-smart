from __future__ import annotations
import os
import subprocess
from datetime import date

github_link = "https://github.com/lunjohnzhang/lifelong_mapf_argos"
DOCS_DEV = os.environ.get("DOCS_DEV", "0") == "1"

# `tags` will be defined by Sphinx
if DOCS_DEV:
    tags.add("devmode") # type: ignore[name-defined]

# -------------------------------------------------
# Project info
# -------------------------------------------------
project = "LSMART"
author = "Yulun Zhang"
arcs_lab = "ARCS Lab, Carnegie Mellon University"
copyright = f"2026, {arcs_lab}"
version = release = "0.1"

# -------------------------------------------------
# Extensions
# -------------------------------------------------
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "myst_parser",
    # "breathe",
    # "exhale",
]

if not DOCS_DEV:
    extensions += ["breathe"]  # and "exhale" only when generating trees

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
    if not DOCS_DEV:
        app.connect("builder-inited", lambda app: run_doxygen())
    app.add_css_file("custom.css")


# -------------------------------------------------
# Breathe (register both XML trees)
# -------------------------------------------------
if not DOCS_DEV:
    breathe_projects = {
        "server": os.path.join(DOCS, "doxygen", "server", "xml"),
        "client": os.path.join(DOCS, "doxygen", "client", "xml"),
    }
    breathe_default_project = "server"

# -------------------------------------------------
# Exhale (ONE project per run)
# -------------------------------------------------
EXHALE_PROJECT = os.environ.get("EXHALE_PROJECT", "").strip().lower()

if (not DOCS_DEV) and EXHALE_PROJECT in ("server", "client"):
    extensions.append("exhale")
    breathe_default_project = EXHALE_PROJECT
    exhale_args = {
        "containmentFolder": f"./api_{EXHALE_PROJECT}",
        "rootFileName": "library_root.rst",
        "rootFileTitle": f"{EXHALE_PROJECT.capitalize()} API",
        "doxygenStripFromPath": REPO,
        "createTreeView": True,
        "generateBreatheFileDirectives": True,
        "exhaleExecutesDoxygen": False,
        # "unabridgedOrphanKinds": ["function", "file", "namespace", "class", "struct"],
        # and importantly:
        "fullApiSubSectionTitle": "Full API",
        "contentsDirectives": True,
    }
else:
    exhale_args = {}

# -------------------------------------------------
# HTML
# -------------------------------------------------
html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_logo = "_static/lsmart-logo-black-only.png"

html_theme_options = {
    # Top bar links like pyribs (Paper / Documentation / GitHub)
    "external_links": [
        {
            "name": "Paper",
            "url": ""
        },
        {
            "name": "Documentation",
            "url": "api.html"
        },
        {
            "name": "GitHub",
            "url": github_link
        },
    ],
    # Right-side icons (optional)
    "icon_links": [
        {
            "name": "GitHub",
            "url": github_link,
            "icon": "fa-brands fa-github"
        },
    ],
    "navbar_end": ["navbar-icon-links"],
    "header_links_before_dropdown":
    6,
    "collapse_navigation": False,

    # Footer customization
    "footer_start": ["copyright"],
    "footer_end": ["sphinx-version"]
}