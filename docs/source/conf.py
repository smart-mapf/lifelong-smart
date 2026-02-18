from __future__ import annotations
import os
import subprocess
import sys
from pathlib import Path
from datetime import date
from sphinx.util.fileutil import copy_asset

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

github_link = "https://github.com/smart-mapf/lifelong-smart"
DOCS_DEV = os.environ.get("DOCS_DEV", "0") == "1"

# `tags` will be defined by Sphinx
if DOCS_DEV:
    tags.add("devmode")  # type: ignore[name-defined]

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
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
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
    # Run Doxygen at start of build (if not in dev mode)
    if not DOCS_DEV:
        app.connect("builder-inited", lambda app: run_doxygen())

    # Copy README assets (logo, video) to output
    def _copy_readme_assets(app):
        src = os.path.join(app.srcdir, "readme_assets")
        dst = os.path.join(app.outdir, "readme_assets")
        if os.path.isdir(src):
            copy_asset(src, dst)

    app.connect("builder-inited", _copy_readme_assets)

    # Add custom CSS
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
# Override Exhale's default specificationsForKind to exclude undocumented
# members
def specificationsForKind(kind: str):
    # For classes/structs, DO NOT include undocumented members.
    if kind in ("class", "struct"):
        return [
            ":members:",
            ":protected-members:",
            # add ":private-members:" if you want, but still documented-only
            # ":private-members:",
        ]

    # For everything else, use Breathe defaults
    return []


# Exhale requires a "picklable" mapping wrapper
from exhale import utils

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
        "customSpecificationsMapping": utils.makeCustomSpecificationsMapping(specificationsForKind),
    }
else:
    exhale_args = {}

# -------------------------------------------------
# HTML
# -------------------------------------------------
html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_logo = "readme_assets/lsmart-logo-black-only.png"
root_doc = "index"

html_theme_options = {
    # Top bar links like pyribs (Documentation / Paper / GitHub)
    "navbar_center": ["navbar-nav"],  # this renders the links
    "external_links": [
        {
            "name": "Paper",
            "url": "http://arxiv.org/abs/2602.15721"
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
    "collapse_navigation":
    False,

    # Footer customization
    "footer_start": ["copyright"],
    "footer_end": ["sphinx-version"]
}

html_sidebars = {
    # Hide left sidebar on index page
    "index": [],
}

# For generating py docs
autosummary_generate = True

# Optional but useful:
# show type hints in text instead of signature
autodoc_typehints = "description"
napoleon_google_docstring = True
napoleon_numpy_docstring = True
autodoc_mock_imports = ["fire", "numpy", "lifelong_mapf_argos"]