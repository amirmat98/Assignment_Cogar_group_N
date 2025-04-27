# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

project = 'Assignment_Cogar_Group_N'
copyright = '2025, AmirMahdi Matin - Mahmoud M. A. Elderiny'
author = 'AmirMahdi Matin - Mahmoud M. A. Elderiny'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram'
]

templates_path = ['_templates']
exclude_patterns = []

autodoc_mock_imports = ["rospy", "std_msgs", "sensor_msgs", "assignments"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

highlight_language = 'python'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_show_sourcelink = False

html_context = {
    "display_github": True,  # Integrate GitHub
    "github_user": "amirmat98",  # Username
    "github_repo": "Assignment_Cogar_Group_N",  # Repo name
    "github_version": "main", 
    "conf_py_path": "/docs/source/",  # Path in the checkout to the docs root
}
