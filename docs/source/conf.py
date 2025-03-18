# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'COGAR example'
copyright = '2025, Alessandro Carfì'
author = 'Alessandro Carfì'
release = '1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_show_sourcelink = False

html_context = {
    "display_github": True,  # Enable GitHub link
    "github_user": "ACarfi",  # Your GitHub username
    "github_repo": "COGAR-Example",  # Repository name
    "github_version": "main",  # Branch name (e.g., 'main' or 'master')
    "conf_py_path": "/",  # Ensures the URL doesn't append file paths
}