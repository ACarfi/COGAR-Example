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
    "display_github": True,  # Enables the GitHub link
    "github_user": "ACarfi",  # Replace with your GitHub username
    "github_repo": "COGAR-Example",  # Replace with your repository name
    "github_version": "main",  # Branch name (e.g., 'main' or 'master')
    "conf_py_path": "/docs/source/",  # Path inside the repo where your docs are stored
}

extensions = [
    'sphinxcontrib.video',
    'sphinxcontrib.youtube',
]