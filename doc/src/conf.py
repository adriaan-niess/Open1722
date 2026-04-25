import os
import subprocess

project = 'Open1722'
author = 'Adriaan Niess'
copyright = '2026, COVESA'
version = '0.9'

extensions = [
    'sphinx_rtd_theme',
    'myst_parser',
    'breathe',
]

html_theme = "sphinx_rtd_theme"

source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}

# Breathe configuration
breathe_projects = {
    'Open1722': '_doxygen/xml'
}
breathe_default_project = 'Open1722'

# Generate Doxygen XML before building
def setup(app):
    subprocess.call(['doxygen', os.path.join(os.path.dirname(__file__), 'Doxyfile')])
    app.connect('config-inited', lambda app, config: None)
