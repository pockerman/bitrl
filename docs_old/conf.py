# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

from textwrap import dedent

# -- Project information -----------------------------------------------------

project = 'rlenvscpp'
copyright = '2025, Alexandros Giavaras'
author = 'Alexandros Giavaras'

# The full version, including alpha/beta/rc tags
release = 'v1.16.1'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [

    # there may be others here already, e.g. 'sphinx.ext.mathjax'
    'breathe',
    'exhale'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


breathe_default_project = "rlenvscpp"
breathe_projects = {
    "rlenvscpp": "./_doxygen/xml"
}


# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "doxygenStripFromPath":  "..",
    # Heavily encouraged optional argument (see docs)
    "rootFileTitle":         "Library API",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": True,
    #"exhaleDoxygenStdin":    "INPUT = ../src"
	
	"exhaleDoxygenStdin": dedent('''
        INPUT       = ../src
		ENABLE_PREPROCESSING = YES
		MACRO_EXPANSION = YES
		RECURSIVE = YES
		SKIP_FUNCTION_MACROS = NO
		EXCLUDE       =  ../src/rlenvs/extern/nlohmann/json/json.hpp
		EXCLUDE       += ../src/rlenvs/extern/nlohmann/json/json_fwd.hpp
		EXCLUDE       += ../src/rlenvs/extern/HTTPRequest.hpp
        # For this code-base, the following helps Doxygen get past a macro
        # that it has trouble with.  It is only meaningful for this code,
        # not for yours.
        PREDEFINED += NAMESPACE_BEGIN(arbitrary)="namespace arbitrary {"
        PREDEFINED += NAMESPACE_END(arbitrary)="}"
    '''),
}



# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'default'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
