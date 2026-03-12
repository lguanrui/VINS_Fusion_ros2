from __future__ import annotations

project = "VINS-Fusion ROS2"
author = "HKUST Aerial Robotics Group, ROS2 port maintained in lab"
copyright = "2026"
release = "ROS2 Humble"

extensions = [
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.githubpages",
]

templates_path = ["_templates"]
exclude_patterns = []
autosectionlabel_prefix_document = True

html_theme = "furo"
html_title = "VINS-Fusion ROS2 Documentation"
html_static_path = ["_static"]
html_css_files = ["custom.css"]

html_theme_options = {
    "navigation_with_keys": True,
    "sidebar_hide_name": False,
    "top_of_page_buttons": ["view", "edit"],
    "source_repository": "https://github.com/lguanrui/VINS_Fusion_ros2/",
    "source_branch": "main",
    "source_directory": "docs/source/",
    "light_css_variables": {
        "color-brand-primary": "#0d6b63",
        "color-brand-content": "#0f7c72",
        "color-admonition-background": "#f7fbfa",
        "color-api-keyword": "#9c4221",
    },
    "dark_css_variables": {
        "color-brand-primary": "#5ec2b7",
        "color-brand-content": "#76d1c7",
    },
}
