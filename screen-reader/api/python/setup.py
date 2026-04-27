"""
WIA Screen Reader Standard - Python SDK Setup
"""
from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="wia-screen-reader",
    version="1.0.0",
    author="WIA Standards",
    author_email="official@wia.codes",
    description="WIA Screen Reader Standard - Universal Accessibility SDK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/WIA-Official/wia-standards",
    project_urls={
        "Bug Tracker": "https://github.com/WIA-Official/wia-standards/issues",
        "Documentation": "https://wiastandards.com/screen-reader/",
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Accessibility",
    ],
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "pyttsx3>=2.90",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=4.0",
            "black>=23.0",
            "mypy>=1.0",
        ],
    },
    keywords=[
        "accessibility",
        "screen-reader",
        "wia",
        "wihp",
        "braille",
        "tts",
        "a11y",
        "nvda",
    ],
)
