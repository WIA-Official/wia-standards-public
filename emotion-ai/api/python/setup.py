"""
WIA Emotion AI SDK

pip install wia-emotion-ai
"""

from setuptools import setup, find_packages

setup(
    name="wia-emotion-ai",
    version="1.0.0",
    description="WIA Emotion AI SDK - Affective Computing Standards",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="WIA (World Interoperability Alliance)",
    author_email="dev@wia.live",
    url="https://github.com/WIA-Official/wia-standards",
    packages=find_packages(),
    install_requires=[
        "requests>=2.28.0",
        "websockets>=10.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-asyncio>=0.20",
        ]
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    keywords="emotion-ai affective-computing emotion-recognition wia",
)
