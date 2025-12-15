"""
WIA Eye Gaze Interoperability Protocol SDK

Standard interface for eye tracking devices and gaze-aware applications.

弘益人間 - 널리 인간을 이롭게
"""

from setuptools import setup, find_packages

setup(
    name="wia-eye-gaze",
    version="1.0.0a1",
    description="WIA Eye Gaze Interoperability Protocol SDK",
    long_description=open("README.md").read() if __import__("os").path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    author="SmileStory Inc. / WIA",
    author_email="contact@wia.family",
    url="https://github.com/anthropics/wia-standards",
    license="MIT",
    packages=find_packages(),
    python_requires=">=3.9",
    install_requires=[
        "typing_extensions>=4.0.0",
    ],
    extras_require={
        "tobii": ["tobii-research>=1.10.0"],
        "pupil": ["pyzmq>=25.0.0", "msgpack>=1.0.0"],
        "dev": [
            "pytest>=7.0.0",
            "pytest-asyncio>=0.21.0",
            "black>=23.0.0",
            "mypy>=1.0.0",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering :: Human Machine Interfaces",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    keywords="eye-tracking gaze accessibility aac assistive-technology wia",
)
