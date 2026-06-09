from pathlib import Path
from setuptools import setup, find_packages

setup(
    name="robot_collision_detection",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "matplotlib"
    ],
    author="10-OASIS-01",
    author_email="",
    description="A Python package for robot collision detection using geometric primitives",
    long_description=Path(__file__).parent.joinpath("README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    url="https://github.com/10-OASIS-01/robot-collision-detection",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
    python_requires=">=3.8",
)