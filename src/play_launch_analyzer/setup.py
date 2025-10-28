from setuptools import find_packages, setup

package_name = "play_launch_analyzer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "matplotlib>=3.0.0",
        "numpy>=1.16.0",
    ],
    zip_safe=True,
    maintainer="Jerry Lin",
    maintainer_email="jerry73204@gmail.com",
    description="Analysis and visualization tools for play_launch execution logs",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plot_play_launch = play_launch_analyzer:main",
        ],
    },
)
