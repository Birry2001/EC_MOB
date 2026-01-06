from setuptools import setup

package_name = "event_depth_midas"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/depth_midas.launch.py"]),
        ("share/" + package_name + "/config", ["config/depth_midas.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nochi",
    maintainer_email="nochi@todo.todo",
    description="MiDaS depth estimation from APS images.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "event_depth_midas_node = event_depth_midas.depth_midas_node:main",
        ],
    },
)
