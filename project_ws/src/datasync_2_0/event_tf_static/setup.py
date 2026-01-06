from setuptools import setup

package_name = "event_tf_static"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/static_tf.launch.py"]),
        ("share/" + package_name + "/config", ["config/static_tf.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nochi",
    maintainer_email="nochi@todo.todo",
    description="Static TF publisher for event camera to base_link.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "event_tf_static_node = event_tf_static.static_tf_node:main",
        ],
    },
)
