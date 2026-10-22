from glob import glob

from setuptools import setup

package_name = "vida_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/paths", glob("paths/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="odroid",
    maintainer_email="167372071+Nucleo-LMT@users.noreply.github.com",
    description="Navegacao autonoma do VIDA",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_tools = vida_navigation.path_tools:main",
            "path_publisher = vida_navigation.path_publisher:main",
            "path_recorder = vida_navigation.path_recorder:main",
        ],
    },
)
