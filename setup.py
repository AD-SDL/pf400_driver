import os
from setuptools import setup, find_packages


install_requires = []
with open('requirements.txt') as reqs:
    for line in reqs.readlines():
        req = line.strip()
        if not req or req.startswith('#'):
            continue
        install_requires.append(req)


package_name = 'pf400_driver'

setup(
    name = package_name,
    version='0.0.1',
    packages = find_packages(),
    install_requires=install_requires,
    zip_safe=True,
    python_requires=">=3.8",
    maintainer='Doga Ozgulbas and Alan Wang',
    maintainer_email='dozgulbas@anl.gov',
    description='Driver for the PF400 robot arm including ROS Nodes',
    url='https://github.com/AD-SDL/PF400_driver.git', 
    license='MIT License',
     classifiers=[
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: POSIX',
        'Operating System :: MacOS :: MacOS X',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
)
