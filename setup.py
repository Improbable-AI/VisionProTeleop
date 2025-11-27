from setuptools import setup, find_packages

setup(
    name='avp_stream',
    version='2.4',
    description='This python package streams diverse tracking data available from AVP to any devices that can communicate with gRPC.',
    author='Younghyo Park',
    author_email='younghyo@mit.edu',
    packages=find_packages(),
    install_requires=[
        'numpy', 'grpcio', 'grpcio-tools', 'matplotlib', 'opencv-python', 
        'aiortc', 'av', 'requests', 'pyyaml', 'mujoco', 'tqdm', 'scipy', 
    ],
    extras_require={
    },
    entry_points={
        'console_scripts': [
            'setup-avp-wired=avp_stream.bridge_avp:main',
        ],
    },
)