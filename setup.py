from setuptools import setup, find_packages

setup(
    name='avp_stream',
    version='0.1',
    description='This python package streams diverse tracking data available from AVP to any devices that can communicate with gRPC.',
    author='Younghyo Park',
    author_email='younghyo@mit.edu',
    packages=find_packages(),
    install_requires=[
        'numpy', 'grpcio', 'grpcio-tools'
    ],
    extras_require={
        # Optional dependencies
        # e.g., 'dev': ['check-manifest'],
        # e.g., 'test': ['coverage'],
    },
)