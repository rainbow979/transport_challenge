from setuptools import setup, find_packages


setup(
    name='transport_challenge',
    version="0.1.2",
    description='Transport Challenge API. Extends the Magnebot API and the TDW API.',
    long_description='Transport Challenge API. Extends the Magnebot API and the TDW API.',
    url='https://github.com/alters-mit/transport_challenge',
    author='Seth Alter',
    license='MIT',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: Software Development',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8'
    ],
    keywords='unity simulation tdw magnebot',
    packages=find_packages(),
    install_requires=['magnebot', 'numpy', 'tdw'],
)
