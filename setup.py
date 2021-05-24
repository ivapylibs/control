from setuptools import setup
setup(name='Control',
      version='1.0',
      description='Classes implementing controllers, simulations, and control systems',
      author='Justin Smith, Patricio Vela, Varun Madabushi',
      package_dir={'control':'./'},
      packages=['control', 'controller', 'trajSynth', 'reachable'],

      )