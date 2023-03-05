from setuptools import setup, find_packages


packages = find_packages(include="seer")
# Ensure that we don't pollute the global namespace.
for p in packages:
    assert p == 'seer' or p.startswith('seer.')

setup(name='seer',
      version='1.0.0',
      description='SEER Advanced Robotics Project',
      url='#',
      author='ddd26, mbg34, nyl25, sd974',
      author_email='author@cam.ac.uk',
      packages=packages,
      package_dir={'seer': 'seer'})
