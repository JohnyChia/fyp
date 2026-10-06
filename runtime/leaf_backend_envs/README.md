# Isolated leaf-backend environments

These environments are prerequisites for a later persistent inference process;
they are not ROS environments and must not contain or replace the active ROS2
Python installation.

`create_environment.sh` consumes an interpreter, expected Python version,
destination, and requirements lock as arguments. It refuses version mismatch
and refuses to overwrite an existing environment. An optional source checkout
and expected commit pair supports frameworks installed from frozen source.

Example, after the exact interpreter/source prerequisites are available:

```bash
runtime/leaf_backend_envs/create_environment.sh \
  /path/to/python 3.11.9 /path/to/environment /path/to/requirements-lock

runtime/leaf_backend_envs/create_environment.sh \
  /path/to/python 3.11.9 /path/to/environment /path/to/requirements-lock \
  /path/to/framework-source expected-git-commit
```

Backend-specific destinations, locks, source locations, and commits belong in
deployment configuration/manifests rather than this script.
