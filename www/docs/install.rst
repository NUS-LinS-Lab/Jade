Installation
==========================================

On Linux or Mac OS X, running Python 3.5, 3.6, 3.7, or 3.8 and with :code:`pip` installed, you can run::

  pip install nimblephysics

We do not currently support Windows. Windows users are recommended to use the `Windows Subsystem for Linux <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_.

Common Errors (and how to fix them)
#############################################

If your version of :code:`pip` is older than :code:`19.3`, you will run into **ERROR: No matching distribution found for nimblephysics**. There's an easy fix, which is to upgrade :code:`pip` and try again::

  pip install -U pip
  pip install nimblephysics