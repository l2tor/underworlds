Underworlds: Geometric & Temporal Representation for Robots
===========================================================

[![Build
Status](https://travis-ci.org/severin-lemaignan/underworlds.svg?branch=master)](https://travis-ci.org/severin-lemaignan/underworlds)
[![Documentation Status](https://readthedocs.org/projects/underworlds/badge/?version=latest)](http://underworlds.readthedocs.org)

Description
-----------

Underworlds is a distributed and lightweight framework that aims at sharing
between clients parallel models of the physical world surrounding a robot.

The clients can be geometric reasoners (that compute topological relations
between objects), motion planner, event monitors, viewers... any software that
need to access a geometric (based on 3D meshes of objects) and/or temporal
(based on events) view of the world.

One of the main specific feature of Underworlds is the ability to store many
parallel worlds: past models of the environment, future models, models with
some objects filtered out, models that are physically consistent, etc.

This package provides the library, and a small set of core clients that are
useful for inspection and debugging.

Installation
------------

Ensure that your underworlds folder is in the same directiory as the interaction manager and DataModel from the L2TOR repository.
Please refer to the [installation documentation](http://underworlds.readthedocs.io/en/latest/installation.html?highlight=installation).

Note that the L2TOR client uses pyassimp to load objects, so the optional section on installing assimp and pyassimp is required for this project. We do provide a shortcut for skipping the installation of assimp below.

Pyassimp
--------
To skip the installation of assimp and to get pyassimp to work (see also the installation documentation of Underworlds above), after installing pyassimp via:

```
>python -m pip install pyassimp
```

...find the directory where it is installed, for example C:\Python27\Lib\site-packages\pyassimp\
Then, place the following file in this directory: https://surfdrive.surf.nl/files/index.php/s/R1BeSVHGyQSqZLZ

Finally, ensure that you have installed the Visual C++ Redistributable for Visual Studio 2015: https://www.microsoft.com/en-us/download/details.aspx?id=48145
After clicking on "Download", choose the file vc_redist.x86.exe

Running the unit-tests
----------------------

Underworlds provides a few unit-tests. Run them with:

```
> cd testing
> ./run_test.py
```

Documentation
-------------

[Head to readthedocs](http://underworlds.readthedocs.org). Sparse for now.

Running for L2TOR
------------------------

You will need to start by running the undeworlds daemon 
from <Underworlds Dir>/bin:

```
> python underworlded foreground
```

Underworlds should then communicate directly with the interaction manager

You can view the scenes that have been loaded into underworlds by running:

```
> python uwds-view l2tor
```

Please note that the viewer is for debugging purposes and should not be used during normal running of the system.


See also
--------

- `underworlds` is one of the child of LAAS' `SPARK`
  [(publication)](https://academia.skadge.org/publis/lemaignan2016artificial.pdf)
- The situation assessment framework [`toaster`](https://github.com/laas/toaster) is another child of SPARK, developed at LAAS
- M.  Naef,  E. Lamboray,  O. Staadt,  and M.  Gross, **The  blue-c  distributed scene graph**
- Bustos, Pablo, et al. **A Unified Internal Representation of the Outer World
  for Social Robotics.** Robot 2015: Second Iberian Robotics Conference. Springer
  International Publishing, 2016.
