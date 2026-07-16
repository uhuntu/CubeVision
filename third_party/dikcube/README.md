# DikCube

This directory contains the 3x3x3 two-phase cube solver written by
Dik T. Winter and originally released in 1993. It is distributed under the
MIT license in `license.txt`.

CubeVision vendors the solver sources from Debian's `rubiks` source package
version 20070912. Only the files needed by the `CubeSolver` executable are
included. `globals.h` was updated to include the standard library headers
required by current C compilers; the solving algorithm is unchanged.

Upstream source archive:
https://deb.debian.org/debian/pool/main/r/rubiks/rubiks_20070912.orig.tar.bz2
