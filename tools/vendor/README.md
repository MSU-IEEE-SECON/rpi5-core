# Vendored libraries (the Pi has no internet)

- `dt_apriltags/` — from the `dt-apriltags` 3.1.7 wheel (manylinux2014_aarch64), Duckietown's
  Python binding of the AprilTag 3 C library (BSD 2-clause, University of Michigan APRIL lab).
  Supports the `tagStandard41h12` family the SECON 2027 arena uses. Needs only numpy.
  Used by `tools/apriltag_test.py`. Replace with `pip install dt-apriltags` once the Pi is online.
