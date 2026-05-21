# TalosHybridMPC

Talos-specific IBRIDO integration package.

This package mirrors the layout used by `CentauroHybridMPC` and contains the
Talos RHC cluster client, Horizon controller wrapper, xacro argument helpers,
and initial controller/joint-impedance configuration files.

The current implementation is an integration scaffold for a one-environment,
open-loop debug setup. The Talos Horizon problem still needs proper tuning and
validation before it should be treated as a production controller.
