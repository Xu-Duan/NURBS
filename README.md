# NURBS C++ Starter

This repository contains a minimal starter structure for a C++ NURBS library.

## Layout

- `include/nurbs/`: public API
- `src/`: implementation
- `examples/`: small usage samples
- `tests/`: lightweight verification

## Current modules

- `core`: shared types and knot-vector handling
- `basis`: B-spline basis evaluation utilities
- `geometry`: NURBS curve object built on top of basis functions

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Suggested next steps

1. Add first- and second-derivative evaluation.
2. Add NURBS surfaces and tensor-product basis evaluation.
3. Introduce stronger linear algebra types instead of raw coordinate vectors.
4. Add parameter-domain utilities, knot insertion, degree elevation, and refinement.
