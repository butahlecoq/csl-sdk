# ADR 0001: Define core numerical value contracts

Date: 2026-11-21
Status: Accepted

## Context

The SDK passes native matrix data, optimization bounds, and residual evidence through several backends. Callers need consistent representations and profile-specific criteria.

## Decision

- **Dense matrices.** `Matrix` owns a contiguous row-major buffer. Zero rows or columns are representable, with dimensions retained and an empty buffer. Checked entry points validate shape and storage size before mapping the data. This lets native adapters use the same representation independently of the matrix library behind each adapter.
- **Sparse matrices.** `SparseMatrix` stores compressed sparse rows. Row offsets delimit each row. The column indices within a row are strictly increasing, unique, and in range. The checked snapshot requires that layout so row lookup can use binary search.
- **Intervals.** An absent lower or upper endpoint leaves that side unbounded. Present endpoints are finite and ordered when both are supplied. This convention represents unbounded inputs without storing infinities in their numeric fields.
- **Scaled residuals.** A numerical profile that applies the `ResidualMeasure` acceptance rule uses binary64 and allows a violation up to `absolute_tolerance + relative_tolerance * scale`. The profile defines the residual and its scale. The LP result schema can also retain finite residual evidence above the tolerance.

## Verification

The native snapshot and LP input checks implement these contracts. The numeric data tests exercise zero extents and invalid CSR ordering, and the LP backend tests exercise a missing endpoint and an over-tolerance residual.
