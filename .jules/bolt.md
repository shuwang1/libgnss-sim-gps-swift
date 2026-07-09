## 2026-06-27 - Replace expensive pow() with direct multiplication on hot paths
**Learning:** Using `pow(x, 2)` and `pow(x, 3)` for small integer exponents in Swift incurs the overhead of generalized floating-point operations. In high-frequency hot paths like per-sample or per-step physical modeling (e.g., Klobuchar ionospheric delay model), this overhead compounds rapidly.
**Action:** Replace `pow()` calls for small, constant exponents with direct multiplications (e.g., `x * x` and `x * x * x`) to significantly reduce CPU cycle cost without altering mathematical outcome.
