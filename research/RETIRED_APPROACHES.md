# Withdrawn Instructor approaches

These approaches were removed from the active project at the user's request.
They are archived for recovery, not presented as a supported alternative.
Selected original reports under `evidence/` retain historical conclusions and
must be read in light of this withdrawal.

| Approach | Why it was withdrawn |
| --- | --- |
| Held-pitch equilibrium and stationary controller-history iteration | Requiring all controller memories to settle was not a justified definition of balanced-flight permission; repeated low-level checks did not establish the chart boundary. |
| Historical/prolonged upper-boundary and angle-target reductions | Target angle, delivered pitch permission and aircraft force balance are different constraints; these paths did not establish a single reliable boundary with valid interior points. |
| Static reference with retained trim and a full search over upper components | A native-checked F-16XL reference near 157.918 km/h had permitted/rejected/permitted points at roughly 1.34/1.355/1.38 g. Upper-cap classification was insufficient; component searches left unresolved intervals and took roughly 93–140 seconds in cited probes. |
| Progressively tightened connected turn / first restriction | Two local F-16XL gates were repeatable, but the general one-g initialization failed for J6K1 at fixed 30% flaps near 771 km/h. Local runtimes of roughly 4–14 seconds per column did not establish a usable fleet-wide method. |
| Removing negative-AoA stall as a cure for initialization | The user retained this chart policy globally, but it did not remove the J6K1 force/moment closure failure. It is not evidence that the connected-turn method works. |

The current static `instructor_aoa` approximation is retained **only because the
user explicitly requested an available experimental option**. Its free trim
allocation and omitted overload memory remain unresolved. Nothing in this
cleanup validates its chart-wide restriction.

Native predictor, protection, auto-trim, actuator and owner/timing ports remain
useful research evidence. Their correctness within tested scopes does not prove
any withdrawn reduction.
