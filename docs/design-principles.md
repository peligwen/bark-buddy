# Design Principles

Prescriptive rules for bark-buddy. These govern architectural decisions and code shape across the life of the project.

## Meta

**Multi-year horizon.** Durable shapes over clever shortcuts. Favor small, single-purpose files and classes from the start, not as an afterthought.

## Architecture boundaries

**Single transport.** `Dog.send_json()` is the only host→firmware path. JSON only; no text protocols.

**Layer discipline.** Firmware owns motion + hardware; host owns coordination/behaviors; web owns presentation. No layer reaches past its neighbor.

**Mock parity.** Mock shares source with real firmware; diverges only at platform shims (`firmware/mock/`) and link-time driver substitution. New drivers require a mock implementation before kernel code uses them.

## Code discipline

**Split early at natural seams.** A class with two reasons to change is two classes. File line count is a lagging signal; responsibility is the leading one.

**No speculative abstractions.** One-implementation ABCs, options no code sets, hooks with no callers — delete.

**No speculative features.** No defensive checks at internal boundaries. Trust internal code; validate at system boundaries.

**No dead code, no ghost comments.** Delete or don't.

**Comments state WHY** when non-obvious. Never narrate WHAT.

## Docs as source of truth

The five `docs/` files are the contract. If a behavior isn't in them, it isn't committed to.

Specs in `docs/superpowers/specs/`, plans in `docs/superpowers/plans/` — both ephemeral, pruned when complete.

`CLAUDE.md` points to core docs; never duplicates them.

## Dev loop

**Mock firmware is the primary dev loop.** Real hardware for integration only.

**`bark test` + `make bark-mock` must pass before commit.**
