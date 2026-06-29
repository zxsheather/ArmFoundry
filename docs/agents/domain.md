# Domain Docs

How the engineering skills should consume this repo's domain documentation when exploring the codebase.

## Layout

This repo uses a single-context layout.

Expected files:

- `CONTEXT.md` at the repo root
- `docs/adr/` for architectural decision records

If these files don't exist, proceed silently. The producer skill (`grill-with-docs`) creates them lazily when terms or decisions actually get resolved.

## Consumer rules

Before exploring, read `CONTEXT.md` if it exists.

Before proposing architecture changes or implementation strategy, read ADRs in `docs/adr/` that touch the relevant area.

When output names a domain concept, use the term as defined in `CONTEXT.md`. If the concept is missing, note the gap instead of inventing project vocabulary.

If output contradicts an existing ADR, surface the conflict explicitly.
