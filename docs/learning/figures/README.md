# learning/figures — diagram generator

Most PNGs in this directory are produced by `generate.py`
(matplotlib). A small number of comparison / structure diagrams
are generated from `.dot` files via graphviz — currently
[`22-tracker-stack-alternatives.dot`](22-tracker-stack-alternatives.dot).
Running `generate.py` renders those too (via its `render_dot_figures()`
step, which shells out to the `dot` binary — see `DOT_FIGURES`), so a
single `generate.py` run reproduces *every* PNG in this directory.
To re-render a `.dot` by hand: `dot -Tpng <name>.dot -o <name>.png`.
If `dot` is not installed, `generate.py` warns and skips the `.dot`
figures but still produces all matplotlib ones.

**Do not edit the PNGs by hand** — change the source script or
`.dot` file and re-render.

## Regenerating

```bash
# one-time venv setup (matplotlib + numpy)
python3 -m venv /tmp/claude/learning-venv
/tmp/claude/learning-venv/bin/pip install matplotlib numpy

# regenerate all PNGs
cd docs/learning/figures
/tmp/claude/learning-venv/bin/python generate.py
```

Output sizes are kept reasonable (under ~200 kB each) by relying
on matplotlib's PNG defaults at `dpi = 110`.

## Adding a new figure

1. Add a `fig_*()` function in `generate.py` that produces the
   plot. Use the existing colour palette
   (`#1f3a5f` = navy, `#aa3333` = red, `#2d8659` = green,
   `#85bbdb` / `#9cc3e8` = light blues) for visual consistency.
2. Call your function from `main()`.
3. Re-run the script.
4. Reference the PNG from the relevant chapter:
   `![alt text](figures/your-figure.png)`.

## Style rules

- Keep titles short and informative — they end up as image
  captions implicitly.
- Always include axis labels with units.
- Prefer light grid (`linestyle=":", alpha=0.4`) over heavy grid.
- Use the same ellipse helper (`_draw_ellipse`) for covariance
  visuals so they look uniform across chapters.
- Save with `fig.savefig(..., bbox_inches="tight")` (handled in
  the `save()` helper).
