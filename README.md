# RITE project page

Static project page for the RITE manuscript. No build step or external web dependency is required. The layout follows a conventional academic project-page hierarchy: paper metadata and resources first, followed by the abstract, method, evaluation protocol, quantitative tables, mechanism figures, qualitative videos, and citation.

The page content and downloadable TeX are synchronized with `paper/9.7修改-RITE_ICRA2027_中文论文优化稿.tex`. The framework image is synchronized with `paper/figures/RITE.png`. The qualitative section includes three synchronized sequences (`01e4`, `3ada`, and `952c`) that stack the six-camera Hard-S3 input above the degraded-versus-restored BEV comparison, with WebM playback and MP4 fallbacks.

Preview locally from this directory:

```bash
python3 -m http.server 8000
```

Then open `http://localhost:8000`.

Before public release, replace the anonymous author/affiliation fields, add the final paper and code URLs, and update the provisional BibTeX entry.
