# Rebuilt 2026 documentation

The docs site for FRC Team 3748's 2026 robot code. Built with Jekyll
and served via GitHub Pages.

## Local development

```bash
cd docs
bundle install
bundle exec jekyll serve
```

Open <http://localhost:4000>.

## Publishing

In the repo's GitHub settings:

1. **Settings → Pages → Source** → "Deploy from a branch".
2. **Branch** → `main`, **Folder** → `/docs`.
3. Save. GitHub Pages will build and publish within a couple of minutes.

The site URL appears in the same Pages settings panel once it's
deployed.

## Editing

- Content is in `*.md` files. Front matter at the top of each file
  controls layout, title, and the eyebrow/description shown in the
  header.
- The navigation sidebar is generated from `_data/nav.yml`. Add new
  pages there too.
- The layout is `_layouts/default.html`. All pages use it.
- Styles are in `assets/css/style.css`. Syntax highlighting is in
  `assets/css/syntax.css`. JS for theme toggle, mobile sidebar, TOC,
  and code-copy buttons is in `assets/js/main.js`.

## Design notes

The visual style is inspired by [Fusion 0.3](https://elttob.uk/Fusion/0.3/)
— dark theme with navy/teal accents by default, a light theme via the
toggle, Inter for body text, JetBrains Mono for code, a left sidebar
nav with section headings, and an auto-generated "On this page" TOC on
wide screens.
