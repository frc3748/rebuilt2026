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

This site uses **Jekyll 4**, which GitHub Pages does not build by
default (it ships Jekyll 3 via the `github-pages` gem). Build it
yourself via the included GitHub Actions workflow.

1. **Settings → Pages → Source** → "GitHub Actions".
2. Push to `main`. The `.github/workflows/pages.yml` workflow builds
   the site with the exact `Gemfile` in this directory and deploys it.
3. Watch the **Actions** tab for build progress. The site URL appears
   on the **Pages** settings panel once deployment completes.

If you instead choose "Deploy from a branch → `main` / `/docs`",
GitHub will try to build with its bundled Jekyll 3 + `github-pages`
gem and the build will either fail or render with broken styles. Use
the Actions source.

Set `url` and `baseurl` in `_config.yml` to match your repo:

```yaml
url: "https://<owner>.github.io"
baseurl: "/<repo-name>"
```

The workflow overrides `baseurl` automatically with the path GitHub
Pages assigns, so the local-dev `baseurl` only needs to match for
`bundle exec jekyll serve`.

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
