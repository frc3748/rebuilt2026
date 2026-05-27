(function () {
  "use strict";

  /* ─── Theme toggle (persisted) ─── */
  const THEME_KEY = "rebuilt2026-theme";
  const root = document.documentElement;

  const stored = localStorage.getItem(THEME_KEY);
  if (stored === "light" || stored === "dark") {
    root.setAttribute("data-theme", stored);
  } else if (window.matchMedia && window.matchMedia("(prefers-color-scheme: light)").matches) {
    root.setAttribute("data-theme", "light");
  }

  const themeBtn = document.getElementById("theme-toggle");
  if (themeBtn) {
    themeBtn.addEventListener("click", () => {
      const next = root.getAttribute("data-theme") === "dark" ? "light" : "dark";
      root.setAttribute("data-theme", next);
      localStorage.setItem(THEME_KEY, next);
    });
  }

  /* ─── Mobile sidebar toggle ─── */
  const sidebar = document.getElementById("sidebar");
  const hamburger = document.getElementById("sidebar-toggle");
  if (hamburger && sidebar) {
    hamburger.addEventListener("click", () => sidebar.classList.toggle("open"));
    document.addEventListener("click", (e) => {
      if (window.innerWidth > 960) return;
      if (!sidebar.contains(e.target) && !hamburger.contains(e.target)) {
        sidebar.classList.remove("open");
      }
    });
  }

  /* ─── Sidebar filter ─── */
  const search = document.getElementById("nav-search");
  if (search) {
    search.addEventListener("input", () => {
      const q = search.value.trim().toLowerCase();
      document.querySelectorAll(".nav-section").forEach((section) => {
        let visible = 0;
        section.querySelectorAll(".nav-list li").forEach((li) => {
          const t = li.textContent.toLowerCase();
          const match = !q || t.includes(q);
          li.classList.toggle("hidden", !match);
          if (match) visible++;
        });
        section.classList.toggle("hidden", visible === 0);
      });
    });
  }

  /* ─── Auto-generate "On this page" TOC from h2/h3 in the article ─── */
  const toc = document.getElementById("toc");
  const articleBody = document.querySelector(".article-body");
  if (toc && articleBody) {
    const headings = articleBody.querySelectorAll("h2, h3");
    if (headings.length > 1) {
      const items = [];
      headings.forEach((h) => {
        if (!h.id) {
          h.id = h.textContent
            .trim()
            .toLowerCase()
            .replace(/[^\w\s-]/g, "")
            .replace(/\s+/g, "-");
        }
        items.push(`<li class="toc-${h.tagName.toLowerCase()}"><a href="#${h.id}">${h.textContent}</a></li>`);
      });
      toc.innerHTML = `<h5>On this page</h5><ul>${items.join("")}</ul>`;

      const links = toc.querySelectorAll("a");
      const linkMap = new Map();
      links.forEach((a) => linkMap.set(a.getAttribute("href").slice(1), a));

      const observer = new IntersectionObserver(
        (entries) => {
          entries.forEach((entry) => {
            const id = entry.target.id;
            const link = linkMap.get(id);
            if (!link) return;
            if (entry.isIntersecting) {
              links.forEach((a) => a.classList.remove("active"));
              link.classList.add("active");
            }
          });
        },
        { rootMargin: "-80px 0px -70% 0px", threshold: 0 }
      );
      headings.forEach((h) => observer.observe(h));
    }
  }

  /* ─── Add copy buttons to code blocks ─── */
  document.querySelectorAll(".article-body pre").forEach((pre) => {
    if (pre.querySelector(".copy-btn")) return;
    const btn = document.createElement("button");
    btn.className = "copy-btn";
    btn.type = "button";
    btn.textContent = "Copy";
    Object.assign(btn.style, {
      position: "absolute",
      top: "0.5rem",
      right: "0.5rem",
      padding: "0.2rem 0.55rem",
      fontSize: "0.72rem",
      fontFamily: "var(--font-sans)",
      background: "var(--bg-soft)",
      color: "var(--text-muted)",
      border: "1px solid var(--border)",
      borderRadius: "4px",
      cursor: "pointer",
      opacity: "0",
      transition: "opacity 150ms ease",
    });
    pre.style.position = "relative";
    pre.appendChild(btn);
    pre.addEventListener("mouseenter", () => (btn.style.opacity = "1"));
    pre.addEventListener("mouseleave", () => (btn.style.opacity = "0"));
    btn.addEventListener("click", async () => {
      const code = pre.querySelector("code");
      try {
        await navigator.clipboard.writeText(code ? code.innerText : pre.innerText);
        btn.textContent = "Copied!";
        setTimeout(() => (btn.textContent = "Copy"), 1500);
      } catch (_) {
        btn.textContent = "Failed";
      }
    });
  });
})();
