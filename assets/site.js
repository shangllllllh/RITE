(() => {
  const header = document.querySelector('[data-header]');
  const navToggle = document.querySelector('[data-nav-toggle]');
  const nav = document.querySelector('[data-nav]');

  const updateHeader = () => header?.classList.toggle('scrolled', window.scrollY > 28);
  updateHeader();
  window.addEventListener('scroll', updateHeader, { passive: true });

  navToggle?.addEventListener('click', () => {
    const open = nav.classList.toggle('open');
    navToggle.setAttribute('aria-expanded', String(open));
  });

  nav?.querySelectorAll('a').forEach((link) => {
    link.addEventListener('click', () => {
      nav.classList.remove('open');
      navToggle?.setAttribute('aria-expanded', 'false');
    });
  });

  const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const reveals = document.querySelectorAll('.reveal');
  if (reducedMotion || !('IntersectionObserver' in window)) {
    reveals.forEach((element) => element.classList.add('visible'));
  } else {
    const observer = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (entry.isIntersecting) {
          entry.target.classList.add('visible');
          observer.unobserve(entry.target);
        }
      });
    }, { threshold: 0.11, rootMargin: '0px 0px -35px' });
    reveals.forEach((element) => observer.observe(element));
  }

  // Re-apply deep links after local assets have initialized. This keeps direct
  // links to long sections reliable in browsers that restore scroll early.
  if (window.location.hash) {
    window.requestAnimationFrame(() => {
      document.querySelector(window.location.hash)?.scrollIntoView();
    });
  }

  document.querySelectorAll('[data-image-switcher]').forEach((viewer) => {
    const image = viewer.querySelector('[data-switch-target]');
    const buttons = viewer.querySelectorAll('[data-image]');
    buttons.forEach((button) => {
      button.addEventListener('click', () => {
        buttons.forEach((item) => item.classList.remove('active'));
        button.classList.add('active');
        image.style.opacity = '0.35';
        const nextSource = button.dataset.image;
        const preload = new Image();
        preload.onload = () => {
          image.src = nextSource;
          image.alt = button.textContent.trim() === 'Clean'
            ? 'Six-camera clean surround view'
            : 'Six-camera view under severe glare';
          image.style.opacity = '1';
        };
        preload.src = nextSource;
      });
    });
    image.style.transition = 'opacity .22s ease';
  });

  document.querySelectorAll('[data-video-gallery]').forEach((gallery) => {
    const video = gallery.querySelector('[data-case-video]');
    const source = video?.querySelector('[data-video-primary]');
    const fallback = video?.querySelector('[data-video-fallback]');
    const title = gallery.querySelector('[data-scene-title-target]');
    const meta = gallery.querySelector('[data-scene-meta-target]');
    const buttons = gallery.querySelectorAll('[data-video-src]');

    if (reducedMotion) {
      video?.removeAttribute('autoplay');
      video?.pause();
    }

    buttons.forEach((button) => {
      button.addEventListener('click', () => {
        if (!video || !source || button.classList.contains('active')) return;

        buttons.forEach((item) => {
          item.classList.remove('active');
          item.setAttribute('aria-selected', 'false');
        });
        button.classList.add('active');
        button.setAttribute('aria-selected', 'true');

        video.pause();
        video.poster = button.dataset.videoPoster;
        source.src = button.dataset.videoSrc;
        if (fallback) fallback.src = button.dataset.videoFallback;
        if (title) title.textContent = button.dataset.sceneTitle;
        if (meta) meta.textContent = button.dataset.sceneMeta;
        video.load();
        if (!reducedMotion) video.play().catch(() => {});
      });
    });
  });

  const copyButton = document.querySelector('[data-copy-bib]');
  const toast = document.querySelector('[data-toast]');
  let toastTimer;
  copyButton?.addEventListener('click', async () => {
    const bibtex = document.querySelector('#bibtex')?.textContent || '';
    try {
      await navigator.clipboard.writeText(bibtex);
    } catch (_) {
      const selection = window.getSelection();
      const range = document.createRange();
      range.selectNodeContents(document.querySelector('#bibtex'));
      selection.removeAllRanges();
      selection.addRange(range);
      document.execCommand('copy');
      selection.removeAllRanges();
    }
    copyButton.innerHTML = '<i></i> Copied';
    toast?.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => {
      toast?.classList.remove('show');
      copyButton.innerHTML = '<i></i> Copy';
    }, 1800);
  });
})();
