document.addEventListener('DOMContentLoaded', () => {
  // Strip class prefix inside main content signatures (page body)
  document.querySelectorAll('code.sig-name.descname').forEach(el => {
    const txt = el.textContent;
    if (txt.startsWith('VisionProStreamer.')) {
      el.textContent = txt.replace(/^VisionProStreamer\./, '');
    }
  });

  // Strip class prefix in sidebar navigation entries
  document.querySelectorAll('.wy-menu code').forEach(codeEl => {
    const txt = codeEl.textContent.trim();
    if (/^VisionProStreamer\.[A-Za-z_]/.test(txt)) {
      codeEl.textContent = txt.replace(/^VisionProStreamer\./, '');
    }
  });
});
