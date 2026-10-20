(() => {
  const measurementId = "G-YPX3G9Y7TB";
  window.dataLayer = window.dataLayer || [];
  window.gtag = function () { window.dataLayer.push(arguments); };
  window.gtag("js", new Date());
  window.gtag("config", measurementId);
  const script = document.createElement("script");
  script.async = true;
  script.src = `https://www.googletagmanager.com/gtag/js?id=${measurementId}`;
  document.head.append(script);

  document.addEventListener("click", (event) => {
    if (!(event.target instanceof Element)) return;
    const button = event.target.closest(".lsmart-hero-buttons a");
    if (!button) return;
    window.gtag("event", "homepage_button_click", {
      link_text: button.textContent.trim(),
      link_url: button.href,
      transport_type: "beacon",
    });
  });
})();
