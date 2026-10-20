document.addEventListener("click", (event) => {
  if (!(event.target instanceof Element)) return;
  const button = event.target.closest(".lsmart-hero-buttons a");
  if (!button || typeof window.gtag !== "function") return;

  window.gtag("event", "homepage_button_click", {
    link_text: button.textContent.trim(),
    link_url: button.href,
    transport_type: "beacon",
  });
});
