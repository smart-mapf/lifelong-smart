import { test, expect } from "bun:test";
import { readFileSync } from "node:fs";

const source = readFileSync(new URL("../source/_static/analytics.js", import.meta.url), "utf8");

function page() {
  class Element {
    closest(selector) {
      expect(selector).toBe(".lsmart-hero-buttons a");
      return this.button || null;
    }
  }
  const scripts = [];
  let click;
  const document = {
    createElement: (tag) => {
      expect(tag).toBe("script");
      return {};
    },
    head: { append: (script) => scripts.push(script) },
    addEventListener: (type, handler) => {
      expect(type).toBe("click");
      click = handler;
    },
  };
  const window = {};
  new Function("window", "document", "Element", source)(window, document, Element);
  return { window, scripts, Element, click, commands: () => window.dataLayer.map((args) => Array.from(args)) };
}

test("loads GA4 immediately without consent gating or additional UI", () => {
  const p = page();
  expect(p.scripts).toEqual([{
    async: true,
    src: "https://www.googletagmanager.com/gtag/js?id=G-YPX3G9Y7TB",
  }]);
  expect(p.commands().map(([command]) => command)).toEqual(["js", "config"]);
  expect(p.commands()[1]).toEqual(["config", "G-YPX3G9Y7TB"]);
});

test("tracks all homepage buttons, including clicks on nested icons", () => {
  const p = page();
  p.click({ target: {} });
  p.click({ target: new p.Element() });
  expect(p.commands()).toHaveLength(2);
  for (const label of ["Install", "Documentation", "Docker Hub", "GitHub", "Youtube Tutorial"]) {
    const icon = new p.Element();
    icon.button = { textContent: ` ${label} `, href: `https://example.com/${encodeURIComponent(label)}` };
    p.click({ target: icon });
    expect(p.commands().at(-1)).toEqual(["event", "homepage_button_click", {
      link_text: label,
      link_url: icon.button.href,
      transport_type: "beacon",
    }]);
  }
  expect(p.commands()).toHaveLength(7);
});
