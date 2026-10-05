export class Hud {
  constructor(host: HTMLElement) {
    const el = document.createElement("div");
    el.className = "hud";
    el.textContent = "WASD drive · R reset · Scene selector at top-left";
    host.appendChild(el);
  }
}
