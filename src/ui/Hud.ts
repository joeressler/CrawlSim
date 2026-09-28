export class Hud {
  constructor(host: HTMLElement) {
    const el = document.createElement("div");
    el.className = "hud";
    el.textContent = "WASD drive · R reset";
    host.appendChild(el);
  }
}
