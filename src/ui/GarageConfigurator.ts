import {
  DEFAULT_GARAGE_CONFIG,
  MOTOR_OPTIONS,
  SERVO_OPTIONS,
  SHOCK_OPTIONS,
  WHEEL_OPTIONS,
  getSelectedOptionSummary,
  type GarageConfig,
  type MotorProfileId,
  type ServoProfileId,
  type ShockProfileId,
  type WheelProfileId,
} from "../garage/catalog.ts";

type Callbacks = {
  onChange: (config: GarageConfig) => void;
};

export class GarageConfigurator {
  private readonly root: HTMLDivElement;
  private readonly shockSelect: HTMLSelectElement;
  private readonly wheelSelect: HTMLSelectElement;
  private readonly motorSelect: HTMLSelectElement;
  private readonly servoSelect: HTMLSelectElement;
  private readonly shockSummary: HTMLDivElement;
  private readonly wheelSummary: HTMLDivElement;
  private readonly motorSummary: HTMLDivElement;
  private readonly servoSummary: HTMLDivElement;
  private readonly linkColorInput: HTMLInputElement;
  private readonly shockColorInput: HTMLInputElement;
  private readonly servoColorInput: HTMLInputElement;

  constructor(host: HTMLElement, initialConfig: GarageConfig, callbacks: Callbacks) {
    this.root = document.createElement("div");
    this.root.className = "garage-config";

    const title = document.createElement("h3");
    title.className = "garage-config__title";
    title.textContent = "Garage Setup";
    this.root.appendChild(title);

    this.shockSelect = this.buildSelect("Shocks", SHOCK_OPTIONS.map((o) => ({ value: o.id, label: o.label })));
    this.shockSummary = this.buildSummary();
    this.wheelSelect = this.buildSelect("Wheels / Tires", WHEEL_OPTIONS.map((o) => ({ value: o.id, label: o.label })));
    this.wheelSummary = this.buildSummary();
    this.motorSelect = this.buildSelect("Motor", MOTOR_OPTIONS.map((o) => ({ value: o.id, label: o.label })));
    this.motorSummary = this.buildSummary();
    this.servoSelect = this.buildSelect("Steering Servo", SERVO_OPTIONS.map((o) => ({ value: o.id, label: o.label })));
    this.servoSummary = this.buildSummary();
    this.linkColorInput = this.buildColor("Link Color");
    this.shockColorInput = this.buildColor("Shock Color");
    this.servoColorInput = this.buildColor("Servo Color");

    this.setConfig(initialConfig);

    const emit = (): void => callbacks.onChange(this.getConfig());
    this.shockSelect.addEventListener("change", () => {
      this.shockSummary.textContent = getSelectedOptionSummary(SHOCK_OPTIONS, this.shockSelect.value as ShockProfileId);
      emit();
    });
    this.wheelSelect.addEventListener("change", () => {
      this.wheelSummary.textContent = getSelectedOptionSummary(WHEEL_OPTIONS, this.wheelSelect.value as WheelProfileId);
      emit();
    });
    this.motorSelect.addEventListener("change", () => {
      this.motorSummary.textContent = getSelectedOptionSummary(MOTOR_OPTIONS, this.motorSelect.value as MotorProfileId);
      emit();
    });
    this.servoSelect.addEventListener("change", () => {
      this.servoSummary.textContent = getSelectedOptionSummary(SERVO_OPTIONS, this.servoSelect.value as ServoProfileId);
      emit();
    });
    this.linkColorInput.addEventListener("input", emit);
    this.shockColorInput.addEventListener("input", emit);
    this.servoColorInput.addEventListener("input", emit);

    host.appendChild(this.root);
  }

  setVisible(visible: boolean): void {
    this.root.style.display = visible ? "block" : "none";
  }

  setConfig(config: GarageConfig): void {
    const safe = { ...DEFAULT_GARAGE_CONFIG, ...config };
    this.shockSelect.value = safe.shockProfileId;
    this.wheelSelect.value = safe.wheelProfileId;
    this.motorSelect.value = safe.motorProfileId;
    this.servoSelect.value = safe.servoProfileId;
    this.linkColorInput.value = safe.linkColor;
    this.shockColorInput.value = safe.shockColor;
    this.servoColorInput.value = safe.servoColor;
    this.shockSummary.textContent = getSelectedOptionSummary(SHOCK_OPTIONS, safe.shockProfileId);
    this.wheelSummary.textContent = getSelectedOptionSummary(WHEEL_OPTIONS, safe.wheelProfileId);
    this.motorSummary.textContent = getSelectedOptionSummary(MOTOR_OPTIONS, safe.motorProfileId);
    this.servoSummary.textContent = getSelectedOptionSummary(SERVO_OPTIONS, safe.servoProfileId);
  }

  private getConfig(): GarageConfig {
    return {
      shockProfileId: this.shockSelect.value as ShockProfileId,
      wheelProfileId: this.wheelSelect.value as WheelProfileId,
      motorProfileId: this.motorSelect.value as MotorProfileId,
      servoProfileId: this.servoSelect.value as ServoProfileId,
      linkColor: this.linkColorInput.value,
      shockColor: this.shockColorInput.value,
      servoColor: this.servoColorInput.value,
    };
  }

  private buildSelect(labelText: string, options: { value: string; label: string }[]): HTMLSelectElement {
    const label = document.createElement("label");
    label.className = "garage-config__field";

    const title = document.createElement("span");
    title.className = "garage-config__field-title";
    title.textContent = labelText;
    label.appendChild(title);

    const select = document.createElement("select");
    select.className = "garage-config__input";
    for (const option of options) {
      const el = document.createElement("option");
      el.value = option.value;
      el.textContent = option.label;
      select.appendChild(el);
    }
    label.appendChild(select);
    this.root.appendChild(label);
    return select;
  }

  private buildSummary(): HTMLDivElement {
    const summary = document.createElement("div");
    summary.className = "garage-config__summary";
    this.root.appendChild(summary);
    return summary;
  }

  private buildColor(labelText: string): HTMLInputElement {
    const label = document.createElement("label");
    label.className = "garage-config__field";

    const title = document.createElement("span");
    title.className = "garage-config__field-title";
    title.textContent = labelText;
    label.appendChild(title);

    const input = document.createElement("input");
    input.type = "color";
    input.className = "garage-config__color";
    label.appendChild(input);
    this.root.appendChild(label);
    return input;
  }
}
