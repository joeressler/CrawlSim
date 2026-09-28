import "./ui/styles.css";
import { PhysicsWorld } from "./physics/PhysicsWorld.ts";
import { Game } from "./game/Game.ts";

await PhysicsWorld.init();
new Game(document.body).start();
