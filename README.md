# Robot-2026 🤖

[![Documentación en DeepWiki](https://img.shields.io/badge/📖_Documentación-DeepWiki-blue?style=for-the-badge)](https://deepwiki.com/Apollo-Black-Cat/Robot-2026)

> **Nota:** La documentación completa y actualizada de nuestro robot, manuales y guías se encuentra en nuestro [DeepWiki](https://deepwiki.com/Apollo-Black-Cat/Robot-2026). ¡Visítalo para aprender más sobre cómo funciona todo!

¡Bienvenidos al repositorio oficial del código de nuestro robot para la temporada 2026! 🚀

Este proyecto contiene toda la programación necesaria para que nuestro robot se mueva, recolecte piezas y las lance, tanto de forma controlada por los pilotos como de manera autónoma.

## 🧩 ¿Qué hace nuestro robot?

El código está dividido en pequeñas partes (subsistemas) que se encargan de tareas específicas:

*   **Chasis (Drive):** Las ruedas y motores que nos permiten movernos en cualquier dirección en la cancha.
*   **Intake (Recolector):** El mecanismo que recoge las piezas del suelo o de la estación.
*   **Conveyor (Banda transportadora):** Mueve las piezas dentro del robot desde que entran hasta el lanzador.
*   **Indexer (Indexador):** Prepara la pieza y se asegura de que entre correctamente al lanzador en el momento exacto.
*   **Shooter (Lanzador):** Las ruedas que giran a toda velocidad para disparar la pieza hacia el objetivo.
*   **Visión:** Las cámaras que ayudan al robot a "ver" dónde está en la cancha y a apuntar automáticamente.

## 💻 ¿Cómo probar el código?

Si quieres probar o pasar el código al robot, estos son los comandos básicos a usar en la terminal:

Para revisar que el código está bien escrito:
```bash
./gradlew build
```

Para enviar el código al cerebro del robot (roboRIO):
```bash
./gradlew deploy
```

---
*Hecho con💛 por el equipo de Apollo-Black-Cat.*
