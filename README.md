# Simulador de Evacuación 3D — by PrevenLab

Simulador de evacuación con modelado basado en agentes (ABM), parametrizado según
**Ley 19.587 / Decreto 351/79 (Cap. 18 — Medios de escape)**.

**Demo (GitHub Pages):** https://hiperfrenetico.github.io/simulador-evacuacion-by-prevenlab/

🧪 **Versión beta abierta.** Herramienta orientativa en prueba, no reemplaza el cálculo
profesional de medios de escape. Comentarios y sugerencias: https://tally.so/r/zENQrk

## Qué hace
- Recinto genérico editable (ancho × profundidad a elección) o plano real en DXF.
- Salidas y obstáculos a elección (clic / arrastre), guarda y abre escenarios (.json).
- Ocupantes siguen un mapa de recorridos real (no línea recta), eligen salida según
  cola/caudal, y tienen 3 niveles de entrenamiento (pánico / básico / avanzado).
- SEGU: el responsable de evacuación, da instrucciones y es el último en salir.

## Cómo funciona esta versión
Página 100% estática (`docs/`), sin backend: el lector de DXF corre en el propio
navegador (`dxf-parser` por CDN + lógica propia en `docs/dxf.js`, puerto exacto del
backend Python de la versión de escritorio — verificado con el mismo resultado en los
DXF de ejemplo). El DWG solo funciona en la versión de PC (necesita un conversor
instalado localmente).

## Desarrollo
Este repo se genera desde el código fuente real del proyecto (no se edita `docs/`
a mano). Fuente y versión de escritorio (con soporte DXF/DWG completo vía backend
Python): preguntarle a Luciano / PrevenLab.
