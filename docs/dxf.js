/* ============================================================
 * LECTOR DE PLANOS DXF — 100% en el navegador (sin backend).
 * ------------------------------------------------------------
 * Puerto de backend/main.py (extraer_plano): misma convención de capas,
 * misma normalización de coordenadas, mismo JSON de salida. Verificado
 * dando resultados idénticos a la versión Python con los DXF de ejemplo
 * (ver simulador-evacuacion-web/scratch/comparar.mjs).
 *
 * Usa dxf-parser (CDN, ver import map en index.html) solo para parsear el
 * texto DXF a entidades; la extracción geométrica es propia.
 * ============================================================ */

import DxfParser from 'dxf-parser';

const CAPAS_SALIDA = ['SALIDA', 'EXIT', 'PUERTA', 'DOOR', 'EGRESO'];
const CAPAS_OBSTACULO = [
  'OBSTAC', 'MOBIL', 'MUEBLE', 'ESTANT', 'EQUIPO', 'MAQUIN', 'GONDOL',
  'FURN', 'RACK', 'MESA', 'MOSTRADOR',
];
/** $INSUNITS del DXF -> metros. */
const ESCALA_POR_INSUNITS = { 1: 0.0254, 2: 0.3048, 4: 0.001, 5: 0.01, 6: 1.0 };
const NOMBRES_UNIDAD = { 1: 'pulgadas', 2: 'pies', 4: 'mm', 5: 'cm', 6: 'm' };
const LONGITUD_MINIMA = 0.01;
const TIPOS_SOPORTADOS = ['LINE', 'LWPOLYLINE', 'POLYLINE'];

const contiene = (capa, claves) => {
  const n = String(capa ?? '').toUpperCase();
  return claves.some((c) => n.includes(c));
};

/** Segmentos [[x1,y1],[x2,y2]] de una entidad soportada. */
function segmentosDe(e) {
  const v = e.vertices ?? [];
  if (e.type === 'LINE') {
    return v.length >= 2 ? [[[v[0].x, v[0].y], [v[1].x, v[1].y]]] : [];
  }
  const pts = v.map((p) => [p.x, p.y]);
  const segs = [];
  for (let i = 0; i + 1 < pts.length; i++) segs.push([pts[i], pts[i + 1]]);
  if (e.shape && pts.length > 2) segs.push([pts[pts.length - 1], pts[0]]);
  return segs;
}

/**
 * Extrae muros/salidas/obstáculos de un DXF ya parseado (objeto de dxf-parser).
 * @throws {Error} con `.status` si el DXF no tiene muros.
 */
export function extraerPlano(dxf) {
  const insunits = Number(dxf?.header?.$INSUNITS ?? 0);
  const escala = ESCALA_POR_INSUNITS[insunits] ?? 1.0;
  const advertencias = [];
  if (!(insunits in ESCALA_POR_INSUNITS)) {
    advertencias.push(`Unidad DXF desconocida ($INSUNITS=${insunits}); se asumen metros.`);
  }

  const murosRaw = [], salidasRaw = [], obstaculosRaw = [];
  let ignoradas = 0;
  for (const e of dxf?.entities ?? []) {
    if (!TIPOS_SOPORTADOS.includes(e.type)) { ignoradas++; continue; }
    if (contiene(e.layer, CAPAS_SALIDA)) salidasRaw.push(...segmentosDe(e));
    else if (contiene(e.layer, CAPAS_OBSTACULO)) {
      const g = segmentosDe(e);
      if (g.length) obstaculosRaw.push(g);
    } else murosRaw.push(...segmentosDe(e));
  }

  if (!murosRaw.length) {
    const err = new Error('El DXF no contiene líneas de muros (LINE/LWPOLYLINE/POLYLINE).');
    err.status = 422;
    throw err;
  }
  if (ignoradas) {
    advertencias.push(`Se ignoraron ${ignoradas} entidades no soportadas (arcos, textos, etc.).`);
  }
  if (!salidasRaw.length) {
    advertencias.push('No se detectaron salidas: dibujá los vanos como líneas en una capa llamada SALIDAS.');
  }

  const esc = (s) => [[s[0][0] * escala, s[0][1] * escala], [s[1][0] * escala, s[1][1] * escala]];
  const murosM = murosRaw.map(esc);
  const salidasM = salidasRaw.map(esc);
  const obstaculosM = obstaculosRaw.map((g) => g.map(esc));

  const todos = [...murosM, ...salidasM];
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const [a, b] of todos) {
    minX = Math.min(minX, a[0], b[0]); minY = Math.min(minY, a[1], b[1]);
    maxX = Math.max(maxX, a[0], b[0]); maxY = Math.max(maxY, a[1], b[1]);
  }
  const aSeg = (s) => {
    const [[x1, y1], [x2, y2]] = s;
    if (Math.hypot(x2 - x1, y2 - y1) < LONGITUD_MINIMA) return null;
    return { x1: x1 - minX, y1: y1 - minY, x2: x2 - minX, y2: y2 - minY };
  };

  const muros = murosM.map(aSeg).filter(Boolean);
  const salidas = salidasM.map(aSeg).filter(Boolean)
    .map((s) => ({ ...s, ancho: Math.hypot(s.x2 - s.x1, s.y2 - s.y1) }));
  const obstaculos = obstaculosM
    .map((g) => g.map(aSeg).filter(Boolean))
    .filter((g) => g.length);

  const ancho = Math.max(maxX - minX, 0.1), alto = Math.max(maxY - minY, 0.1);
  return {
    muros, salidas,
    bbox: { ancho, alto },
    unidades: NOMBRES_UNIDAD[insunits] ?? 'sin definir (se asumen m)',
    superficie_bbox: Math.round(ancho * alto * 100) / 100,
    advertencias,
    obstaculos,
  };
}

/**
 * Lee el texto de un archivo .dxf y devuelve el Plano (mismo formato que
 * el backend). @param {string} texto Contenido del archivo .dxf.
 */
export function leerDXF(texto) {
  let dxf;
  try {
    dxf = new DxfParser().parseSync(texto);
  } catch (e) {
    const err = new Error(`No se pudo interpretar el DXF: ${e.message}`);
    err.status = 422;
    throw err;
  }
  return extraerPlano(dxf);
}

/**
 * Plano de ejemplo (oficina 20×10 m, 2 salidas, 2 mesas) ya resuelto —
 * evita volver a parsear un DXF grande solo para el botón "Cargar
 * ejemplo". Generado con `backend/generar_dxf_prueba.py` y verificado
 * idéntico entre esta función y el backend Python.
 */
export const PLANO_EJEMPLO = {
  muros: [
    { x1: 0, y1: 0, x2: 9.45, y2: 0 }, { x1: 10.55, y1: 0, x2: 20, y2: 0 },
    { x1: 20, y1: 0, x2: 20, y2: 4.725 }, { x1: 20, y1: 5.275, x2: 20, y2: 10 },
    { x1: 20, y1: 10, x2: 0, y2: 10 }, { x1: 0, y1: 10, x2: 0, y2: 0 },
    { x1: 13, y1: 4, x2: 13, y2: 10 },
  ],
  salidas: [
    { x1: 9.45, y1: 0, x2: 10.55, y2: 0, ancho: 1.1000000000000014 },
    { x1: 20, y1: 4.725, x2: 20, y2: 5.275, ancho: 0.5500000000000007 },
  ],
  bbox: { ancho: 20, alto: 10 },
  unidades: 'm',
  superficie_bbox: 200,
  advertencias: [],
  obstaculos: [
    [{ x1: 3, y1: 5, x2: 5.4, y2: 5 }, { x1: 5.4, y1: 5, x2: 5.4, y2: 6 },
      { x1: 5.4, y1: 6, x2: 3, y2: 6 }, { x1: 3, y1: 6, x2: 3, y2: 5 }],
    [{ x1: 7, y1: 5, x2: 9.4, y2: 5 }, { x1: 9.4, y1: 5, x2: 9.4, y2: 6 },
      { x1: 9.4, y1: 6, x2: 7, y2: 6 }, { x1: 7, y1: 6, x2: 7, y2: 5 }],
  ],
};
