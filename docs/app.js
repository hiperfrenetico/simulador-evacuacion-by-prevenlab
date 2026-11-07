/* ============================================================
 * Simulador de Evacuación 3D — MVP
 * ------------------------------------------------------------
 * Motor ABM (Agent-Based Model) simplificado, inspirado en el
 * modelo de fuerza social de Helbing:
 *   - Fuerza de atracción direccional hacia la salida más cercana.
 *   - Fuerza de repulsión exponencial entre agentes (evita solapes
 *     y genera el cuello de botella en la abertura).
 *   - Fuerza de repulsión + colisión ESTRICTA contra muros
 *     (proyección posicional: un agente jamás atraviesa un muro).
 *
 * Parámetros normativos (Ley 19.587 / Dec. 351/79, Cap. 18):
 *   - N (ocupantes) = Superficie / Factor de ocupación x.
 *   - U.A.S.: 1 unidad de ancho de salida = 0,55 m libres.
 *   - n requeridas = N / 100 (redondeo por exceso, mínimo 2).
 *
 * Coordenadas: la física corre en 2D sobre el plano (x, y) del DXF
 * (metros). El render 3D mapea (x, y) → (x, 0, -y) de Three.js.
 * ============================================================ */

/**
 * Configuración por entorno (config.js). Vacía en la versión de PC; en la
 * versión web trae las URLs de la Edge Function y el enlace de comentarios.
 * @type {{web?:boolean, urlProcesar?:string, urlEjemplo?:string, urlComentarios?:string}}
 */
const CFG = window.EVAC_CONFIG ?? {};

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { leerDXF, PLANO_EJEMPLO } from './dxf.js';

/* ------------------------------------------------------------
 * Tipos (JSDoc)
 * ------------------------------------------------------------
 * @typedef {{x1:number, y1:number, x2:number, y2:number}} Segmento
 * @typedef {Segmento & {ancho:number, cx:number, cy:number}} Salida
 * @typedef {{segs:Segmento[], rect?:{x0:number, y0:number, x1:number, y1:number}}} Obstaculo
 *   Obstáculo bajo (mueble, estantería, máquina): rectángulo dibujado en el
 *   visor (`rect`) o grupo de segmentos de una entidad de la capa CAD.
 * @typedef {{muros:Segmento[], salidas:Salida[], obstaculos:Obstaculo[],
 *            bbox:{ancho:number, alto:number},
 *            nombre:string, superficie:number|null}} Plano
 * @typedef {{id:number, x:number, y:number, vx:number, vy:number,
 *            prevX:number, prevY:number, velDeseada:number,
 *            evacuado:boolean, mesh:THREE.Mesh}} Agente
 */

/* ---------------- Constantes físicas y normativas ---------------- */

/** @type {number} Metros libres por Unidad de Ancho de Salida (Dec. 351/79). */
const UAS_METROS = 0.55;
/** @type {number} Radio corporal del agente (m) — persona ≈ 0,50 m de hombros. */
const RADIO_AGENTE = 0.25;
/** @type {number} Altura del cilindro que representa al ocupante (m). */
const ALTURA_AGENTE = 1.70;
/** @type {number} Altura de extrusión de los muros (m). */
const ALTURA_MURO = 2.60;
/** @type {number} Espesor visual de los muros (m). */
const ESPESOR_MURO = 0.15;
/**
 * @type {number} Holgura de colisión contra el eje del muro (m).
 * Se usa RADIO_AGENTE + HOLGURA (y no + medio espesor): así un vano de
 * 1 U.A.S. (0,55 m) deja pasar a una persona de 0,50 m de hombros, como
 * prevé la norma. El leve solape visual con la cara del muro es aceptable.
 */
const HOLGURA_MURO = 0.02;
/** @type {number} Tiempo de relajación del modelo de fuerza social (s). */
const TAU = 0.4;
/**
 * Repulsión agente–agente: F = A · e^((2r − d)/B).
 * B pequeño (≈ 0,10 m, como en Helbing) hace la repulsión de corto
 * alcance: con B grande dos agentes que flanquean un vano angosto quedan
 * en equilibrio estable con la atracción y bloquean la puerta.
 */
const REP_AG_A = 5.0, REP_AG_B = 0.10;
/** Repulsión agente–muro: F = A · e^((r − d)/B) */
const REP_MU_A = 5.0, REP_MU_B = 0.08;
/**
 * @type {number} Amplitud de la fuerza de fluctuación aleatoria (m/s²).
 * Término ξ del modelo de Helbing: rompe los "arcos" (equilibrios de
 * fuerzas) que se forman en las puertas angostas y traban la salida.
 */
const RUIDO_A = 0.5;
/** @type {number} Multiplicador del ruido para agentes casi estáticos. */
const RUIDO_ANTIATASCO = 6;
/**
 * Mecanismo de "ceder el paso": si un agente pasa más de un umbral
 * aleatorio (1,0–2,5 s) casi detenido, retrocede durante ~1 s para
 * deshacer la cuña con el agente vecino (como se alterna en una puerta).
 */
const CEDER_UMBRAL_MIN = 1.0, CEDER_UMBRAL_VAR = 1.5;   // s
const CEDER_DURACION_MIN = 0.8, CEDER_DURACION_VAR = 0.7; // s
/** @type {number} Rapidez por debajo de la cual se considera "atascado" (m/s). */
const VEL_ATASCO = 0.2;
/**
 * Frenado por densidad (diagrama fundamental peatonal): la velocidad
 * deseada se reduce con la cantidad de vecinos próximos. Sin esto el
 * caudal por los vanos resulta muy optimista (≈ 6 p/s/m vs. ~1,9 p/s/m
 * empíricos) y los tiempos de evacuación dan irrealmente bajos.
 */
const DENSIDAD_RADIO = 1.0;    // m: radio para contar vecinos
const DENSIDAD_COEF = 0.35;    // reducción por vecino adicional
/** @type {number} Distancia máx. a la que un muro ejerce repulsión (m). */
const ALCANCE_REP_MURO = 1.0;
/** @type {number} Distancia máx. a la que otro agente ejerce repulsión (m). */
const ALCANCE_REP_AGENTE = 1.5;
/** @type {number} Paso de integración física de referencia (s). */
const DT_FISICA = 1 / 120;
/** @type {number} Tope de ocupantes simulados (rendimiento del MVP). */
const MAX_OCUPANTES = 800;

/* ---------------- Niveles de entrenamiento ----------------
 * Cada nivel multiplica parámetros del modelo de fuerza social (no cambia
 * la física en sí, solo "cómo se comporta" el ocupante):
 *   - multVelocidad: más rápido = corren.
 *   - multRuido: más ruido = movimiento errático, menos derecho a la salida.
 *   - multRepAgente: repulsión entre personas más débil = se amontonan,
 *     se empujan y toleran más contacto (menos "respeto" del espacio).
 *   - multDensidad: cuánto se autorregula la velocidad al haber vecinos
 *     cerca; más bajo = no se frenan al apretarse, lo que en los cuellos
 *     de botella termina siendo MÁS lento en conjunto (el clásico efecto
 *     "más rápido es más lento" del pánico, de Helbing).
 *   - probCeder: probabilidad de efectivamente ceder el paso al "tocarle".
 *   - fraccionIgnoraEleccion: fracción de ocupantes que jamás reevalúa
 *     cola/caudal entre salidas (siempre van a la más cercana, aunque
 *     esté más colapsada).
 * ------------------------------------------------------------ */
const NIVELES_ENTRENAMIENTO = {
  ninguno: {
    etiqueta: 'Sin entrenamiento (pánico)',
    // Sin ventaja de velocidad real (en pánico "se siente" correr, pero el
    // movimiento errático —multRuido alto— alarga el camino). Bajo
    // amontonamiento el empuje los frena mucho (multDensidad alto), casi no
    // ceden el paso (probCeder bajo) ni lo intentan pronto (multUmbral alto):
    // el atasco de la puerta se sostiene. Casi todos van a la salida más
    // obvia sin mirar la cola. Nunca sale mejor que los otros niveles.
    multVelocidad: 1.0, multRuido: 2.8, multRepAgente: 1.3,
    multDensidad: 2.2, probCeder: 0.05, multUmbral: 2.5, fraccionIgnoraEleccion: 0.9,
  },
  basico: {
    etiqueta: 'Entrenado — nivel básico',
    multVelocidad: 1.0, multRuido: 1.0, multRepAgente: 1.0,
    multDensidad: 1.0, probCeder: 1.0, multUmbral: 1.0, fraccionIgnoraEleccion: 0.3,
  },
  avanzado: {
    etiqueta: 'Entrenado — nivel avanzado',
    // Caminan decididos y derechos (algo más rápidos de base, casi sin ruido),
    // resuelven un atasco antes de que crezca (multUmbral bajo) y mantienen
    // el ritmo al apretarse (multDensidad bajo = fila ordenada, no empujón).
    multVelocidad: 1.08, multRuido: 0.35, multRepAgente: 0.8,
    multDensidad: 0.6, probCeder: 1.0, multUmbral: 0.5, fraccionIgnoraEleccion: 0,
  },
};

/* ---------------- Recorridos (mapa de distancias) ---------------- */

/** @type {number} Lado de la celda del mapa de recorridos (m). */
const CELDA_RUTA = 0.2;
/**
 * @type {number} Una celda queda bloqueada si su centro está a menos de
 * esta distancia de un muro u obstáculo. Debe ser ≥ media diagonal de la
 * celda (0,141 m) para que ningún muro "se escurra" entre celdas libres,
 * y lo bastante chica para que un vano de 1 U.A.S. (0,55 m) quede abierto.
 */
const DESPEJE_RUTA = 0.145;
/** @type {number} Distancia a muros por debajo de la cual el camino se encarece (m). */
const ALCANCE_PENAL_RUTA = 0.45;
/** @type {number} Cada cuánto recalcula su objetivo cada ocupante (s). */
const PERIODO_RUTA = 0.15;
/** @type {number} Celdas de la ruta que mira hacia adelante cada ocupante. */
const PASOS_MIRADA = 14;
/**
 * Elección de salida según la cola. Caudal específico de referencia por
 * metro de ancho (p/s/m, valor de diseño habitual 1,3); cada cuánto se
 * reevalúa (s), qué fracción reevalúa por ronda y cuántos segundos debe
 * ganar como mínimo para cambiar de salida.
 */
const CAUDAL_ESPECIFICO = 1.3;
const PERIODO_ELECCION = 1.0;
const FRACCION_REEVALUA = 0.35;
const HISTERESIS_SALIDA_S = 2.0;
/** @type {number} Tope de celdas del mapa (rendimiento en planos enormes). */
const MAX_CELDAS_RUTA = 2_500_000;

/* ---------------- Obstáculos ---------------- */

/** @type {number} Altura con que se dibujan los obstáculos (m). */
const ALTURA_OBSTACULO = 1.0;
/** @type {number} Lado mínimo de un obstáculo dibujado (m). */
const LADO_MIN_OBSTACULO = 0.2;
/** @type {number} Pasos de deshacer que se guardan. */
const MAX_HISTORIAL = 50;

/* ---------------- SEGU (responsable de evacuación) ---------------- */

/** @type {number} Color de SEGU (naranja de marca PrevenLab). */
const COLOR_SEGU = 0xff8c42;
/** @type {number} Altura del cilindro de SEGU (un poco más alto, para distinguirlo). */
const ALTURA_SEGU = 1.95;
/** @type {number} Radio de SEGU. */
const RADIO_SEGU = 0.27;
/** @type {number} Cada cuánto cambia el consejo que muestra SEGU (s). */
const PERIODO_CONSEJO_SEGU = 7;

/** Consejos que SEGU va mostrando en el HUD mientras dirige la evacuación. */
const CONSEJOS_SEGU = [
  'Caminen, no corran: así se evitan caídas y tapones en la puerta.',
  'Dejen sus cosas. Nada vale más que ustedes.',
  'No usen el ascensor: vayan por la salida marcada.',
  'Si alguien queda trabado cerca tuyo, ayudalo a salir.',
  'Una vez afuera, aléjense del edificio: no vuelvan a entrar.',
  'Si hay humo, agáchense: el aire limpio queda abajo.',
  'Sigan el camino más corto: no se desvíen a buscar otra salida.',
];

/* ---------------- Estado global de la simulación ---------------- */

/**
 * Estado único de la aplicación.
 * `estado`: 'listo' | 'corriendo' | 'pausado' | 'completado'
 */
const sim = {
  /** @type {Plano|null} */    plano: null,
  /** @type {Plano|null} */    planoBase: null,   // copia del plano DXF original
  /** @type {Agente[]} */      agentes: [],
  estado: 'listo',
  tiempo: 0,
  evacuados: 0,
  total: 0,
  /** true = plano fijo (DXF/DWG, escenario guardado o recinto genérico ya editado). */
  modoDXF: false,
  /** Herramienta de edición activa: null | 'salida' | 'obstaculo' | 'borrar'. */
  herramienta: null,
  /** @type {Plano[]} Estados anteriores del plano para "Deshacer" (solo modoDXF). */
  historial: [],
  /** @type {object[]} Ediciones del recinto genérico, en orden (ver regenerarGenerico). */
  edicionesGenerico: [],
  /** @type {MapaRutas|null} Mapa de recorridos del plano vigente. */
  mapa: null,
  /** Ocupantes que no tienen ningún recorrido posible hasta una salida. */
  atrapados: 0,
  /** @type {'ninguno'|'basico'|'avanzado'} Nivel de entrenamiento de los ocupantes. */
  nivelEntrenamiento: 'basico',
};

/* ============================================================
 * GRILLA ESPACIAL (hash uniforme)
 * Acelera la búsqueda de vecinos y de muros cercanos:
 * evita el costo O(n²) del chequeo de pares por fuerza bruta.
 * ============================================================ */

class GrillaEspacial {
  /** @param {number} celda Lado de cada celda en metros. */
  constructor(celda = 1.0) {
    this.celda = celda;
    /** @type {Map<string, Agente[]>} */
    this.agentesPorCelda = new Map();
    /** @type {Map<string, Segmento[]>} */
    this.murosPorCelda = new Map();
  }

  /** @returns {string} Clave de la celda que contiene (x, y). */
  clave(x, y) {
    return `${Math.floor(x / this.celda)},${Math.floor(y / this.celda)}`;
  }

  /** Registra los muros del plano (se hace una sola vez por escenario). */
  registrarMuros(muros) {
    this.murosPorCelda.clear();
    const margen = Math.max(ALCANCE_REP_MURO, this.celda);
    for (const m of muros) {
      const i0 = Math.floor((Math.min(m.x1, m.x2) - margen) / this.celda);
      const i1 = Math.floor((Math.max(m.x1, m.x2) + margen) / this.celda);
      const j0 = Math.floor((Math.min(m.y1, m.y2) - margen) / this.celda);
      const j1 = Math.floor((Math.max(m.y1, m.y2) + margen) / this.celda);
      for (let i = i0; i <= i1; i++) {
        for (let j = j0; j <= j1; j++) {
          const k = `${i},${j}`;
          if (!this.murosPorCelda.has(k)) this.murosPorCelda.set(k, []);
          this.murosPorCelda.get(k).push(m);
        }
      }
    }
  }

  /** Reindexa todos los agentes activos (se llama en cada subpaso). */
  reindexarAgentes(agentes) {
    this.agentesPorCelda.clear();
    for (const a of agentes) {
      if (a.evacuado) continue;
      const k = this.clave(a.x, a.y);
      if (!this.agentesPorCelda.has(k)) this.agentesPorCelda.set(k, []);
      this.agentesPorCelda.get(k).push(a);
    }
  }

  /** @returns {Agente[]} Agentes en las 9 celdas alrededor de (x, y). */
  vecinos(x, y) {
    const ci = Math.floor(x / this.celda), cj = Math.floor(y / this.celda);
    const res = [];
    for (let i = ci - 1; i <= ci + 1; i++) {
      for (let j = cj - 1; j <= cj + 1; j++) {
        const lista = this.agentesPorCelda.get(`${i},${j}`);
        if (lista) res.push(...lista);
      }
    }
    return res;
  }

  /** @returns {Segmento[]} Muros registrados en la celda de (x, y). */
  murosCercanos(x, y) {
    return this.murosPorCelda.get(this.clave(x, y)) ?? [];
  }
}

const grilla = new GrillaEspacial(1.0);

/* ============================================================
 * GEOMETRÍA 2D (utilidades)
 * ============================================================ */

/**
 * Punto del segmento `s` más cercano al punto (px, py).
 * @returns {{x:number, y:number}}
 */
function puntoMasCercano(s, px, py) {
  const dx = s.x2 - s.x1, dy = s.y2 - s.y1;
  const l2 = dx * dx + dy * dy;
  if (l2 === 0) return { x: s.x1, y: s.y1 };
  let t = ((px - s.x1) * dx + (py - s.y1) * dy) / l2;
  t = Math.max(0, Math.min(1, t));
  return { x: s.x1 + t * dx, y: s.y1 + t * dy };
}

/** Orientación con signo del triángulo (a, b, c). */
function orientacion(ax, ay, bx, by, cx, cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/**
 * ¿Se cruzan los segmentos p1→p2 y q1→q2?
 * Se usa para detectar el cruce del umbral de salida (evacuación).
 */
function seCruzan(p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y) {
  const o1 = orientacion(p1x, p1y, p2x, p2y, q1x, q1y);
  const o2 = orientacion(p1x, p1y, p2x, p2y, q2x, q2y);
  const o3 = orientacion(q1x, q1y, q2x, q2y, p1x, p1y);
  const o4 = orientacion(q1x, q1y, q2x, q2y, p2x, p2y);
  return (o1 * o2 < 0) && (o3 * o4 < 0);
}

/**
 * Test punto-en-recinto por paridad de cruces (ray casting hacia +x)
 * contra el contorno muros ∪ salidas. Válido para plantas cerradas.
 */
function puntoInterior(plano, px, py) {
  let cruces = 0;
  const contorno = [...plano.muros, ...plano.salidas];
  for (const s of contorno) {
    const { x1, y1, x2, y2 } = s;
    if ((y1 > py) === (y2 > py)) continue;          // no cruza la horizontal
    const xCruce = x1 + ((py - y1) / (y2 - y1)) * (x2 - x1);
    if (xCruce > px) cruces++;
  }
  return cruces % 2 === 1;
}

/** @returns {Segmento[]} Todos los segmentos de los obstáculos del plano. */
function segmentosObstaculos(plano) {
  return (plano.obstaculos ?? []).flatMap((o) => o.segs);
}

/** @returns {Segmento[]} Muros + obstáculos: todo lo que frena a un ocupante. */
function segmentosColision(plano) {
  return [...plano.muros, ...segmentosObstaculos(plano)];
}

/** ¿El punto (px, py) cae dentro del obstáculo `o`? */
function dentroDeObstaculo(o, px, py) {
  if (o.rect) {
    return px >= o.rect.x0 && px <= o.rect.x1 && py >= o.rect.y0 && py <= o.rect.y1;
  }
  let cruces = 0;   // paridad: válido para polilíneas cerradas del CAD
  for (const { x1, y1, x2, y2 } of o.segs) {
    if ((y1 > py) === (y2 > py)) continue;
    if (x1 + ((py - y1) / (y2 - y1)) * (x2 - x1) > px) cruces++;
  }
  return cruces % 2 === 1;
}

/** Crea un obstáculo rectangular a partir de dos esquinas opuestas. */
function obstaculoRectangular(ax, ay, bx, by) {
  const x0 = Math.min(ax, bx), x1 = Math.max(ax, bx);
  const y0 = Math.min(ay, by), y1 = Math.max(ay, by);
  return {
    rect: { x0, y0, x1, y1 },
    segs: [
      { x1: x0, y1: y0, x2: x1, y2: y0 },
      { x1: x1, y1: y0, x2: x1, y2: y1 },
      { x1: x1, y1: y1, x2: x0, y2: y1 },
      { x1: x0, y1: y1, x2: x0, y2: y0 },
    ],
  };
}

/* ============================================================
 * MAPA DE RECORRIDOS (pathfinding por mapa de distancias)
 * ------------------------------------------------------------
 * El plano se divide en celdas de 0,20 m. Se bloquean las celdas
 * pegadas a muros y obstáculos y se calcula, con Dijkstra desde
 * todas las salidas a la vez, la distancia a pie de cada celda a
 * la salida más cercana POR RECORRIDO REAL (no en línea recta).
 * Cada ocupante sigue esa ruta y apunta al punto más lejano de
 * ella que tiene a la vista, así rodea muebles, tabiques y pasa
 * por las puertas interiores. El último tramo hacia la salida es
 * igual al del modelo original (apunta al centro del vano).
 * ============================================================ */

class MapaRutas {
  /**
   * @param {Plano} plano Plano vigente (muros, obstáculos y salidas).
   */
  constructor(plano) {
    const margen = 2;
    const area = (plano.bbox.ancho + 2 * margen) * (plano.bbox.alto + 2 * margen);
    // Un mapa por salida: en planos grandes con muchas salidas se agranda la
    // celda para no pasar de ~8 millones de celdas en total.
    const maxCeldas = Math.min(MAX_CELDAS_RUTA, 8_000_000 / Math.max(1, plano.salidas.length));
    this.celda = Math.max(CELDA_RUTA, Math.sqrt(area / maxCeldas));
    this.aproximado = this.celda > CELDA_RUTA;
    this.despeje = DESPEJE_RUTA * (this.celda / CELDA_RUTA);
    this.ox = -margen;
    this.oy = -margen;
    this.nx = Math.ceil((plano.bbox.ancho + 2 * margen) / this.celda);
    this.ny = Math.ceil((plano.bbox.alto + 2 * margen) / this.celda);
    const n = this.nx * this.ny;

    /** Distancia (m) del centro de cada celda al muro/obstáculo más cercano. */
    this.despejeCelda = new Float32Array(n).fill(99);
    /** 1 = celda intransitable. */
    this.bloq = new Uint8Array(n);
    /** Distancia a pie hasta la salida más cercana (m). */
    this.dist = new Float64Array(n).fill(Infinity);
    /** Siguiente celda del recorrido (-1 = es salida o no tiene camino). */
    this.sig = new Int32Array(n).fill(-1);
    /** Índice de la salida a la que lleva el recorrido de cada celda. */
    this.salidaDe = new Int16Array(n).fill(-1);

    this.#rasterizar(segmentosColision(plano));
    this.#propagar(plano.salidas);
  }

  /** Índice lineal de la celda (i, j). */
  idx(i, j) { return j * this.nx + i; }

  /** Centro de la celda k en coordenadas de plano. */
  centro(k) {
    const i = k % this.nx, j = (k - i) / this.nx;
    return { x: this.ox + (i + 0.5) * this.celda, y: this.oy + (j + 0.5) * this.celda };
  }

  /** @returns {number} Celda que contiene (x, y), o -1 si cae fuera del mapa. */
  celdaDe(x, y) {
    const i = Math.floor((x - this.ox) / this.celda);
    const j = Math.floor((y - this.oy) / this.celda);
    if (i < 0 || j < 0 || i >= this.nx || j >= this.ny) return -1;
    return this.idx(i, j);
  }

  /** Marca el despeje de cada celda respecto de todos los segmentos. */
  #rasterizar(segs) {
    const c = this.celda, alcance = Math.max(ALCANCE_PENAL_RUTA, this.despeje);
    for (const s of segs) {
      const i0 = Math.max(0, Math.floor((Math.min(s.x1, s.x2) - alcance - this.ox) / c));
      const i1 = Math.min(this.nx - 1, Math.floor((Math.max(s.x1, s.x2) + alcance - this.ox) / c));
      const j0 = Math.max(0, Math.floor((Math.min(s.y1, s.y2) - alcance - this.oy) / c));
      const j1 = Math.min(this.ny - 1, Math.floor((Math.max(s.y1, s.y2) + alcance - this.oy) / c));
      for (let j = j0; j <= j1; j++) {
        const py = this.oy + (j + 0.5) * c;
        for (let i = i0; i <= i1; i++) {
          const px = this.ox + (i + 0.5) * c;
          const q = puntoMasCercano(s, px, py);
          const d = Math.hypot(px - q.x, py - q.y);
          const k = this.idx(i, j);
          if (d < this.despejeCelda[k]) this.despejeCelda[k] = d;
        }
      }
    }
    for (let k = 0; k < this.bloq.length; k++) {
      this.bloq[k] = this.despejeCelda[k] < this.despeje ? 1 : 0;
    }
  }

  /**
   * Dijkstra desde cada salida por separado: deja un mapa de distancias y
   * de "siguiente celda" por salida (para elegir salida según la cola) y
   * el mapa combinado de la salida más cercana.
   */
  #propagar(salidas) {
    const { nx, ny, bloq, dist, sig, salidaDe } = this;
    const n = nx * ny;
    const radioSemilla = Math.max(0.25, this.celda * 1.25);
    /** @type {Float64Array[]} */ this.distE = [];
    /** @type {Int32Array[]} */ this.sigE = [];

    for (const [e, s] of salidas.entries()) {
      // Float64: con Float32 el redondeo hace que Dijkstra descarte celdas.
      const dE = new Float64Array(n).fill(Infinity);
      const sE = new Int32Array(n).fill(-1);
      const heap = new MonticuloMin();
      const c = this.celda;
      const i0 = Math.max(0, Math.floor((Math.min(s.x1, s.x2) - radioSemilla - this.ox) / c));
      const i1 = Math.min(nx - 1, Math.floor((Math.max(s.x1, s.x2) + radioSemilla - this.ox) / c));
      const j0 = Math.max(0, Math.floor((Math.min(s.y1, s.y2) - radioSemilla - this.oy) / c));
      const j1 = Math.min(ny - 1, Math.floor((Math.max(s.y1, s.y2) + radioSemilla - this.oy) / c));
      for (let j = j0; j <= j1; j++) {
        for (let i = i0; i <= i1; i++) {
          const k = this.idx(i, j);
          if (bloq[k]) continue;
          const p = this.centro(k);
          const q = puntoMasCercano(s, p.x, p.y);
          if (Math.hypot(p.x - q.x, p.y - q.y) > radioSemilla) continue;
          dE[k] = 0;
          heap.push(k, 0);
        }
      }
      this.#dijkstra(heap, dE, sE);
      this.distE.push(dE);
      this.sigE.push(sE);
      for (let k = 0; k < n; k++) {
        if (dE[k] < dist[k]) { dist[k] = dE[k]; sig[k] = sE[k]; salidaDe[k] = e; }
      }
    }
  }

  /** Expande el montículo sobre la grilla (8 vecinos, sin cortar esquinas). */
  #dijkstra(heap, dist, sig) {
    const { nx, ny, bloq } = this;
    const VEC = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];
    while (heap.size) {
      const [k, dk] = heap.pop();
      if (dk > dist[k]) continue;   // entrada vieja del montículo
      const i = k % nx, j = (k - i) / nx;
      for (const [di, dj] of VEC) {
        const ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
        const n = nj * nx + ni;
        if (bloq[n]) continue;
        const diagonal = di !== 0 && dj !== 0;
        // Sin "cortar esquinas": en diagonal, ambas celdas laterales libres.
        if (diagonal && (bloq[j * nx + ni] || bloq[nj * nx + i])) continue;
        // Caminar pegado a muros cuesta más: los recorridos van por el medio.
        const desp = this.despejeCelda[n];
        const penal = desp < ALCANCE_PENAL_RUTA ? 1 + 1.5 * (ALCANCE_PENAL_RUTA - desp) / ALCANCE_PENAL_RUTA : 1;
        const nd = dk + this.celda * (diagonal ? Math.SQRT2 : 1) * penal;
        if (nd < dist[n]) {
          dist[n] = nd;
          sig[n] = k;
          heap.push(n, nd);
        }
      }
    }
  }

  /**
   * Celda transitable con recorrido para un ocupante en (x, y). Si el
   * centro de su celda quedó bloqueado (está pegado a un muro), se usa la
   * mejor celda vecina. @returns {number} -1 si no hay recorrido posible.
   */
  celdaUtil(x, y) {
    const k = this.celdaDe(x, y);
    if (k < 0) return -1;
    if (!this.bloq[k] && this.dist[k] < Infinity) return k;
    const i = k % this.nx, j = (k - i) / this.nx;
    let mejor = -1, mejorD = Infinity;
    for (let r = 1; r <= 3 && mejor < 0; r++) {
      for (let dj = -r; dj <= r; dj++) {
        for (let di = -r; di <= r; di++) {
          const ni = i + di, nj = j + dj;
          if (ni < 0 || nj < 0 || ni >= this.nx || nj >= this.ny) continue;
          const n = this.idx(ni, nj);
          if (!this.bloq[n] && this.dist[n] < mejorD) { mejorD = this.dist[n]; mejor = n; }
        }
      }
    }
    return mejor;
  }

  /** ¿Hay línea libre (sin celdas bloqueadas) entre dos puntos? */
  visible(x0, y0, x1, y1) {
    const L = Math.hypot(x1 - x0, y1 - y0);
    const pasos = Math.ceil(L / (this.celda * 0.5));
    for (let s = 1; s <= pasos; s++) {
      const t = s / pasos;
      if (t * L < 0.2) continue;   // el ocupante puede estar rozando un muro
      const k = this.celdaDe(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t);
      if (k < 0 || this.bloq[k]) return false;
    }
    return true;
  }
}

/** Montículo binario mínimo (cola de prioridad para Dijkstra). */
class MonticuloMin {
  constructor() { this.k = []; this.p = []; }
  get size() { return this.k.length; }
  push(k, p) {
    const K = this.k, P = this.p;
    let i = K.length;
    K.push(k); P.push(p);
    while (i > 0) {
      const m = (i - 1) >> 1;
      if (P[m] <= p) break;
      K[i] = K[m]; P[i] = P[m]; i = m;
    }
    K[i] = k; P[i] = p;
  }
  pop() {
    const K = this.k, P = this.p;
    const k0 = K[0], p0 = P[0];
    const kl = K.pop(), pl = P.pop();
    const n = K.length;
    if (n > 0) {
      let i = 0;
      for (;;) {
        let h = 2 * i + 1;
        if (h >= n) break;
        if (h + 1 < n && P[h + 1] < P[h]) h++;
        if (P[h] >= pl) break;
        K[i] = K[h]; P[i] = P[h]; i = h;
      }
      K[i] = kl; P[i] = pl;
    }
    return [k0, p0];
  }
}

/* ============================================================
 * PLANO GENÉRICO (recinto cuadrado hardcodeado)
 * ============================================================ */

/**
 * Genera un recinto cuadrado cuyo lado surge de la superficie,
 * con una única salida centrada en el muro sur de `uas` × 0,55 m.
 * @param {number} superficie m² del sector de incendio.
 * @param {number} uas Unidades de ancho de salida provistas.
 * @returns {Plano}
 */
/**
 * Genera un recinto rectangular de `anchoX` × `anchoY` m, con una única
 * salida centrada en el muro sur de `uas` × 0,55 m.
 * @param {number} anchoX Ancho (eje x) en metros.
 * @param {number} anchoY Profundidad (eje y) en metros.
 * @param {number} uas Unidades de ancho de salida provistas.
 * @returns {Plano}
 */
function crearPlanoDemo(anchoX, anchoY, uas) {
  const lx = Math.max(4, anchoX), ly = Math.max(4, anchoY);
  const anchoSalida = Math.min(uas * UAS_METROS, lx - 1);
  const c = lx / 2, h = anchoSalida / 2;

  /** @type {Segmento[]} */
  const muros = [
    { x1: 0, y1: ly, x2: lx, y2: ly },           // norte
    { x1: 0, y1: 0, x2: 0, y2: ly },              // oeste
    { x1: lx, y1: 0, x2: lx, y2: ly },            // este
    { x1: 0, y1: 0, x2: c - h, y2: 0 },           // sur (tramo izquierdo)
    { x1: c + h, y1: 0, x2: lx, y2: 0 },          // sur (tramo derecho)
  ];

  /** @type {Salida[]} */
  const salidas = [
    { x1: c - h, y1: 0, x2: c + h, y2: 0, ancho: anchoSalida, cx: c, cy: 0 },
  ];

  return {
    muros, salidas, obstaculos: [],
    bbox: { ancho: lx, alto: ly },
    nombre: `Recinto genérico ${lx.toFixed(1)} × ${ly.toFixed(1)} m`,
    superficie: lx * ly,
  };
}

/* ============================================================
 * ESCENA THREE.JS
 * ============================================================ */

const contenedor = document.getElementById('contenedor3d');
const escena = new THREE.Scene();
escena.background = new THREE.Color(0x0e131b);
escena.fog = new THREE.Fog(0x0e131b, 60, 160);

const camara = new THREE.PerspectiveCamera(55, 1, 0.1, 500);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
contenedor.appendChild(renderer.domElement);

const controles = new OrbitControls(camara, renderer.domElement);
controles.enableDamping = true;
controles.maxPolarAngle = Math.PI / 2.05;   // no mirar desde abajo del piso

/* Iluminación: hemisférica (ambiente) + direccional (volumen). */
escena.add(new THREE.HemisphereLight(0xdfe8ff, 0x30281e, 1.05));
const luzDir = new THREE.DirectionalLight(0xffffff, 1.4);
luzDir.position.set(30, 50, 20);
escena.add(luzDir);

/** Grupo con la geometría del escenario vigente (se reconstruye entero). */
let grupoEscenario = new THREE.Group();
escena.add(grupoEscenario);

/** Materiales compartidos. */
const MAT_MURO = new THREE.MeshLambertMaterial({ color: 0x8fa3bd });
const MAT_PISO = new THREE.MeshLambertMaterial({ color: 0x232d3d });
const MAT_SALIDA = new THREE.MeshBasicMaterial({
  color: 0x37c871, transparent: true, opacity: 0.35, side: THREE.DoubleSide,
});
const MAT_SALIDA_PISO = new THREE.MeshBasicMaterial({
  color: 0x37c871, transparent: true, opacity: 0.8, side: THREE.DoubleSide,
});
const MAT_OBSTACULO = new THREE.MeshLambertMaterial({ color: 0xc8843c });
const MAT_BORRADOR = new THREE.MeshBasicMaterial({
  color: 0xffb259, transparent: true, opacity: 0.45, depthWrite: false,
});
const COLOR_ATRAPADO = 0xe5484d;
const COLOR_CEDIENDO = 0x4ec9e0;   // indicador visual: cediendo el paso
const GEO_AGENTE = new THREE.CylinderGeometry(RADIO_AGENTE, RADIO_AGENTE, ALTURA_AGENTE, 12);
const GEO_SEGU = new THREE.CylinderGeometry(RADIO_SEGU, RADIO_SEGU, ALTURA_SEGU, 12);

/** Crea el cartelito "SEGU" (sprite, siempre de frente a cámara) sobre su cabeza. */
function crearEtiquetaSegu() {
  const canvas = document.createElement('canvas');
  canvas.width = 160; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = 'rgba(20, 14, 6, 0.85)';
  ctx.beginPath();
  ctx.roundRect ? ctx.roundRect(2, 2, 156, 60, 12) : ctx.rect(2, 2, 156, 60);
  ctx.fill();
  ctx.fillStyle = '#ff8c42';
  ctx.font = 'bold 34px "Segoe UI", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('SEGU', 80, 34);
  const textura = new THREE.CanvasTexture(canvas);
  const material = new THREE.SpriteMaterial({ map: textura, depthTest: false });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(1.3, 0.52, 1);
  sprite.renderOrder = 10;
  return sprite;
}

/** Libera geometrías/materiales propios de un grupo y lo vacía. */
function limpiarGrupo(grupo) {
  grupo.traverse((obj) => {
    if (obj.isMesh) {
      obj.geometry?.dispose();
      if (obj.material?.userData?.propio) obj.material.dispose();
    }
  });
  grupo.clear();
}

/**
 * Construye el entorno 3D a partir del plano vigente:
 * piso, muros extruidos y marcadores de salida.
 */
function construirEscenario(mantenerCamara = false) {
  limpiarGrupo(grupoEscenario);
  const p = sim.plano;

  /* Piso: plano con margen alrededor del bounding box. */
  const margen = 4;
  const piso = new THREE.Mesh(
    new THREE.PlaneGeometry(p.bbox.ancho + 2 * margen, p.bbox.alto + 2 * margen),
    MAT_PISO,
  );
  piso.rotation.x = -Math.PI / 2;
  piso.position.set(p.bbox.ancho / 2, -0.02, -p.bbox.alto / 2);
  grupoEscenario.add(piso);

  const rejilla = new THREE.GridHelper(
    Math.max(p.bbox.ancho, p.bbox.alto) + 2 * margen, // tamaño
    Math.round(Math.max(p.bbox.ancho, p.bbox.alto)) + 2 * margen, // 1 división ≈ 1 m
    0x33415a, 0x27303f,
  );
  rejilla.position.set(p.bbox.ancho / 2, -0.01, -p.bbox.alto / 2);
  grupoEscenario.add(rejilla);

  /* Muros: una caja extruida por segmento (largo × alto × espesor). */
  for (const m of p.muros) {
    const dx = m.x2 - m.x1, dy = m.y2 - m.y1;
    const largo = Math.hypot(dx, dy);
    if (largo < 0.02) continue;
    const caja = new THREE.Mesh(
      new THREE.BoxGeometry(largo, ALTURA_MURO, ESPESOR_MURO), MAT_MURO,
    );
    caja.position.set((m.x1 + m.x2) / 2, ALTURA_MURO / 2, -(m.y1 + m.y2) / 2);
    caja.rotation.y = Math.atan2(dy, dx);
    grupoEscenario.add(caja);
  }

  /* Salidas: portal translúcido verde + franja en el piso. */
  for (const s of p.salidas) {
    const dx = s.x2 - s.x1, dy = s.y2 - s.y1;
    const angulo = Math.atan2(dy, dx);

    const portal = new THREE.Mesh(
      new THREE.PlaneGeometry(s.ancho, ALTURA_MURO), MAT_SALIDA,
    );
    portal.position.set(s.cx, ALTURA_MURO / 2, -s.cy);
    portal.rotation.y = angulo;
    grupoEscenario.add(portal);

    const franja = new THREE.Mesh(
      new THREE.PlaneGeometry(s.ancho, 0.6), MAT_SALIDA_PISO,
    );
    franja.rotation.x = -Math.PI / 2;
    franja.rotation.z = angulo;
    franja.position.set(s.cx, 0.005, -s.cy);
    grupoEscenario.add(franja);
  }

  /* Obstáculos: caja baja (rectángulos) o tabique bajo por segmento (CAD). */
  for (const o of p.obstaculos ?? []) {
    if (o.rect) {
      const { x0, y0, x1, y1 } = o.rect;
      const caja = new THREE.Mesh(
        new THREE.BoxGeometry(x1 - x0, ALTURA_OBSTACULO, y1 - y0), MAT_OBSTACULO,
      );
      caja.position.set((x0 + x1) / 2, ALTURA_OBSTACULO / 2, -(y0 + y1) / 2);
      grupoEscenario.add(caja);
      continue;
    }
    for (const m of o.segs) {
      const dx = m.x2 - m.x1, dy = m.y2 - m.y1;
      const largo = Math.hypot(dx, dy);
      if (largo < 0.02) continue;
      const tramo = new THREE.Mesh(
        new THREE.BoxGeometry(largo, ALTURA_OBSTACULO, 0.12), MAT_OBSTACULO,
      );
      tramo.position.set((m.x1 + m.x2) / 2, ALTURA_OBSTACULO / 2, -(m.y1 + m.y2) / 2);
      tramo.rotation.y = Math.atan2(dy, dx);
      grupoEscenario.add(tramo);
    }
  }

  if (!mantenerCamara) encuadrarCamara();
}

/** Posiciona la cámara para encuadrar el bounding box del plano. */
function encuadrarCamara() {
  const p = sim.plano;
  const tam = Math.max(p.bbox.ancho, p.bbox.alto);
  const cx = p.bbox.ancho / 2, cz = -p.bbox.alto / 2;
  camara.position.set(cx, tam * 1.05 + 6, cz + tam * 0.95 + 6);
  controles.target.set(cx, 0, cz);
  controles.update();
}

/** Ajusta el tamaño del render al contenedor (si ya tiene layout). */
function ajustarTamano() {
  const w = contenedor.clientWidth, h = contenedor.clientHeight;
  if (w === 0 || h === 0) return;   // el contenedor todavía no tiene tamaño
  camara.aspect = w / h;
  camara.updateProjectionMatrix();
  renderer.setSize(w, h);
}
window.addEventListener('resize', ajustarTamano);
// ResizeObserver cubre el caso en que el contenedor obtiene tamaño después
// de la carga (p. ej. paneles de preview) sin disparar `resize` de window.
new ResizeObserver(ajustarTamano).observe(contenedor);

/* ============================================================
 * AGENTES (ocupantes)
 * ============================================================ */

/** Elimina los agentes actuales (y la etiqueta de SEGU) de la escena. */
function eliminarAgentes() {
  for (const a of sim.agentes) {
    escena.remove(a.mesh);
    a.mesh.material.dispose();
    if (a.etiqueta) {
      escena.remove(a.etiqueta);
      a.etiqueta.material.map?.dispose();
      a.etiqueta.material.dispose();
    }
  }
  sim.agentes = [];
  sim.segu = null;
}

/**
 * Busca una posición aleatoria válida del recinto: dentro del contorno,
 * lejos de muros/obstáculos y sin solaparse con los agentes ya puestos.
 * @returns {{x:number,y:number}|null} null si no encontró lugar.
 */
function posicionAgenteValida(p, bordes) {
  for (let intento = 0; intento < 300; intento++) {
    const x = Math.random() * p.bbox.ancho, y = Math.random() * p.bbox.alto;
    if (!puntoInterior(p, x, y)) continue;
    if ((p.obstaculos ?? []).some((o) => dentroDeObstaculo(o, x, y))) continue;

    let valida = true;
    for (const m of bordes) {                        // lejos de muros y obstáculos
      const q = puntoMasCercano(m, x, y);
      if (Math.hypot(x - q.x, y - q.y) < RADIO_AGENTE + 0.20) { valida = false; break; }
    }
    if (!valida) continue;
    for (const b of sim.agentes) {                    // sin solaparse
      if (Math.hypot(x - b.x, y - b.y) < 2 * RADIO_AGENTE + 0.05) { valida = false; break; }
    }
    if (valida) return { x, y };
  }
  return null;   // recinto saturado: no hay lugar
}

/**
 * Crea un agente (ocupante normal o SEGU) en (x, y) y lo agrega a la
 * escena y a `sim.agentes`.
 * @param {number} id @param {number} x @param {number} y
 * @param {boolean} [esSegu=false]
 */
function crearAgente(id, x, y, esSegu = false) {
  // SEGU siempre se comporta como "avanzado" (personal entrenado), más allá
  // del nivel elegido para el resto de los ocupantes.
  const cfg = esSegu
    ? NIVELES_ENTRENAMIENTO.avanzado
    : NIVELES_ENTRENAMIENTO[sim.nivelEntrenamiento] ?? NIVELES_ENTRENAMIENTO.basico;

  const material = new THREE.MeshLambertMaterial({
    color: esSegu
      ? COLOR_SEGU
      : new THREE.Color().setHSL(0.07 + Math.random() * 0.06, 0.85, 0.55),
  });
  material.userData.propio = true;
  const mesh = new THREE.Mesh(esSegu ? GEO_SEGU : GEO_AGENTE, material);
  mesh.position.set(x, (esSegu ? ALTURA_SEGU : ALTURA_AGENTE) / 2, -y);
  escena.add(mesh);

  let etiqueta = null;
  if (esSegu) {
    etiqueta = crearEtiquetaSegu();
    etiqueta.position.set(x, ALTURA_SEGU + 0.45, -y);
    escena.add(etiqueta);
  }

  const agente = {
    id, x, y, vx: 0, vy: 0, prevX: x, prevY: y,
    velDeseada: (1.1 + Math.random() * 0.4) * cfg.multVelocidad,   // 1,1 – 1,5 m/s de base
    evacuado: false, mesh,
    // Estado del mecanismo de "ceder el paso":
    tAtascado: 0,                                            // s detenido
    cederHasta: 0,                                           // instante fin de retroceso
    umbralAtasco: (CEDER_UMBRAL_MIN + Math.random() * CEDER_UMBRAL_VAR) * cfg.multUmbral,
    cediendoVisual: false,   // color actual refleja "cediendo el paso"
    // Recorrido: punto al que apunta y cuándo recalcularlo.
    objX: x, objY: y, tRuta: 0, atrapado: false,
    salida: -1, celdaEleccion: -1,   // salida elegida (índice) y celda al elegir
    esSegu, etiqueta,
    esperando: esSegu,   // SEGU: da instrucciones hasta que salgan los demás
    // Nivel de entrenamiento (fijado al crear el ocupante, no cambia en la corrida):
    multRuido: cfg.multRuido, multRepAgente: cfg.multRepAgente,
    multDensidad: cfg.multDensidad, probCeder: cfg.probCeder, multUmbral: cfg.multUmbral,
    optimizaSalida: Math.random() >= cfg.fraccionIgnoraEleccion,
  };
  sim.agentes.push(agente);
  return agente;
}

/**
 * Instancia `n` ocupantes en posiciones aleatorias válidas del recinto,
 * más SEGU: el responsable de evacuación, que da instrucciones y es el
 * último en salir.
 * @param {number} n Cantidad de ocupantes a distribuir.
 */
function crearAgentes(n) {
  eliminarAgentes();
  const p = sim.plano;
  const bordes = segmentosColision(p);

  for (let i = 0; i < n; i++) {
    const pos = posicionAgenteValida(p, bordes);
    if (!pos) break;   // recinto saturado: se instancian menos agentes
    crearAgente(i, pos.x, pos.y);
  }
  sim.total = sim.agentes.length;
  sim.evacuados = 0;

  sim.segu = null;
  const posSegu = posicionAgenteValida(p, bordes);
  if (posSegu) sim.segu = crearAgente(-1, posSegu.x, posSegu.y, true);

  // Ocupantes sin ningún recorrido posible hasta una salida (p. ej. en un
  // local cerrado o encerrados por obstáculos): se marcan en rojo.
  sim.atrapados = 0;
  if (sim.mapa && p.salidas.length) {
    for (const a of sim.agentes) {
      if (sim.mapa.celdaUtil(a.x, a.y) < 0) {
        a.atrapado = true;
        if (!a.esSegu) a.mesh.material.color.setHex(COLOR_ATRAPADO);
        if (!a.esSegu) sim.atrapados++;
      }
    }
  }
  sim.seguMensajeIdx = Math.floor(Math.random() * CONSEJOS_SEGU.length);
  sim.seguMensajeHasta = 0;
}

/* ============================================================
 * MOTOR DE FÍSICA (paso de simulación)
 * ============================================================ */

/** @returns {Salida} La salida más cercana al punto (x, y). */
function salidaMasCercana(x, y) {
  const salidas = sim.plano.salidas;
  let mejor = salidas[0], mejorD = Infinity;
  for (const s of salidas) {
    const d = (s.cx - x) ** 2 + (s.cy - y) ** 2;
    if (d < mejorD) { mejorD = d; mejor = s; }
  }
  return mejor;
}

/**
 * Elección de salida según la cola (cada ~1 s de simulación): cada
 * ocupante estima cuánto tardaría por cada salida como el mayor entre
 * (a) lo que tarda en llegar caminando y (b) lo que tarda en pasar la
 * gente asignada a esa salida que está más cerca que él (cola / caudal).
 * Si otra salida le conviene claramente, cambia. Solo una fracción
 * reevalúa en cada ronda, con histéresis, para que no oscilen todos juntos.
 */
function reasignarSalidas() {
  const mapa = sim.mapa, salidas = sim.plano.salidas;
  if (!mapa || salidas.length < 2) return;
  const activos = [];
  for (const a of sim.agentes) {
    if (a.evacuado || a.atrapado) continue;
    const c = mapa.celdaUtil(a.x, a.y);
    if (c < 0) continue;
    a.celdaEleccion = c;
    if (!(a.salida >= 0 && mapa.distE[a.salida][c] < Infinity)) a.salida = mapa.salidaDe[c];
    activos.push(a);
  }
  /** Distancias (ordenadas) de los ocupantes asignados a cada salida. */
  const colas = salidas.map(() => []);
  for (const a of activos) colas[a.salida].push(mapa.distE[a.salida][a.celdaEleccion]);
  for (const l of colas) l.sort((x, y) => x - y);
  const caudal = salidas.map((s) => Math.max(s.ancho, UAS_METROS) * CAUDAL_ESPECIFICO);
  /** Cantidad de valores de la lista ordenada menores que d. */
  const delanteDe = (l, d) => {
    let lo = 0, hi = l.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (l[m] < d) lo = m + 1; else hi = m; }
    return lo;
  };

  for (const a of activos) {
    // Sin entrenamiento, buena parte nunca reevalúa: va siempre a la salida
    // más cercana aunque esté más colapsada que otra (ver optimizaSalida).
    if (!a.optimizaSalida || Math.random() > FRACCION_REEVALUA) continue;
    const c = a.celdaEleccion;
    const costo = (e) => {
      const d = mapa.distE[e][c];
      if (!(d < Infinity)) return Infinity;
      return Math.max(d / a.velDeseada, delanteDe(colas[e], d) / caudal[e]);
    };
    const actual = costo(a.salida);
    let mejor = a.salida, mejorC = actual;
    for (let e = 0; e < salidas.length; e++) {
      if (e === a.salida) continue;
      const ce = costo(e);
      if (ce < mejorC) { mejorC = ce; mejor = e; }
    }
    if (mejor === a.salida || mejorC > actual - Math.max(HISTERESIS_SALIDA_S, 0.1 * actual)) continue;
    // Pasar al ocupante de una cola a la otra.
    const lv = colas[a.salida], dv = mapa.distE[a.salida][c];
    lv.splice(Math.min(delanteDe(lv, dv), lv.length - 1), 1);
    const ln = colas[mejor], dn = mapa.distE[mejor][c];
    ln.splice(delanteDe(ln, dn), 0, dn);
    a.salida = mejor;
    a.tRuta = 0;   // recalcular el recorrido ya
  }
}

/**
 * Recalcula el punto al que camina el ocupante siguiendo el mapa de
 * recorridos: si ve el centro de su salida va directo; si no, avanza
 * hacia el punto más lejano que tiene a la vista de su ruta.
 * Sin mapa o sin recorrido posible: línea recta a la salida más cercana.
 */
function actualizarObjetivo(a) {
  a.tRuta = sim.tiempo + PERIODO_RUTA * (0.8 + Math.random() * 0.4);
  if (a.esSegu && a.esperando) { objetivoSeguEsperando(a); return; }
  const mapa = sim.mapa;
  const c = mapa ? mapa.celdaUtil(a.x, a.y) : -1;
  if (c < 0) {
    const s = salidaMasCercana(a.x, a.y);
    a.objX = s.cx; a.objY = s.cy;
    return;
  }
  // Salida elegida por el ocupante (si desde ahí tiene recorrido) o la más cercana.
  if (!(a.salida >= 0 && mapa.distE[a.salida][c] < Infinity)) a.salida = mapa.salidaDe[c];
  const sig = mapa.sigE[a.salida];
  const s = sim.plano.salidas[a.salida];
  if (s && mapa.visible(a.x, a.y, s.cx, s.cy)) {
    a.objX = s.cx; a.objY = s.cy;
    return;
  }
  let k = c, mejor = sig[c] >= 0 ? sig[c] : c;
  for (let paso = 0; paso < PASOS_MIRADA; paso++) {
    const n = sig[k];
    if (n < 0) break;
    k = n;
    const q = mapa.centro(k);
    if (mapa.visible(a.x, a.y, q.x, q.y)) mejor = k;
  }
  const q = mapa.centro(mejor);
  a.objX = q.x; a.objY = q.y;
}

/**
 * Mientras alguien más sigue sin evacuar, SEGU no se va: recorre el
 * centro de masa de los ocupantes que quedan (como quien va recorriendo
 * el sector y dando indicaciones) en vez de ir directo a una salida.
 * Cuando ya no queda nadie más por evacuar (o quedaron atrapados), deja
 * de esperar y sale como cualquier ocupante.
 */
function objetivoSeguEsperando(a) {
  const activos = sim.agentes.filter((o) => !o.esSegu && !o.evacuado && !o.atrapado);
  if (activos.length === 0) {
    a.esperando = false;
    actualizarObjetivo(a);
    return;
  }
  let cx = 0, cy = 0;
  for (const o of activos) { cx += o.x; cy += o.y; }
  a.objX = cx / activos.length;
  a.objY = cy / activos.length;
}

/**
 * Un subpaso de integración del modelo de fuerza social.
 * @param {number} dt Paso de tiempo (s).
 */
function pasoFisica(dt) {
  sim.tiempo += dt;   // reloj de la simulación (cronómetro del HUD)
  if (sim.tiempo >= sim.tEleccion) {
    reasignarSalidas();
    sim.tEleccion = sim.tiempo + PERIODO_ELECCION;
  }
  grilla.reindexarAgentes(sim.agentes);

  /* --- 1) Fuerzas e integración de velocidad/posición --- */
  for (const a of sim.agentes) {
    if (a.evacuado) continue;
    a.prevX = a.x; a.prevY = a.y;

    // Atracción: velocidad deseada siguiendo el recorrido hacia la salida.
    if (sim.tiempo >= a.tRuta) actualizarObjetivo(a);
    let dx = a.objX - a.x, dy = a.objY - a.y;
    const dObj = Math.hypot(dx, dy) || 1e-9;
    dx /= dObj; dy /= dObj;

    // "Ceder el paso": tras quedar atascado un tiempo, retroceder ~1 s
    // para deshacer la cuña formada con otro agente en el vano.
    const rapidez2 = a.vx * a.vx + a.vy * a.vy;
    const cediendo = sim.tiempo < a.cederHasta;
    if (cediendo !== a.cediendoVisual && !a.esSegu && !a.atrapado) {
      // Indicador visual: mientras retrocede para "ceder el paso" se ve
      // celeste un instante; ayuda a entender el mecanismo en el visor.
      a.cediendoVisual = cediendo;
      a.mesh.material.color.set(cediendo
        ? COLOR_CEDIENDO
        : new THREE.Color().setHSL(0.07 + Math.random() * 0.06, 0.85, 0.55));
    }
    if (cediendo) {
      dx = -dx; dy = -dy;                       // se aparta de la salida
    } else {
      a.tAtascado = rapidez2 < VEL_ATASCO * VEL_ATASCO ? a.tAtascado + dt : 0;
      if (a.tAtascado > a.umbralAtasco) {
        // Nivel de entrenamiento: probCeder gobierna si REALMENTE cede el
        // paso (sin entrenamiento, casi nunca — sigue empujando y se arma
        // el amontonamiento) o solo reinicia el intento.
        if (Math.random() < a.probCeder) {
          a.cederHasta = sim.tiempo + CEDER_DURACION_MIN + Math.random() * CEDER_DURACION_VAR;
        }
        a.tAtascado = 0;
        a.umbralAtasco = (CEDER_UMBRAL_MIN + Math.random() * CEDER_UMBRAL_VAR) * a.multUmbral;
      }
    }
    // Repulsión entre agentes (solo vecinos de la grilla espacial),
    // contando de paso los vecinos próximos para el frenado por densidad.
    let fRepX = 0, fRepY = 0, vecinosCerca = 0;
    for (const b of grilla.vecinos(a.x, a.y)) {
      if (b === a) continue;
      const sx = a.x - b.x, sy = a.y - b.y;
      const d = Math.hypot(sx, sy) || 1e-9;
      if (d < DENSIDAD_RADIO) vecinosCerca++;
      if (d > ALCANCE_REP_AGENTE) continue;
      const f = REP_AG_A * a.multRepAgente * Math.exp((2 * RADIO_AGENTE - d) / REP_AG_B);
      fRepX += (sx / d) * f;
      fRepY += (sy / d) * f;
    }

    // Velocidad deseada efectiva: cede el paso + frenado por densidad
    // (según nivel de entrenamiento: sin entrenar casi no se autorregulan,
    // lo que en los cuellos de botella termina siendo más lento en total).
    const factorDensidad = 1 / (1 + DENSIDAD_COEF * a.multDensidad * Math.max(0, vecinosCerca - 1));
    let impulso = a.velDeseada * factorDensidad;
    if (sim.tiempo < a.cederHasta) impulso *= 0.4;
    let fx = (dx * impulso - a.vx) / TAU + fRepX;
    let fy = (dy * impulso - a.vy) / TAU + fRepY;

    // Repulsión de muros cercanos.
    for (const m of grilla.murosCercanos(a.x, a.y)) {
      const q = puntoMasCercano(m, a.x, a.y);
      const sx = a.x - q.x, sy = a.y - q.y;
      const d = Math.hypot(sx, sy) || 1e-9;
      if (d > ALCANCE_REP_MURO) continue;
      const f = REP_MU_A * Math.exp((RADIO_AGENTE - d) / REP_MU_B);
      fx += (sx / d) * f;
      fy += (sy / d) * f;
    }

    // Fluctuación aleatoria (ξ): evita equilibrios perfectos en cuellos
    // de botella. Si el agente quedó casi estático, se amplifica. El
    // nivel de entrenamiento la escala: sin entrenar, movimiento errático.
    const casiEstatico = (a.vx * a.vx + a.vy * a.vy) < 0.01;
    const ruido = RUIDO_A * a.multRuido * (casiEstatico ? RUIDO_ANTIATASCO : 1);
    fx += (Math.random() - 0.5) * 2 * ruido;
    fy += (Math.random() - 0.5) * 2 * ruido;

    // Integración semi-implícita de Euler con tope de velocidad.
    a.vx += fx * dt; a.vy += fy * dt;
    const v = Math.hypot(a.vx, a.vy);
    const vMax = a.velDeseada * 1.5;
    if (v > vMax) { a.vx *= vMax / v; a.vy *= vMax / v; }
    a.x += a.vx * dt; a.y += a.vy * dt;
  }

  /* --- 2) Colisiones estrictas (corrección posicional, 2 iteraciones) --- */
  for (let iter = 0; iter < 2; iter++) {
    grilla.reindexarAgentes(sim.agentes);

    for (const a of sim.agentes) {
      if (a.evacuado) continue;

      // Agente–agente: separar mitades del solape.
      for (const b of grilla.vecinos(a.x, a.y)) {
        if (b.id <= a.id) continue;
        let sx = a.x - b.x, sy = a.y - b.y;
        let d = Math.hypot(sx, sy);
        const minD = 2 * RADIO_AGENTE;
        if (d >= minD) continue;
        if (d < 1e-6) { sx = 1; sy = 0; d = 1e-6; }   // coincidentes: separar en x
        const corr = (minD - d) / 2;
        a.x += (sx / d) * corr; a.y += (sy / d) * corr;
        b.x -= (sx / d) * corr; b.y -= (sy / d) * corr;
      }

      // Agente–muro: proyectar fuera y cancelar la velocidad entrante.
      for (const m of grilla.murosCercanos(a.x, a.y)) {
        const q = puntoMasCercano(m, a.x, a.y);
        let sx = a.x - q.x, sy = a.y - q.y;
        let d = Math.hypot(sx, sy);
        const minD = RADIO_AGENTE + HOLGURA_MURO;
        if (d >= minD) continue;
        if (d < 1e-6) { sx = -a.vx; sy = -a.vy; d = Math.hypot(sx, sy) || 1; }
        const nx = sx / d, ny = sy / d;
        a.x += nx * (minD - d);
        a.y += ny * (minD - d);
        const vn = a.vx * nx + a.vy * ny;
        if (vn < 0) { a.vx -= vn * nx; a.vy -= vn * ny; }  // deslizar, no atravesar
      }
    }
  }

  /* --- 3) Evacuación: cruce del umbral de una salida --- */
  for (const a of sim.agentes) {
    if (a.evacuado) continue;
    for (const s of sim.plano.salidas) {
      if (seCruzan(a.prevX, a.prevY, a.x, a.y, s.x1, s.y1, s.x2, s.y2)) {
        a.evacuado = true;
        a.mesh.visible = false;
        if (a.etiqueta) a.etiqueta.visible = false;
        if (a.esSegu) sim.seguTiempoSalida = sim.tiempo;
        else sim.evacuados++;
        break;
      }
    }
  }

  // Mensaje de SEGU: rota cada PERIODO_CONSEJO_SEGU segundos mientras dirige
  // la evacuación (deja de tener sentido una vez que SEGU mismo evacuó).
  if (sim.segu && !sim.segu.evacuado && sim.tiempo >= sim.seguMensajeHasta) {
    sim.seguMensajeIdx = (sim.seguMensajeIdx + 1) % CONSEJOS_SEGU.length;
    sim.seguMensajeHasta = sim.tiempo + PERIODO_CONSEJO_SEGU;
  }
}

/* ============================================================
 * BUCLE PRINCIPAL (render + tiempo real)
 * ============================================================ */

let tUltimo = performance.now();

function animar(tAhora) {
  requestAnimationFrame(animar);
  const dtFrame = Math.min((tAhora - tUltimo) / 1000, 0.05);  // tope anti-salto
  tUltimo = tAhora;

  if (sim.estado === 'corriendo') {
    // Subpasos fijos ~120 Hz para estabilidad numérica.
    const pasos = Math.max(1, Math.min(6, Math.round(dtFrame / DT_FISICA)));
    const dtSub = dtFrame / pasos;
    for (let i = 0; i < pasos; i++) pasoFisica(dtSub);   // avanza sim.tiempo

    // Sincronizar la posición 3D de cada cilindro (y la etiqueta de SEGU).
    for (const a of sim.agentes) {
      if (a.evacuado) continue;
      const altura = a.esSegu ? ALTURA_SEGU : ALTURA_AGENTE;
      a.mesh.position.set(a.x, altura / 2, -a.y);
      if (a.etiqueta) a.etiqueta.position.set(a.x, altura + 0.45, -a.y);
    }

    if (sim.total > 0 && sim.agentes.every((a) => a.evacuado || a.atrapado)) {
      sim.estado = 'completado';
    }
    actualizarHUD();
  }

  controles.update();
  renderer.render(escena, camara);
}

/* ============================================================
 * EDICIÓN DEL PLANO EN EL VISOR
 * Herramientas (sección 3 del panel):
 *   - 'salida':    clic sobre un muro → se abre un vano del ancho
 *                  elegido en "Ancho de cada salida nueva".
 *   - 'obstaculo': arrastrar sobre el piso → rectángulo (mueble,
 *                  estantería, máquina) que los ocupantes rodean.
 *   - 'borrar':    clic sobre una salida (se cierra con muro) o un
 *                  obstáculo (se quita).
 * Cada cambio se puede deshacer. Funciona sobre planos DXF/DWG y
 * también sobre el recinto genérico (que pasa a quedar fijo).
 * ============================================================ */

const raycaster = new THREE.Raycaster();
const planoSuelo = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

/** @returns {number} Ancho (m) para la próxima salida que se marque. */
function anchoSalidaNueva() {
  const uas = Math.max(1, parseInt(ui.uasNueva.value, 10) || 1);
  return uas * UAS_METROS;
}

/**
 * Abre una salida en el muro más cercano al punto (px, py) del plano.
 * Corta el muro en dos tramos y agrega el vano como salida.
 * @param {number} px @param {number} py Coordenadas de plano (m).
 * @param {number} [anchoForzado] Ancho a usar (si no, el de la sección 3).
 * @returns {boolean} true si se pudo marcar.
 */
function marcarSalidaEn(px, py, anchoForzado) {
  const p = sim.plano;
  let mejor = null, mejorD = Infinity, mejorQ = null;
  for (const m of p.muros) {
    const q = puntoMasCercano(m, px, py);
    const d = Math.hypot(px - q.x, py - q.y);
    if (d < mejorD) { mejorD = d; mejor = m; mejorQ = q; }
  }
  if (!mejor || mejorD > 2.0) {
    avisarEdicion('Hacé clic más cerca de un muro: la salida se abre en el muro más próximo.');
    return false;
  }

  const dx = mejor.x2 - mejor.x1, dy = mejor.y2 - mejor.y1;
  const L = Math.hypot(dx, dy);
  const ux = dx / L, uy = dy / L;

  const anchoPedido = anchoForzado ?? anchoSalidaNueva();
  const ancho = Math.min(anchoPedido, Math.max(L - 0.1, 0.3));
  const h = ancho / 2;
  if (ancho < anchoPedido - 1e-6) {
    avisarEdicion(`Ese tramo de muro mide ${L.toFixed(2)} m: la salida quedó de ${ancho.toFixed(2)} m.`);
  }

  // Proyección del clic sobre el muro, acotada para que el vano quede dentro.
  let t = (mejorQ.x - mejor.x1) * ux + (mejorQ.y - mejor.y1) * uy;
  t = Math.max(h, Math.min(L - h, t));

  const ax = mejor.x1 + ux * (t - h), ay = mejor.y1 + uy * (t - h);
  const bx = mejor.x1 + ux * (t + h), by = mejor.y1 + uy * (t + h);

  // Reemplazar el muro original por los tramos que quedan a cada lado.
  p.muros.splice(p.muros.indexOf(mejor), 1);
  if (t - h > 0.05) p.muros.push({ x1: mejor.x1, y1: mejor.y1, x2: ax, y2: ay });
  if (L - (t + h) > 0.05) p.muros.push({ x1: bx, y1: by, x2: mejor.x2, y2: mejor.y2 });
  p.salidas.push({
    x1: ax, y1: ay, x2: bx, y2: by, ancho,
    cx: (ax + bx) / 2, cy: (ay + by) / 2,
  });
  return true;
}

/**
 * Borra el elemento más cercano al punto: una salida (el vano se cierra
 * con un tramo de muro) o un obstáculo.
 * @returns {{tipo:'salida'|'obstaculo', indice:number}|null} lo borrado.
 */
function borrarElementoEn(px, py) {
  const p = sim.plano;
  let tipo = null, indice = -1, mejorD = Infinity;

  for (const [i, s] of p.salidas.entries()) {
    const q = puntoMasCercano(s, px, py);
    const d = Math.hypot(px - q.x, py - q.y);
    if (d < 0.8 && d < mejorD) { mejorD = d; tipo = 'salida'; indice = i; }
  }
  for (const [i, o] of (p.obstaculos ?? []).entries()) {
    let d = dentroDeObstaculo(o, px, py) ? 0 : Infinity;
    for (const m of o.segs) {
      const q = puntoMasCercano(m, px, py);
      d = Math.min(d, Math.hypot(px - q.x, py - q.y));
    }
    if (d < 0.5 && d < mejorD) { mejorD = d; tipo = 'obstaculo'; indice = i; }
  }

  if (tipo === 'salida') {
    const [s] = p.salidas.splice(indice, 1);
    p.muros.push({ x1: s.x1, y1: s.y1, x2: s.x2, y2: s.y2 });
    return { tipo, indice };
  }
  if (tipo === 'obstaculo') {
    p.obstaculos.splice(indice, 1);
    return { tipo, indice };
  }
  avisarEdicion('No hay ninguna salida ni obstáculo cerca de ese punto.');
  return null;
}

/* ------------------------------------------------------------
 * Edición del recinto genérico: a diferencia de un plano DXF/JSON
 * (geometría fija, tal cual el archivo), el recinto genérico sigue
 * siendo PARAMÉTRICO después de editarlo: las salidas y obstáculos
 * que agregás se guardan como posición relativa al tamaño del
 * rectángulo (`sim.edicionesGenerico`) y se "repintan" cada vez que
 * cambia el ancho, la profundidad o las U.A.S., así el recinto puede
 * seguir creciendo, achicándose o cambiando de forma sin perder tus
 * ediciones.
 * ------------------------------------------------------------ */

/** Reconstruye el recinto genérico desde cero y reaplica las ediciones. */
function regenerarGenerico() {
  const plano = crearPlanoDemo(anchoEdificio(), profundidadEdificio(), parseInt(ui.uas.value, 10) || 1);
  sim.plano = plano;
  for (const e of sim.edicionesGenerico) {
    if (e.tipo === 'salida') {
      marcarSalidaEn(e.fx * plano.bbox.ancho, e.fy * plano.bbox.alto, e.ancho);
    } else if (e.tipo === 'obstaculo') {
      plano.obstaculos.push(obstaculoRectangular(
        e.fx0 * plano.bbox.ancho, e.fy0 * plano.bbox.alto,
        e.fx1 * plano.bbox.ancho, e.fy1 * plano.bbox.alto,
      ));
    } else if (e.tipo === 'borrarDefault' && plano.salidas[0]) {
      const s = plano.salidas[0];
      borrarElementoEn(s.cx, s.cy);
    }
  }
}

/**
 * Agrega una salida (clic sobre un muro), en el recinto genérico o en un
 * plano fijo (DXF/JSON), según corresponda. Guarda la edición para
 * "Deshacer" y, en el genérico, para que sobreviva a un cambio de tamaño.
 */
function agregarSalidaEn(px, py) {
  if (sim.estado === 'corriendo') sim.estado = 'pausado';
  if (sim.modoDXF) {
    const antes = structuredClone(sim.plano);
    sim.plano.obstaculos ??= [];
    if (!marcarSalidaEn(px, py)) return;
    sim.historial.push(antes);
    if (sim.historial.length > MAX_HISTORIAL) sim.historial.shift();
  } else {
    const bbox = sim.plano.bbox, ancho = anchoSalidaNueva();
    if (!marcarSalidaEn(px, py, ancho)) return;
    sim.edicionesGenerico.push({ tipo: 'salida', fx: px / bbox.ancho, fy: py / bbox.alto, ancho });
  }
  reconstruirTodo({ mantenerCamara: true });
}

/** Agrega un obstáculo rectangular (arrastre sobre el piso). */
function agregarObstaculoRect(x0, y0, x1, y1) {
  if (sim.estado === 'corriendo') sim.estado = 'pausado';
  if (sim.modoDXF) {
    const antes = structuredClone(sim.plano);
    sim.plano.obstaculos ??= [];
    sim.plano.obstaculos.push(obstaculoRectangular(x0, y0, x1, y1));
    sim.historial.push(antes);
    if (sim.historial.length > MAX_HISTORIAL) sim.historial.shift();
  } else {
    const bbox = sim.plano.bbox;
    sim.plano.obstaculos.push(obstaculoRectangular(x0, y0, x1, y1));
    sim.edicionesGenerico.push({
      tipo: 'obstaculo', fx0: x0 / bbox.ancho, fy0: y0 / bbox.alto,
      fx1: x1 / bbox.ancho, fy1: y1 / bbox.alto,
    });
  }
  reconstruirTodo({ mantenerCamara: true });
}

/** Borra la salida u obstáculo más cercano al punto. */
function borrarEnPunto(px, py) {
  if (sim.estado === 'corriendo') sim.estado = 'pausado';
  if (sim.modoDXF) {
    const antes = structuredClone(sim.plano);
    sim.plano.obstaculos ??= [];
    if (!borrarElementoEn(px, py)) return;
    sim.historial.push(antes);
    if (sim.historial.length > MAX_HISTORIAL) sim.historial.shift();
  } else {
    const bbox = sim.plano.bbox;
    const borrado = borrarElementoEn(px, py);
    if (!borrado) return;
    if (borrado.tipo === 'salida' && borrado.indice === 0) {
      // Era la salida sur que trae el recinto genérico por defecto: se
      // guarda la marca para que no reaparezca al cambiar el tamaño.
      sim.edicionesGenerico.push({ tipo: 'borrarDefault' });
    } else if (borrado.tipo === 'salida') {
      // Las salidas agregadas quedan, en orden, después de la del medio
      // (índice 0 = la original). El resto del array de edicionesGenerico
      // con tipo "salida" respeta ese mismo orden de inserción.
      const salidasEd = sim.edicionesGenerico.filter((e) => e.tipo === 'salida');
      const objetivo = salidasEd[borrado.indice - 1];
      if (objetivo) sim.edicionesGenerico.splice(sim.edicionesGenerico.indexOf(objetivo), 1);
    } else {
      const obstEd = sim.edicionesGenerico.filter((e) => e.tipo === 'obstaculo')[borrado.indice];
      if (obstEd) sim.edicionesGenerico.splice(sim.edicionesGenerico.indexOf(obstEd), 1);
    }
  }
  reconstruirTodo({ mantenerCamara: true });
}

/** Deshace la última edición (Ctrl+Z o botón). */
function deshacer() {
  if (sim.modoDXF) {
    const anterior = sim.historial.pop();
    if (!anterior) return;
    sim.plano = anterior;
  } else {
    if (sim.edicionesGenerico.length === 0) return;
    sim.edicionesGenerico.pop();
  }
  reconstruirTodo({ mantenerCamara: true });
}

/** Quita todas las ediciones y vuelve al plano tal como se cargó/empezó. */
function restaurarPlano() {
  if (sim.modoDXF) {
    if (!sim.planoBase) return;
    sim.historial.push(structuredClone(sim.plano));
    sim.plano = structuredClone(sim.planoBase);
  } else {
    if (sim.edicionesGenerico.length === 0) return;
    sim.edicionesGenerico = [];
  }
  reconstruirTodo({ mantenerCamara: true });
}

/** Muestra un aviso breve debajo de las herramientas. */
let temporizadorAviso = 0;
function avisarEdicion(texto) {
  ui.avisoEdicion.textContent = texto;
  ui.avisoEdicion.classList.remove('oculto');
  clearTimeout(temporizadorAviso);
  temporizadorAviso = setTimeout(() => ui.avisoEdicion.classList.add('oculto'), 6000);
}

/** Convierte un evento de puntero en coordenadas de plano (m), o null. */
function puntoEnPlano(e) {
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(ndc, camara);
  const punto = new THREE.Vector3();
  if (!raycaster.ray.intersectPlane(planoSuelo, punto)) return null;
  // Mundo 3D (x, z) → coordenadas de plano (x, y = -z).
  return { x: punto.x, y: -punto.z };
}

/** Redondea a 5 cm para que los obstáculos queden prolijos. */
const redondear = (v) => Math.round(v * 20) / 20;

/* Clic vs. arrastre de cámara: solo se edita si el puntero casi no se movió. */
let puntoPointerDown = null;
/** Rectángulo en curso de la herramienta "obstáculo". */
let dibujo = null;

renderer.domElement.addEventListener('pointerdown', (e) => {
  puntoPointerDown = { x: e.clientX, y: e.clientY };
  if (sim.herramienta !== 'obstaculo' || e.button !== 0) return;
  const q = puntoEnPlano(e);
  if (!q) return;
  const malla = new THREE.Mesh(new THREE.BoxGeometry(1, ALTURA_OBSTACULO, 1), MAT_BORRADOR);
  malla.visible = false;
  escena.add(malla);
  dibujo = { x0: redondear(q.x), y0: redondear(q.y), x1: redondear(q.x), y1: redondear(q.y), malla };
  renderer.domElement.setPointerCapture(e.pointerId);
});

renderer.domElement.addEventListener('pointermove', (e) => {
  if (!dibujo) return;
  const q = puntoEnPlano(e);
  if (!q) return;
  dibujo.x1 = redondear(q.x); dibujo.y1 = redondear(q.y);
  const w = Math.abs(dibujo.x1 - dibujo.x0), h = Math.abs(dibujo.y1 - dibujo.y0);
  dibujo.malla.visible = w > 0.01 && h > 0.01;
  dibujo.malla.scale.set(Math.max(w, 0.01), 1, Math.max(h, 0.01));
  dibujo.malla.position.set((dibujo.x0 + dibujo.x1) / 2, ALTURA_OBSTACULO / 2,
    -(dibujo.y0 + dibujo.y1) / 2);
  ui.avisoEdicion.textContent = `Obstáculo: ${w.toFixed(2)} × ${h.toFixed(2)} m`;
  ui.avisoEdicion.classList.remove('oculto');
});

renderer.domElement.addEventListener('pointerup', (e) => {
  if (dibujo) {
    const d = dibujo;
    dibujo = null;
    escena.remove(d.malla);
    d.malla.geometry.dispose();
    const w = Math.abs(d.x1 - d.x0), h = Math.abs(d.y1 - d.y0);
    if (w < LADO_MIN_OBSTACULO || h < LADO_MIN_OBSTACULO) {
      avisarEdicion('Arrastrá sobre el piso para dibujar el obstáculo (mínimo 0,20 × 0,20 m).');
      return;
    }
    agregarObstaculoRect(d.x0, d.y0, d.x1, d.y1);
    avisarEdicion(`Obstáculo agregado: ${w.toFixed(2)} × ${h.toFixed(2)} m.`);
    return;
  }

  if (!sim.herramienta || !puntoPointerDown) return;
  const dx = e.clientX - puntoPointerDown.x, dy = e.clientY - puntoPointerDown.y;
  if (dx * dx + dy * dy > 36) return;   // fue un giro de cámara, no un clic
  const q = puntoEnPlano(e);
  if (!q) return;
  if (sim.herramienta === 'salida') agregarSalidaEn(q.x, q.y);
  else if (sim.herramienta === 'borrar') borrarEnPunto(q.x, q.y);
});

window.addEventListener('keydown', (e) => {
  if (e.key === 'Escape' && sim.herramienta) seleccionarHerramienta(null);
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z'
      && !(e.target instanceof HTMLInputElement)) {
    e.preventDefault();
    deshacer();
  }
});

/* ============================================================
 * HUD Y PANEL DE CONTROL (UI)
 * ============================================================ */

const $ = (id) => document.getElementById(id);

const ui = {
  superficie: $('in-superficie'),
  anchoEdificio: $('in-ancho-edificio'),
  profundidadEdificio: $('in-profundidad-edificio'),
  nivelEntrenamiento: $('sel-entrenamiento'),
  factorSel: $('sel-factor'),
  factorCustom: $('in-factor-custom'),
  uas: $('in-uas'),
  ayudaUas: $('ayuda-uas'),
  outOcupantes: $('out-ocupantes'),
  outAncho: $('out-ancho-salida'),
  outUasReq: $('out-uas-req'),
  outCumplimiento: $('out-cumplimiento'),
  outPlano: $('out-plano'),
  outDwg: $('out-dwg'),
  dxf: $('in-dxf'),
  dwg: $('in-dwg'),
  btnEjemplo: $('btn-ejemplo'),
  btnDemo: $('btn-demo'),
  btnsHerramienta: [...document.querySelectorAll('[data-herramienta]')],
  ayudaHerramienta: $('ayuda-herramienta'),
  avisoEdicion: $('aviso-edicion'),
  uasNueva: $('in-uas-nueva'),
  btnDeshacer: $('btn-deshacer'),
  btnRestaurar: $('btn-restaurar'),
  btnGuardar: $('btn-guardar'),
  btnIniciar: $('btn-iniciar'),
  btnReiniciar: $('btn-reiniciar'),
  hudTiempo: $('hud-tiempo'),
  hudEvacuados: $('hud-evacuados'),
  hudRestantes: $('hud-restantes'),
  hudEstado: $('hud-estado'),
  hudCongestion: $('hud-congestion'),
  panelSegu: $('panel-segu'),
  seguEstado: $('segu-estado'),
  seguMensaje: $('segu-mensaje'),
};

/** @returns {number} Factor de ocupación vigente (m²/persona). */
function factorOcupacion() {
  if (ui.factorSel.value === 'custom') {
    return Math.max(0.5, parseFloat(ui.factorCustom.value) || 3);
  }
  return parseFloat(ui.factorSel.value);
}

/** @returns {number} Superficie del sector vigente (m²). */
function superficie() {
  return Math.max(10, parseFloat(ui.superficie.value) || 200);
}

/** @returns {number} Ancho (eje x) del recinto genérico (m). */
function anchoEdificio() {
  return Math.max(2, parseFloat(ui.anchoEdificio.value) || 10);
}

/** @returns {number} Profundidad (eje y) del recinto genérico (m). */
function profundidadEdificio() {
  return Math.max(2, parseFloat(ui.profundidadEdificio.value) || 10);
}

/**
 * Mantiene sincronizados "Superficie" con "Ancho × Profundidad" del recinto
 * genérico: cambiar uno actualiza el otro. `origen` indica cuál cambió
 * para no entrar en bucle. Cuando el origen es el ancho/profundidad, la
 * superficie pasa a ser simplemente su producto; cuando el origen es la
 * superficie, el rectángulo escala manteniendo su forma (relación ancho/
 * profundidad) vigente.
 * @param {'dimensiones'|'superficie'} origen
 */
function sincronizarDimensiones(origen) {
  if (sim.modoDXF) return;   // con archivo, la superficie es solo informativa
  if (origen === 'dimensiones') {
    ui.superficie.value = String(Math.round(anchoEdificio() * profundidadEdificio()));
  } else {
    const anchoActual = anchoEdificio(), profActual = profundidadEdificio();
    const factor = Math.sqrt(superficie() / Math.max(4, anchoActual * profActual));
    ui.anchoEdificio.value = (anchoActual * factor).toFixed(1);
    ui.profundidadEdificio.value = (profActual * factor).toFixed(1);
  }
}

/** @returns {number} N máx. de ocupantes según Sup / x (con tope de rendimiento). */
function ocupantesMax() {
  return Math.min(MAX_OCUPANTES, Math.max(1, Math.floor(superficie() / factorOcupacion())));
}

/** @returns {number} U.A.S. requeridas: n = N/100 por exceso, mínimo 2. */
function uasRequeridas(n) {
  return Math.max(2, Math.ceil(n / 100));
}

/** Refresca los indicadores normativos del panel. */
function actualizarPanel() {
  const n = ocupantesMax();
  const req = uasRequeridas(n);
  ui.outOcupantes.textContent = `${n} personas`;
  ui.outUasReq.textContent = `${req} U.A.S. (${(req * UAS_METROS).toFixed(2)} m)`;

  // Con un plano fijo (DXF o escenario .json) las U.A.S. salen de las
  // salidas del plano; con el recinto genérico (editado o no) el campo
  // sigue valiendo: define el ancho de la salida del medio y el de cada
  // salida nueva que agregues sin tocar ese campo.
  ui.uas.disabled = sim.modoDXF;
  ui.ayudaUas.classList.toggle('oculto', !sim.modoDXF);
  // El ancho/profundidad del recinto genérico no aplica a un plano de archivo.
  ui.anchoEdificio.disabled = sim.modoDXF;
  ui.profundidadEdificio.disabled = sim.modoDXF;

  let provistas;
  if (sim.modoDXF) {
    provistas = sim.plano.salidas.reduce(
      (acc, s) => acc + Math.floor(s.ancho / UAS_METROS + 1e-6), 0);
    const anchoTotal = sim.plano.salidas.reduce((acc, s) => acc + s.ancho, 0);
    ui.outAncho.textContent =
      `${anchoTotal.toFixed(2)} m en ${sim.plano.salidas.length} salida(s) del plano`;
  } else {
    provistas = parseInt(ui.uas.value, 10) || 1;
    ui.outAncho.textContent = `${(provistas * UAS_METROS).toFixed(2)} m`;
  }

  ui.outCumplimiento.classList.remove('oculto', 'ok', 'error');
  if (provistas >= req) {
    ui.outCumplimiento.classList.add('ok');
    ui.outCumplimiento.textContent =
      `✔ Cumple: ${provistas} U.A.S. provistas ≥ ${req} requeridas.`;
  } else {
    ui.outCumplimiento.classList.add('error');
    ui.outCumplimiento.textContent =
      `✘ NO cumple: ${provistas} U.A.S. provistas < ${req} requeridas.`;
  }

  ui.btnDeshacer.disabled = sim.modoDXF
    ? sim.historial.length === 0 : sim.edicionesGenerico.length === 0;
  ui.btnRestaurar.disabled = sim.modoDXF
    ? !sim.planoBase : sim.edicionesGenerico.length === 0;
}

/** Refresca el HUD (cronómetro, contadores y panel de SEGU). */
function actualizarHUD() {
  const restantes = sim.total - sim.evacuados;
  ui.hudTiempo.textContent = `${sim.tiempo.toFixed(1).replace('.', ',')} s`;
  ui.hudEvacuados.textContent = `${sim.evacuados} / ${sim.total}`;
  ui.hudRestantes.textContent = sim.atrapados
    ? `${restantes} (${sim.atrapados} sin salida)` : String(restantes);

  const etiquetas = {
    listo: 'Listo', corriendo: 'Evacuando…',
    pausado: 'Pausado',
    completado: sim.atrapados ? `⚠ Terminado: ${sim.atrapados} sin salida` : '✔ Completado',
  };
  ui.hudEstado.textContent = etiquetas[sim.estado];
  ui.hudEstado.classList.toggle('estado-corriendo', sim.estado === 'corriendo');
  ui.hudEstado.classList.toggle('estado-completado', sim.estado === 'completado' && !sim.atrapados);
  ui.hudEstado.classList.toggle('estado-alerta', sim.estado === 'completado' && sim.atrapados > 0);

  ui.btnIniciar.textContent =
    sim.estado === 'corriendo' ? '⏸ Pausar'
      : sim.estado === 'pausado' ? '▶ Reanudar'
        : '▶ Iniciar evacuación';

  // Congestionados: ocupantes frenados más de 1 s (indicador de cuello de botella).
  let congestion = 0;
  for (const a of sim.agentes) {
    if (!a.evacuado && !a.esSegu && a.tAtascado > 1) congestion++;
  }
  ui.hudCongestion.textContent = String(congestion);

  actualizarPanelSegu();
}

/** Refresca el cartel de SEGU (estado + consejo de evacuación). */
function actualizarPanelSegu() {
  if (!sim.segu || sim.estado === 'listo') {
    ui.panelSegu.classList.add('oculto');
    return;
  }
  ui.panelSegu.classList.remove('oculto');
  if (sim.segu.atrapado) {
    ui.seguEstado.textContent = '👷 SEGU — ⚠ sin salida';
    ui.seguMensaje.textContent = 'No hay ningún recorrido posible hasta una salida desde su posición.';
  } else if (sim.segu.evacuado) {
    ui.seguEstado.textContent = '👷 SEGU — ✔ evacuado (el último)';
    ui.seguMensaje.textContent = `Salió a los ${sim.seguTiempoSalida?.toFixed(1).replace('.', ',') ?? '—'} s, ` +
      'después de que salió (o quedó sin salida) todo el resto.';
  } else if (sim.segu.esperando) {
    ui.seguEstado.textContent = '👷 SEGU — dirigiendo la evacuación';
    ui.seguMensaje.textContent = `💬 "${CONSEJOS_SEGU[sim.seguMensajeIdx]}"`;
  } else {
    ui.seguEstado.textContent = '👷 SEGU — evacuando (fue el último en salir)';
    ui.seguMensaje.textContent = '💬 "Ya está todo el mundo afuera. Ahora salgo yo."';
  }
}

/** Muestra la descripción del plano vigente en el panel. */
function describirPlano() {
  const p = sim.plano;
  const obst = p.obstaculos?.length ?? 0;
  const editado = sim.modoDXF
    ? false   // el nombre de un plano de archivo ya dice "(editado)" si corresponde
    : sim.edicionesGenerico.length > 0;
  const lineas = [
    `Plano: ${p.nombre}${editado ? ' (editado)' : ''}`,
    `Dimensiones: ${p.bbox.ancho.toFixed(1)} × ${p.bbox.alto.toFixed(1)} m`,
    `Muros: ${p.muros.length} · Salidas: ${p.salidas.length} · Obstáculos: ${obst}`,
  ];
  for (const [i, s] of p.salidas.entries()) {
    lineas.push(
      `  Salida ${i + 1}: ${s.ancho.toFixed(2)} m ≈ ${Math.floor(s.ancho / UAS_METROS + 1e-6)} U.A.S.`);
  }
  if (p.advertencias?.length) lineas.push(...p.advertencias.map((a) => `⚠ ${a}`));
  if (sim.atrapados) {
    lineas.push(`⚠ ${sim.atrapados} ocupante(s) (en rojo) no tienen ningún recorrido ` +
      'hasta una salida: falta una salida o un obstáculo les cierra el paso.');
  }
  if (sim.mapa?.aproximado) {
    lineas.push(`⚠ Plano muy grande: recorridos calculados con celdas de ` +
      `${sim.mapa.celda.toFixed(2)} m (aproximados en pasos angostos).`);
  }
  ui.outPlano.textContent = lineas.join('\n');
}

/* ============================================================
 * ORQUESTACIÓN (reconstruir / iniciar / reiniciar)
 * ============================================================ */

/**
 * Reconstruye escenario, mapa de recorridos y agentes con los
 * parámetros vigentes.
 * @param {{mantenerCamara?: boolean}} [opciones]
 */
function reconstruirTodo({ mantenerCamara = false } = {}) {
  sim.estado = 'listo';
  sim.tiempo = 0;
  sim.tEleccion = 0;
  sim.evacuados = 0;
  sim.nivelEntrenamiento = ui.nivelEntrenamiento.value;

  if (!sim.modoDXF) {
    regenerarGenerico();
  }
  sim.plano.obstaculos ??= [];
  grilla.registrarMuros(segmentosColision(sim.plano));
  sim.mapa = sim.plano.salidas.length ? new MapaRutas(sim.plano) : null;
  construirEscenario(mantenerCamara);
  crearAgentes(ocupantesMax());

  actualizarPanel();
  describirPlano();
  actualizarHUD();
}

const AYUDAS_HERRAMIENTA = {
  salida: 'Clic sobre un muro, donde va la puerta. Ancho: el de "Ancho de cada salida nueva".',
  obstaculo: 'Arrastrá sobre el piso para dibujar el obstáculo. Para girar la vista: botón derecho.',
  borrar: 'Clic sobre una salida (se cierra con muro) o sobre un obstáculo (se quita).',
};

/**
 * Activa una herramienta de edición (o la apaga si ya estaba activa).
 * @param {'salida'|'obstaculo'|'borrar'|null} h
 */
function seleccionarHerramienta(h) {
  sim.herramienta = sim.herramienta === h ? null : h;
  for (const b of ui.btnsHerramienta) {
    b.classList.toggle('activo', b.dataset.herramienta === sim.herramienta);
  }
  renderer.domElement.style.cursor = sim.herramienta ? 'crosshair' : '';
  // Dibujando obstáculos el botón izquierdo dibuja; la vista gira con el derecho.
  controles.mouseButtons.LEFT = sim.herramienta === 'obstaculo' ? -1 : THREE.MOUSE.ROTATE;
  controles.mouseButtons.RIGHT = sim.herramienta === 'obstaculo' ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN;
  ui.ayudaHerramienta.textContent = sim.herramienta
    ? AYUDAS_HERRAMIENTA[sim.herramienta] + ' (Esc para salir)'
    : 'Elegí una herramienta para editar el plano.';
}

/** Deja el plano recibido como plano fijo vigente. */
function fijarPlano(plano) {
  sim.plano = plano;
  sim.plano.obstaculos ??= [];
  sim.modoDXF = true;
  sim.planoBase = structuredClone(sim.plano);   // para "Restaurar plano original"
  sim.historial = [];
}

/** Carga en el simulador un plano recibido del backend (JSON del DXF). */
function aplicarPlanoBackend(json, nombre) {
  /** @type {Salida[]} */
  const salidas = json.salidas.map((s) => ({
    ...s, cx: (s.x1 + s.x2) / 2, cy: (s.y1 + s.y2) / 2,
  }));
  fijarPlano({
    muros: json.muros,
    salidas,
    obstaculos: (json.obstaculos ?? []).map((segs) => ({ segs })),
    bbox: json.bbox,
    nombre,
    superficie: json.superficie_bbox,
    advertencias: json.advertencias ?? [],
  });
  if (json.superficie_bbox) {
    ui.superficie.value = String(Math.round(json.superficie_bbox));
  }
  reconstruirTodo();
  if (salidas.length === 0) {
    avisarEdicion('El plano no trae salidas: elegí "🚪 Agregar salida" y hacé clic ' +
      'sobre el muro donde está cada puerta.');
    seleccionarHerramienta('salida');
  }
}

/* ---------------- Guardar / abrir escenario (.json) ---------------- */

const FORMATO_ESCENARIO = 'simulador-evacuacion-3d';

/** Descarga el plano editado y los parámetros como archivo .json. */
function guardarEscenario() {
  const datos = {
    formato: FORMATO_ESCENARIO,
    version: 1,
    plano: sim.plano,
    parametros: {
      superficie: ui.superficie.value,
      factor: ui.factorSel.value,
      factorCustom: ui.factorCustom.value,
      uas: ui.uas.value,
      uasNueva: ui.uasNueva.value,
    },
  };
  const nombre = sim.plano.nombre
    .replace(/\.(dxf|dwg|json)$/i, '')
    .replace(/[^\p{L}\p{N}]+/gu, '-').replace(/^-|-$/g, '') || 'escenario';
  const blob = new Blob([JSON.stringify(datos)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `escenario-${nombre}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

/** Abre un escenario .json guardado con "Guardar escenario". */
function abrirEscenario(texto, nombreArchivo) {
  const datos = JSON.parse(texto);
  if (datos?.formato !== FORMATO_ESCENARIO || !datos.plano?.muros) {
    throw new Error('el archivo no es un escenario de este simulador');
  }
  const par = datos.parametros ?? {};
  if (par.superficie) ui.superficie.value = par.superficie;
  if (par.factor) ui.factorSel.value = par.factor;
  if (par.factorCustom) ui.factorCustom.value = par.factorCustom;
  if (par.uas) ui.uas.value = par.uas;
  if (par.uasNueva) ui.uasNueva.value = par.uasNueva;
  ui.factorCustom.classList.toggle('oculto', ui.factorSel.value !== 'custom');
  fijarPlano(datos.plano);
  sim.plano.nombre ||= nombreArchivo;
  reconstruirTodo();
}

/* ---------------- Eventos de la UI ---------------- */

ui.btnIniciar.addEventListener('click', () => {
  if (sim.estado === 'corriendo') {
    sim.estado = 'pausado';
  } else if (sim.estado === 'pausado') {
    sim.estado = 'corriendo';
  } else {
    if (sim.estado === 'completado') reconstruirTodo({ mantenerCamara: true });
    if (sim.plano.salidas.length === 0) {
      avisarEdicion('No hay salidas: agregá al menos una con "🚪 Agregar salida".');
      return;
    }
    sim.estado = 'corriendo';
  }
  actualizarHUD();
});

ui.btnReiniciar.addEventListener('click', () => reconstruirTodo({ mantenerCamara: true }));

for (const el of [ui.uas, ui.factorCustom]) {
  el.addEventListener('change', () => reconstruirTodo({ mantenerCamara: sim.modoDXF }));
}

ui.superficie.addEventListener('change', () => {
  sincronizarDimensiones('superficie');
  reconstruirTodo({ mantenerCamara: sim.modoDXF });
});

for (const el of [ui.anchoEdificio, ui.profundidadEdificio]) {
  el.addEventListener('change', () => {
    sincronizarDimensiones('dimensiones');
    reconstruirTodo({ mantenerCamara: sim.modoDXF });
  });
}

ui.factorSel.addEventListener('change', () => {
  ui.factorCustom.classList.toggle('oculto', ui.factorSel.value !== 'custom');
  reconstruirTodo({ mantenerCamara: true });
});

ui.nivelEntrenamiento.addEventListener('change', () => reconstruirTodo({ mantenerCamara: true }));

/** Procesa el archivo elegido: .dxf se lee en el navegador, .json es un escenario. */
ui.dxf.addEventListener('change', async () => {
  const archivo = ui.dxf.files?.[0];
  if (!archivo) return;
  const nombre = archivo.name.toLowerCase();
  if (nombre.endsWith('.json')) {
    try {
      abrirEscenario(await archivo.text(), archivo.name);
    } catch (e) {
      alert(`No se pudo abrir el escenario: ${e.message}`);
    }
    return;
  }
  try {
    const plano = leerDXF(await archivo.text());
    aplicarPlanoBackend(plano, archivo.name);
  } catch (e) {
    alert(`No se pudo procesar el plano: ${e.message}`);
  }
});

ui.btnEjemplo.addEventListener('click', () => {
  aplicarPlanoBackend(PLANO_EJEMPLO, 'Oficina de ejemplo (DXF)');
});

ui.btnDemo.addEventListener('click', () => {
  sim.modoDXF = false;
  sim.planoBase = null;
  sim.historial = [];
  sim.edicionesGenerico = [];
  ui.dxf.value = '';
  reconstruirTodo();
});

for (const b of ui.btnsHerramienta) {
  b.addEventListener('click', () => seleccionarHerramienta(b.dataset.herramienta));
}

ui.btnDeshacer.addEventListener('click', deshacer);

ui.btnRestaurar.addEventListener('click', restaurarPlano);

ui.btnGuardar.addEventListener('click', guardarEscenario);

/** Carga DXF/DWG (prueba beta) vía backend; a diferencia de in-dxf, siempre deja un plano fijo. */
async function subirPlanoCad(archivo) {
  const datos = new FormData();
  datos.append('archivo', archivo);
  try {
    const resp = await fetch(CFG.urlProcesar ?? '/api/procesar-dxf', { method: 'POST', body: datos });
    if (!resp.ok) {
      const err = await resp.json().catch(() => ({}));
      throw new Error(err.detail || `Error HTTP ${resp.status}`);
    }
    aplicarPlanoBackend(await resp.json(), archivo.name);
  } catch (e) {
    alert(`No se pudo procesar el plano: ${e.message}`);
  }
}

/** DWG (sección aparte, "prueba beta"): mismo backend, distinta entrada. */
ui.dwg?.addEventListener('change', async () => {
  const archivo = ui.dwg.files?.[0];
  if (archivo) await subirPlanoCad(archivo);
});

/** Consulta al backend si hay conversor DWG disponible y lo informa (sección beta). */
(async () => {
  if (CFG.web) {   // versión web: no existe el conversor DWG, se oculta la sección
    document.getElementById('detalle-dwg')?.remove();
    return;
  }
  if (!ui.outDwg) return;
  try {
    const resp = await fetch('/api/capacidades');
    const cap = await resp.json();
    ui.outDwg.textContent = cap.dwg
      ? `✔ Hay un conversor instalado (${cap.conversor === 'libredwg' ? 'LibreDWG' : 'ODA File Converter'}), pero la conversión puede fallar según el DWG.`
      : '✘ No hay conversor DWG instalado en esta PC: esta opción no va a funcionar.';
  } catch { /* sin backend no hay nada que informar */ }
})();

/* ---------------- Arranque ---------------- */

// Versión web (beta abierta): aviso y enlace de comentarios, según config.js.
if (CFG.web) document.getElementById('aviso-beta')?.classList.remove('oculto');
if (CFG.urlComentarios) {
  const enlace = document.getElementById('enlace-comentarios');
  if (enlace) { enlace.href = CFG.urlComentarios; enlace.classList.remove('oculto'); }
}

ajustarTamano();
seleccionarHerramienta(null);
reconstruirTodo();
requestAnimationFrame(animar);

// Handle de depuración (consola del navegador): estado, escena y motor.
window.__evacSim = {
  sim, renderer, escena, camara,
  pasoFisica, reconstruirTodo, actualizarHUD, aplicarPlanoBackend,
  marcarSalidaEn, borrarElementoEn, agregarSalidaEn, agregarObstaculoRect, borrarEnPunto,
  deshacer, restaurarPlano, regenerarGenerico, obstaculoRectangular,
  abrirEscenario, guardarEscenario, seleccionarHerramienta, fijarPlano,
  leerDXF, PLANO_EJEMPLO,
};
