"""Grafo de celdas exploradas y ruta mas corta (fase 2, speed-run).

Modulo autocontenido para la SEGUNDA fase de la corrida (carrera): durante el
mapeo por pared izquierda se graba, de forma PASIVA, el grafo de celdas que el
robot realmente recorrio (nodos = celdas visitadas, aristas = pares de celdas
entre las que el robot se movio). Cuando el robot regresa al inicio y ya vio la
meta (verde), se calcula la ruta mas corta inicio->meta con BFS sobre ese grafo.

Como el grafo solo contiene aristas que el robot SI recorrio, BFS nunca cruza
una pared u obstaculo (por construccion) y elimina automaticamente los
callejones sin salida y las vueltas redundantes: la ruta mas corta ignora
cualquier ramal por el que se entro y se salio.

Grilla 12x8 CARDINAL, con la misma convencion que ``web/index.html`` y
``visualizador_web.py`` (columnas A..L -> col 0..11, filas 1..8 -> row 0..7):
- NORTE disminuye la fila (sube en el plano), SUR la aumenta.
- ESTE aumenta la columna, OESTE la disminuye.

Esto es a proposito distinto de ``motion_grid.py`` (grilla 6x4 del Flujo A
original): no se toca ese modulo para no alterar el mapeo actual.
"""

from collections import deque
from dataclasses import dataclass, field

HEADINGS = ['NORTE', 'ESTE', 'SUR', 'OESTE']

_DELTA = {
    'NORTE': (0, -1),
    'ESTE': (1, 0),
    'SUR': (0, 1),
    'OESTE': (-1, 0),
}


def turn_right(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) + 1) % 4]


def turn_left(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) - 1) % 4]


def turn_180(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) + 2) % 4]


def cell_from_name(name: str) -> tuple:
    """'A7' -> (col=0, row=6). Columnas A..L, filas 1..8 (base 1)."""
    name = name.strip().upper()
    col = ord(name[0]) - ord('A')
    row = int(name[1:]) - 1
    return col, row


def _arista(a: tuple, b: tuple) -> frozenset:
    """Arista no dirigida entre dos celdas."""
    return frozenset((a, b))


@dataclass
class RouteExplorer:
    """Graba el grafo de celdas recorridas durante el mapeo.

    ``avanzar()`` y ``girar()`` se llaman desde ``maze_solver`` cuando el robot
    ya ejecuto (por su cuenta) un avance de una celda o un giro fijo de 90; este
    modulo NO decide movimiento, solo lleva la cuenta logica de celda/heading y
    registra nodos y aristas.
    """

    col: int
    row: int
    heading: str
    num_columns: int = 12
    num_rows: int = 8
    nodos: set = field(default_factory=set)
    aristas: set = field(default_factory=set)

    def __post_init__(self):
        self.nodos.add((self.col, self.row))

    @classmethod
    def from_cell_name(cls, name: str, heading: str,
                       num_columns: int = 12, num_rows: int = 8):
        col, row = cell_from_name(name)
        return cls(col=col, row=row, heading=heading,
                   num_columns=num_columns, num_rows=num_rows)

    @property
    def cell(self) -> tuple:
        return (self.col, self.row)

    def girar(self, direccion: str) -> None:
        """direccion in {'DERECHA','IZQUIERDA','ATRAS','NINGUNO'}."""
        if direccion == 'DERECHA':
            self.heading = turn_right(self.heading)
        elif direccion == 'IZQUIERDA':
            self.heading = turn_left(self.heading)
        elif direccion == 'ATRAS':
            self.heading = turn_180(self.heading)
        elif direccion == 'NINGUNO':
            return
        else:
            raise ValueError(f'direccion de giro desconocida: {direccion}')

    def avanzar(self) -> tuple:
        """Avanza una celda al heading actual (con clamp a bordes) y registra
        nodo + arista prev->nueva. Devuelve la celda nueva."""
        prev = (self.col, self.row)
        dx, dy = _DELTA[self.heading]
        new_col = max(0, min(self.num_columns - 1, self.col + dx))
        new_row = max(0, min(self.num_rows - 1, self.row + dy))
        self.col, self.row = new_col, new_row
        nueva = (new_col, new_row)
        self.nodos.add(nueva)
        if nueva != prev:
            self.aristas.add(_arista(prev, nueva))
        return nueva


def bfs_ruta(inicio: tuple, meta: tuple, aristas) -> list:
    """Ruta mas corta inicio->meta (lista de celdas incluida) por BFS.

    ``aristas`` es un iterable de ``frozenset({celda_a, celda_b})``. Devuelve la
    lista de celdas ``[inicio, ..., meta]`` o ``None`` si la meta es inalcanzable
    con las aristas recorridas.
    """
    if inicio == meta:
        return [inicio]

    adyacencia = {}
    for arista in aristas:
        a, b = tuple(arista)
        adyacencia.setdefault(a, set()).add(b)
        adyacencia.setdefault(b, set()).add(a)

    if inicio not in adyacencia or meta not in adyacencia:
        return None

    previo = {inicio: None}
    cola = deque([inicio])
    while cola:
        actual = cola.popleft()
        if actual == meta:
            break
        for vecino in adyacencia.get(actual, ()):  # orden no determinista, ok
            if vecino not in previo:
                previo[vecino] = actual
                cola.append(vecino)

    if meta not in previo:
        return None

    ruta = []
    nodo = meta
    while nodo is not None:
        ruta.append(nodo)
        nodo = previo[nodo]
    ruta.reverse()
    return ruta


def _direccion_hacia(desde: tuple, hacia: tuple) -> str:
    """Heading cardinal para ir de una celda a otra adyacente."""
    dc = hacia[0] - desde[0]
    dr = hacia[1] - desde[1]
    for heading, (dx, dy) in _DELTA.items():
        if (dx, dy) == (dc, dr):
            return heading
    raise ValueError(f'celdas no adyacentes: {desde} -> {hacia}')


def _giro_entre(heading_actual: str, heading_objetivo: str) -> str:
    """Giro necesario para pasar de un heading a otro."""
    if heading_actual == heading_objetivo:
        return 'NINGUNO'
    if turn_right(heading_actual) == heading_objetivo:
        return 'DERECHA'
    if turn_left(heading_actual) == heading_objetivo:
        return 'IZQUIERDA'
    return 'ATRAS'


def celdas_a_movimientos(ruta: list, heading_inicial: str) -> list:
    """Convierte una lista de celdas en un guion de movimientos para MANEJAR.

    Devuelve una lista de dicts ``{'giro': <IZQUIERDA|DERECHA|ATRAS|NINGUNO>,
    'celdas': <n>}``: en cada tramo recto se gira una sola vez al rumbo del
    tramo y luego se avanzan ``celdas`` celdas seguidas. Lista vacia si la ruta
    tiene menos de dos celdas.
    """
    if not ruta or len(ruta) < 2:
        return []

    movimientos = []
    heading = heading_inicial
    i = 0
    while i < len(ruta) - 1:
        objetivo = _direccion_hacia(ruta[i], ruta[i + 1])
        giro = _giro_entre(heading, objetivo)
        heading = objetivo
        # Contar cuantas celdas seguidas van en este mismo rumbo.
        celdas = 1
        j = i + 1
        while (j < len(ruta) - 1 and
               _direccion_hacia(ruta[j], ruta[j + 1]) == objetivo):
            celdas += 1
            j += 1
        movimientos.append({'giro': giro, 'celdas': celdas})
        i = j
    return movimientos
