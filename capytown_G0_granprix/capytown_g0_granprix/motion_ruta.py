"""Proyeccion en celdas de la ruta fija (fase 2, speed-run).

Grilla 12x8 CARDINAL, con la misma convencion que ``web/index.html`` y
``visualizador_web.py`` (columnas A..L -> col 0..11, filas 1..8 -> row 0..7):
- NORTE disminuye la fila (sube en el plano), SUR la aumenta.
- ESTE aumenta la columna, OESTE la disminuye.

Esto es a proposito distinto de ``motion_grid.py`` (grilla 6x4 del Flujo A
original): no se toca ese modulo para no alterar el mapeo actual.
"""

from dataclasses import dataclass

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


@dataclass
class RouteExplorer:
    """Cursor lógico usado para dibujar el guion fijo sobre la grilla web."""

    col: int
    row: int
    heading: str
    num_columns: int = 12
    num_rows: int = 8
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
        """Avanza una celda al heading actual, limitado a los bordes."""
        dx, dy = _DELTA[self.heading]
        new_col = max(0, min(self.num_columns - 1, self.col + dx))
        new_row = max(0, min(self.num_rows - 1, self.row + dy))
        self.col, self.row = new_col, new_row
        return (new_col, new_row)
