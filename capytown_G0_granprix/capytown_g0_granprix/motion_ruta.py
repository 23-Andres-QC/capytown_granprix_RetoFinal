"""Módulo motion_ruta."""

from dataclasses import dataclass

HEADINGS = ['NORTE', 'ESTE', 'SUR', 'OESTE']

_DELTA = {
    'NORTE': (0, -1),
    'ESTE': (1, 0),
    'SUR': (0, 1),
    'OESTE': (-1, 0),
}


def turn_right(heading: str) -> str:
    """Ejecuta turn right."""
    return HEADINGS[(HEADINGS.index(heading) + 1) % 4]


def turn_left(heading: str) -> str:
    """Ejecuta turn left."""
    return HEADINGS[(HEADINGS.index(heading) - 1) % 4]


def turn_180(heading: str) -> str:
    """Ejecuta turn 180."""
    return HEADINGS[(HEADINGS.index(heading) + 2) % 4]


def cell_from_name(name: str) -> tuple:
    """Ejecuta cell from name."""
    name = name.strip().upper()
    col = ord(name[0]) - ord('A')
    row = int(name[1:]) - 1
    return col, row


@dataclass
class RouteExplorer:
    """Implementa RouteExplorer."""

    col: int
    row: int
    heading: str
    num_columns: int = 12
    num_rows: int = 8
    @classmethod
    def from_cell_name(cls, name: str, heading: str,
                       num_columns: int = 12, num_rows: int = 8):
        """Ejecuta from cell name."""
        col, row = cell_from_name(name)
        return cls(col=col, row=row, heading=heading,
                   num_columns=num_columns, num_rows=num_rows)

    @property
    def cell(self) -> tuple:
        """Ejecuta cell."""
        return (self.col, self.row)

    def girar(self, direccion: str) -> None:
        """Ejecuta girar."""
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
        """Ejecuta avanzar."""
        dx, dy = _DELTA[self.heading]
        new_col = max(0, min(self.num_columns - 1, self.col + dx))
        new_row = max(0, min(self.num_rows - 1, self.row + dy))
        self.col, self.row = new_col, new_row
        return (new_col, new_row)
