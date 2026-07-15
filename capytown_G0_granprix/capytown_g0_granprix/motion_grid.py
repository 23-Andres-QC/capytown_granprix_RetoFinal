"""Módulo motion_grid."""

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


def cell_name(col: int, row: int) -> str:
    """Ejecuta cell name."""
    letter = chr(ord('A') + col)
    return f'{letter}{row + 1}'


def cell_from_name(name: str) -> tuple:
    """Ejecuta cell from name."""
    name = name.strip().upper()
    col = ord(name[0]) - ord('A')
    row = int(name[1:]) - 1
    return col, row


@dataclass
class GridTracker:
    """Implementa GridTracker."""

    col: int
    row: int
    heading: str
    num_columns: int = 6
    num_rows: int = 4

    @classmethod
    def from_cell_name(cls, name: str, heading: str, num_columns: int = 6, num_rows: int = 4):
        """Ejecuta from cell name."""
        col, row = cell_from_name(name)
        return cls(col=col, row=row, heading=heading, num_columns=num_columns, num_rows=num_rows)

    @property
    def cell(self) -> str:
        """Ejecuta cell."""
        return cell_name(self.col, self.row)

    def advance_cell(self) -> None:
        """Ejecuta advance cell."""
        dx, dy = _DELTA[self.heading]
        new_col = self.col + dx
        new_row = self.row + dy
        self.col = max(0, min(self.num_columns - 1, new_col))
        self.row = max(0, min(self.num_rows - 1, new_row))

    def apply_turn(self, direction: str) -> None:
        """Ejecuta apply turn."""
        if direction == 'DERECHA':
            self.heading = turn_right(self.heading)
        elif direction == 'IZQUIERDA':
            self.heading = turn_left(self.heading)
        elif direction == 'ATRAS':
            self.heading = turn_180(self.heading)
        elif direction == 'NINGUNO':
            return
        else:
            raise ValueError(f'direccion de giro desconocida: {direction}')

