#include "Board.hpp"
#include <iostream>
#include <ctime>


Board::Board() {
    int i = 0;
    board[RED] = 0x0000242424244949ULL;
    board[BLUE] = 0x0000929249499292ULL;
    board[YELLOW] = 0x0000494992924444ULL;
    oneMask = 0x0000000000000001ULL;
    zero = 0x0000000000000000ULL;
    fullMask = (0x0000ffffffffffffULL);
    winingBoard = ~fullMask;
    int colors[3] = {0, 0, 0};
    srand(time(NULL));

    // Colores aleatorios para el tablero.
    
}

Board::~Board() = default;

bool Board::endGame() {
    // tablero vacio, el jugador gana.
    if ((board[RED] | board[BLUE] | board[YELLOW]) == zero) {
        std::cout << "¡Has ganado! El tablero está vacío." << std::endl;
        return true;
    }

    // no hay más movimientos validos, el juego termina.
    if (!hasValidMoves()) {
        std::cout << "No hay más movimientos válidos. El juego ha terminado." << std::endl;
        return true;
    }

    return false;  
}

bool Board::isLegalMove(int position, DIR direction) {
    int px = position % X;
    int py = position / X;
    int nx = px + DIRECTIONS[direction][0];
    int ny = py + DIRECTIONS[direction][1];

    if (nx >= 0 && nx < X && ny >= 0 && ny < Y) {
        int newPos = (nx * X) + ny;
        for (int i = 0; i < 3; i++) {
            if (board[i] & (oneMask << newPos)) {
                return false;  
            }
        }
        return true;
    }
    return false; 
}

void Board::reset(){
    board[RED] = 0x0000242424244949ULL;
    board[BLUE] = 0x0000929249499292ULL;
    board[YELLOW] = 0x0000494992924444ULL;
}

void Board::print() {
    U64 rTable = board[RED];
    U64 bTable = board[BLUE];
    U64 yTable = board[YELLOW];

    std::cout << "BoxOff" << std::endl;
    for (int i = 0; i < Y; i++) {
        for (int j = 0; j < X; j++) {
            int pos = i * X + j;
            if (rTable & (oneMask << pos)) {
                std::cout << "R ";
            }
            else if (bTable & (oneMask << pos)) {
                std::cout << "B ";
            }
            else if (yTable & (oneMask << pos)) {
                std::cout << "Y ";
            }
            else {
                std::cout << "* ";
            }
        }
        std::cout << "\n";
    }
}

bool Board::areSameColor(int pos1, int pos2) {
    
    for (int i = 0; i < 3; i++) {
        if ((board[i] & (oneMask << pos1)) && (board[i] & (oneMask << pos2))) {
            return true;  
        }
    }
    return false;
}

bool Board::isPathClear(int pos1, int pos2) {
    int x1 = pos1 % X;
    int y1 = pos1 / X;
    int x2 = pos2 % X;
    int y2 = pos2 / X;

    // Calcula las direcciones
    int dx = (x2 > x1) ? 1 : (x2 < x1) ? -1 : 0;
    int dy = (y2 > y1) ? 1 : (y2 < y1) ? -1 : 0;

    if (pos1 == pos2) return true;

    // Movimiento vertical
    if (dx == 0) {
        // Verifica si el camino en la columna está despejado
        for (int y = std::min(y1, y2) + 1; y < std::max(y1, y2); ++y) {
            int currentPos = y * X + x1;
            if ((board[RED] & (oneMask << currentPos)) || 
                (board[BLUE] & (oneMask << currentPos)) || 
                (board[YELLOW] & (oneMask << currentPos))) {
                return false;
            }
        }
    }
    // Movimiento horizontal 
    else if (dy == 0) {
        // Verifica si el camino en la fila está despejado
        for (int x = std::min(x1, x2) + 1; x < std::max(x1, x2); ++x) {
            int currentPos = y1 * X + x;
            if ((board[RED] & (oneMask << currentPos)) || 
                (board[BLUE] & (oneMask << currentPos)) || 
                (board[YELLOW] & (oneMask << currentPos))) {
                return false;
            }
        }
    }
    // Si el movimiento es diagonal (dx != 0 y dy != 0)
    else {
        //verifica las posiciones intermedias
        int currentX = x1 + dx;
        int currentY = y1 + dy;

        // Recorre las posiciones intermedias
        while (currentX != x2 && currentY != y2) {
            int currentPos = currentY * X + currentX;
            if ((board[RED] & (oneMask << currentPos)) ||
                (board[BLUE] & (oneMask << currentPos)) ||
                (board[YELLOW] & (oneMask << currentPos))) {
                return false;
            }
            currentX += dx;
            currentY += dy;
        }
    }

    // retorna que el camino esta despejado en caso de no haber obstaculos
    return true;
}

bool Board::isRectangleClear(int pos1, int pos2) 
{
    int x1 = pos1 % X;
    int y1 = pos1 / X;
    int x2 = pos2 % X;
    int y2 = pos2 / X;

    // Define los límites del rectángulo

    int startX = std::min(x1, x2);
    int endX = std::max(x1, x2);
    int startY = std::min(y1, y2);
    int endY = std::max(y1, y2);

    // Recorre el área dentro del rectángulo, excluyendo las posiciones en `pos1` y `pos2`
    for (int i = startY; i <= endY; i++)
    {
        for (int j = startX; j <= endX; j++)
        {
            int currentPos = i * X + j;
            // Excluir las posiciones seleccionadas `pos1` y `pos2`
            if (currentPos != pos1 && currentPos != pos2) 
            {
                // Si hay una ficha en `currentPos`, el rectángulo no está vacío

                if ((board[RED] & (oneMask << currentPos)) ||
                    (board[BLUE] & (oneMask << currentPos)) ||
                    (board[YELLOW] & (oneMask << currentPos))) 
                    {
                    return false;  // Hay una pieza dentro del rectángulo
                    }
            }
        }
    }

    return true;  // Área despejada
}

bool Board::removePair(int pos1, int pos2) {
    // Verifica que la función se llama y si las posiciones son válidas
    if (pos1 < 0 || pos1 >= BOARD_SIZE || pos2 < 0 || pos2 >= BOARD_SIZE) 
    {
        return false;  // Posiciones fuera del rango.
    }

    // Intenta eliminar el par de fichas en las posiciones especificadas
    if (areSameColor(pos1, pos2) && isPathClear(pos1, pos2) && isRectangleClear(pos1, pos2)) 
    {
        for (int i = 0; i < 3; i++) 
        {
            if ((board[i] & (oneMask << pos1)) && (board[i] & (oneMask << pos2))) 
            {
                board[i] &= ~(oneMask << pos1);
                board[i] &= ~(oneMask << pos2);
                return true;
            }
        }
    }

    return false;
}

bool Board::hasValidMoves() 
{
    // Recorre todas las posiciones del tablero
    for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
    {
        // Verifica si la posición tiene alguna ficha (en cualquiera de los colores)
        if ((board[RED] & (oneMask << pos1)) || (board[BLUE] & (oneMask << pos1)) || (board[YELLOW] & (oneMask << pos1))) 
        {
            
            // Compara con todas las otras posiciones para encontrar pares
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
            {
                // Verifica si la posición 2 también tiene una ficha
                if ((board[RED] & (oneMask << pos2)) || (board[BLUE] & (oneMask << pos2)) || (board[YELLOW] & (oneMask << pos2))) 
                {
                    
                    // Verifica si ambas fichas son del mismo color
                    if (areSameColor(pos1, pos2)) 
                    {
                        // Verifica si el camino entre las posiciones está despejado y 
                        //el rectángulo que forman es válido
                        if (isPathClear(pos1, pos2) && isRectangleClear(pos1, pos2)) 
                        {
                            return true;  // Si se puede eliminar el par, el juego continúa
                        }
                    }
                }
            }
        }
    }
    return false;  // Si no se encuentran pares válidos, el juego termina
}
