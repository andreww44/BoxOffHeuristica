#ifndef BOARD_H
#define BOARD_H

#include <cstdint>
#include <cstdlib>
#include <bitset>

typedef uint64_t U64;

enum MARK { RED, BLUE, YELLOW };

const int X = 8;
const int Y = 6;
const int BOARD_SIZE = 48;

enum DIR { R, L, D, U, RD, RU, LD, LU };

const int DIRECTIONS[8][2] = {
    {1, 0},   // Derecha
    {-1, 0},  // Izquierda
    {0, 1},   // Abajo
    {0, -1},  // Arriba
    {1, 1},   // Diagonal derecha-abajo
    {1, -1},  // Diagonal derecha-arriba
    {-1, 1},  // Diagonal izquierda-abajo
    {-1, -1}  // Diagonal izquierda-arriba
};

class Board {
    
   
   

    bool isLegalMove(int position, DIR direction);  

public:
    Board();    
    ~Board();    
    
    void reset();
    void print();  // Imprime el estado del tablero.
    bool endGame();  // Verifica si el juego ha terminado.
    
    bool areSameColor(int pos1, int pos2);   // Verifica si dos posiciones son del mismo color.
    bool isPathClear(int pos1, int pos2);    // Verifica si el camino entre dos posiciones está libre.
    bool isRectangleClear(int pos1, int pos2);  // Verifica si el rectángulo entre dos posiciones está libre.
    bool removePair(int pos1, int pos2);    // Elimina un par de piezas.
    bool hasValidMoves();

    U64 board[3]{};          // Representación de las piezas en el tablero.
    U64 oneMask{};           // Máscara para mover las piezas.
    U64 winingBoard{};       // Máscara de ganador.
    U64 fullMask{};          // Máscara completa para el tablero.
    U64 zero{};              // Representación de cero.
};

#endif //BOARD_H
