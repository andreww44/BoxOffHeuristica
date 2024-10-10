//
// Created by monsa on 06/10/2024.
//

#ifndef BOARD_H
#define BOARD_H


#include <cstdint>
#include <cstdlib>
#include <bitset>
#include <vector>

typedef uint64_t U64;
enum MARK{RED, BLUE, YELLOW };

const int X = 8;
const int Y = 6;
const int BOARD_SIZE = 48;

enum DIR {R, L, D, U, RD, RU, LD, LU};
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

    U64 board[3]{};
    U64 maskBoard[3]{};
    U64 oneMask{};
    U64 winingBoard{};
    U64 fullMask{};
    U64 zero;

    static bool isLegalMove(int position, DIR direction); //Verificar si la pos es legal.

public:
    Board(); //Constructor prototipe
    ~Board(); //Destructor prototipo
    //Board(uint16_t red, uint16_t blue, uint64_t yellow);
    bool makeMove(int position, DIR direction); //Permite mover
    //void undoMove(int position);
    void print(); //Imprime el tablero
    //int evaluateBoard(int depth);
    bool endGame();

};



#endif //BOARD_H
