
#include <iostream>
#include "Board.hpp"

int main() {
    std::cout << "Hello, CLion!" << std::endl;

    Board board;
    board.print();
    board.makeMove(36, D);
    board.print();
    return 0;
}

