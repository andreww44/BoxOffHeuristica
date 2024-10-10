//
// Created by monsa on 06/10/2024.
//

#include "Board.hpp"

#include <iostream>


Board::Board()
{
    int i = 0;
    board[RED] = 0x0000000000000000ULL;
    board[BLUE] = 0x0000000000000000ULL;
    board[YELLOW] = 0x0000000000000000ULL;
    oneMask = 0x0000000000000001ULL;
    zero = 0x0000000000000000ULL;
    fullMask = (0x0000ffffffffffffULL);
    winingBoard = ~fullMask;
    int colors[3] = {0, 0, 0};
    srand (time(NULL));
    while (i < BOARD_SIZE)
    {
        int a = rand() % 3;
        std::cout << i << std::endl;
        colors[a]++;
        if(colors[a] <= 16)
        {
            board[a] |= (oneMask << i);
            i++;
        }
    }
    std::bitset<64> bits(fullMask);
    std::cout <<  bits << std::endl;
}
Board::~Board() = default;

bool Board::endGame()
{
    U64 end = board[RED] | board[BLUE] | board[YELLOW];
    if(end & winingBoard == zero){
        return true;
    }
    return false;
}


bool Board::isLegalMove(int position, DIR direction)
{

    return true;
}

void Board::print()
{
    U64 rTable = board[RED];
    U64 bTable = board[BLUE];
    U64 yTable = board[YELLOW];

    std::cout << "BoxOff" << std::endl;
    for (int i = 0; i < Y; i++) {
        for(int j = 0; j < X; j++){
            if(rTable&(oneMask << (j+ (i*Y)))){
                std::cout<<"R ";
            }
            else if(bTable&(oneMask << (j+ (i*Y)))){
                std::cout<<"B ";
            }
            else if(yTable&(oneMask << (j+ (i*Y)))){
                std::cout<<"Y ";
            }
            else{
                std::cout<< "* ";
            }
        }
        std::cout<< "\n";
    }
}

bool Board::makeMove(int position, DIR direction)
{
    if(position < 0 && position >= BOARD_SIZE){
        return false;
    }
    else if(isLegalMove(position, direction))
    {
        //Hola Mundo
        int index = 0;
        for (int i = 0; i < 3; i++)
        {
            if((board[i] & oneMask << position))
            {
                index = i;
                break;
            }
        }
        int px = position/X;
        int py = position % X;
        int nx = (px+DIRECTIONS[direction][0]);
        int ny = (py+DIRECTIONS[direction][1]);
        std::cout << position << " " << ((nx*X)+ny) << " " << nx << " " << ny << std::endl;
        board[index] &= ~(oneMask << position);
        board[index] &= ~(oneMask << ((nx*X)+ny));
        return true;
    }

    //return true;
    return false;
}
