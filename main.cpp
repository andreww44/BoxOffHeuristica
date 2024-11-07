#include "Board.hpp"
#include <iostream>
#include <string>
#include "Algoritmos.hpp"

void printMenu() {
    std::cout << "Seleccione una opción:\n";
    std::cout << "1. Jugar normalmente\n";
    std::cout << "2. Usar BFS\n";
    std::cout << "3. Usar A*\n";
    std::cout << "4. Usar IDA*\n";
    std::cout << "5. Salir\n";
    std::cout << "Ingrese su opción (1-5): ";
}

int main() {
    int option;
    std::cout << "Bienvenido al juego de eliminación de fichas!\n";
    printMenu();
    std::cin >> option;

    while (option != 5) {
        Board board;
        board.print();

        if (option == 1) {
            // Juego normal
            while (true) {
                int pos1, pos2;

                // Solicita al usuario que ingrese las posiciones de las fichas que desea eliminar
                std::cout << "Ingresa las posiciones de las fichas que deseas eliminar (o -1 para salir): ";
                std::cin >> pos1;
                if (pos1 == -1) break; // Permite salir del bucle al ingresar -1
                std::cin >> pos2;

                // Intenta eliminar el par de fichas
                if (board.removePair(pos1, pos2)) {
                    std::cout << "Par eliminado correctamente.\n";
                } else {
                    std::cout << "No se pudo eliminar el par en las posiciones especificadas.\n";
                }

                // Muestra el tablero después del intento de eliminación
                board.print();

                // Verifica si el juego ha terminado
                if (board.endGame()) {
                    std::cout << "Juego terminado. ¡Has ganado!\n";
                    break;
                } else {
                    std::cout << "El juego continúa.\n";
                }
            }
        } else if (option == 2) {
            // Usar BFS
            if (bfs(board)) {
                std::cout << "Solución encontrada con BFS.\n";
            } else {
                std::cout << "No se encontró solución con BFS.\n";
            }
        } else if (option == 3) {
            // Usar A*
            if (astar(board)) {
                std::cout << "Solución encontrada con A*.\n";
            } else {
                std::cout << "No se encontró solución con A*.\n";
            }
        } else if (option == 4) {
            // Usar IDA*
            if (ida_star(board)) {
                std::cout << "Solución encontrada con IDA*.\n";
            } else {
                std::cout << "No se encontró solución con IDA*.\n";
            }
        }

        // Mostrar el menú nuevamente después de completar un ciclo de búsqueda o juego normal
        board.reset();
        printMenu();
        std::cin >> option;
    }

    std::cout << "Gracias por jugar. ¡Hasta la próxima!\n";
    return 0;
}
