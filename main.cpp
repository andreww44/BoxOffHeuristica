#include "Board.hpp"
#include <iostream>
#include <string>
#include "Algoritmos.hpp"

void printMenu() {
    std::cout << "Seleccione un Algoritmo:\n";
    std::cout << "1. Jugar normalmente\n";
    std::cout << "2. Usar BFS\n";
    std::cout << "3. Usar A*\n";
    std::cout << "4. Usar IDA*\n";
    std::cout << "5. Usar DFS\n";
    std::cout << "6. Usar Dijkstra\n";
    std::cout << "7. Usar WeightedA*\n";
    std::cout << "8. Usar Beam Search\n";
    std::cout << "9. Usar RBFS\n";             
    std::cout << "10. Usar SMA*\n";  
    std::cout << "11. Salir\n";
    std::cout << "Ingrese su opción (1-11): ";
}

int main() {
    int option;
    std::cout << "Bienvenido al juego de eliminación de fichas!\n";
    //printMenu();
    
    while (option != 11) 
    {
        Board board;
 
        board.restartBoard();
        printMenu();
        board.print();
        std::cin >> option;

        if (option == 1) 
        {
            // Juego normal
            while (true) 
            {
                int pos1, pos2;

                // Solicita al usuario que ingrese las posiciones de las fichas que desea eliminar
                std::cout << "Ingresa las posiciones de las fichas que deseas eliminar (o -1 para salir): ";
                std::cin >> pos1;
                if (pos1 == -1) break; // Permite salir del bucle al ingresar -1
                std::cin >> pos2;

                // Intenta eliminar el par de fichas
                if (board.removePair(pos1, pos2))
                {
                    std::cout << "Par eliminado correctamente.\n";
                } 
                else 
                {
                    std::cout << "No se pudo eliminar el par en las posiciones especificadas.\n";
                }

                // Muestra el tablero después del intento de eliminación
                board.print();

                // Verifica si el juego ha terminado
                if (board.endGame())
                {
                    std::cout << "Juego terminado. ¡Has ganado!\n";
                    break;
                } 
                else 
                {
                    std::cout << "El juego continúa.\n";
                }
            }
        } 
        
        else if (option == 2) 
        {
            // Usar BFS
            if (bfs(board)) {
                std::cout << "Solución encontrada con BFS.\n";
            } else {
                std::cout << "No se encontró solución con BFS.\n";
            }
        } 

        else if (option == 3) 
        {
            // Usar A*
            if (astar(board)) {
                std::cout << "Solución encontrada con A*.\n";
            } else {
                std::cout << "No se encontró solución con A*.\n";
            }
        } 

        else if (option == 4) 
        {
            // Usar IDA*
            if (ida_star(board)) {
                std::cout << "Solución encontrada con IDA*.\n";
            } else {
                std::cout << "No se encontró solución con IDA*.\n";
            }
        } 

        else if(option == 5)
        {

            if(start_dfs(board)){
                std::cout << "Solución encontrada con DFS*.\n";
            } else {
                std::cout << "No se encontró solución con DFS*.\n";
            }
        } 

        else if(option == 6)
        {

            if(Dijkstra(board)){
                std::cout << "Solución encontrada con Dijsktra*.\n";
            } else {
                std::cout << "No se encontró solución con Dijkstra*.\n";
            }
        } 

        else if(option == 7)
        {

            if(WeightedAStar(board, 3)){
                std::cout << "Solución encontrada con WeightedA*.\n";
            } else {
                std::cout << "No se encontró solución con WeightedA*.\n";
            }
        }

        else if (option == 8) 
        {
            // Usar Beam Search
            int beamWidth;
            std::cout << "Ingrese el ancho de haz para Beam Search: ";
            std::cin >> beamWidth;
            if (BeamSearch(board, beamWidth)) {
                std::cout << "Solución encontrada con Beam Search.\n";
            } else {
                std::cout << "No se encontró solución con Beam Search.\n";
            }
        } 
        else if (option == 9) 
        {
            // Usar RBFS
            if (start_rbfs(board)) {
                std::cout << "Solución encontrada con RBFS.\n";
            } else {
                std::cout << "No se encontró solución con RBFS.\n";
            }
        } 
        else if (option == 10) 
        {
            // Usar SMA*
            int maxMemory;
            std::cout << "Ingrese el límite de memoria para SMA*: ";
            std::cin >> maxMemory;
            if (SMAStar(board, maxMemory)) {
                std::cout << "Solución encontrada con SMA*.\n";
            } else {
                std::cout << "No se encontró solución con SMA*.\n";
            }
        }
    

        // Mostrar el menú nuevamente después de completar un ciclo de búsqueda o juego normal
        
    }

    std::cout << "Gracias por jugar. ¡Hasta la próxima!\n";
    return 0;
}
