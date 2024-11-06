#include "Algoritmos.hpp"
#include <cmath>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <climits>
#include <chrono>  // Para medir tiempo
#include <sys/resource.h>  // Para medir uso de memoria
using namespace std;

const int FOUND = 1;
const U64 oneMask = 1;
const int MAX_COST = 1000000;

auto start_time = chrono::high_resolution_clock::now();  // Tiempo de inicio global
auto start_memory = rusage();  // Uso de memoria

// Función para obtener el tiempo de ejecución en segundos
double get_execution_time() {
    auto end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end_time - start_time;
    return duration.count();
}

// Función para obtener el consumo de memoria (en bytes)
size_t get_memory_usage() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss;  // Retorna la memoria máxima usada (en kilobytes)
}

// operador de comparación de Nodo
bool Node::operator==(const Node& other) const {
    return board[RED] == other.board[RED] && 
           board[BLUE] == other.board[BLUE] &&
           board[YELLOW] == other.board[YELLOW];
}

// operador de hash para Node
size_t std::hash<Node>::operator()(const Node& node) const {
    size_t h1 = std::hash<U64>{}(node.board[RED]);
    size_t h2 = std::hash<U64>{}(node.board[BLUE]);
    size_t h3 = std::hash<U64>{}(node.board[YELLOW]);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

// operador de comparación NodeAStar
bool NodeAStar::operator==(const NodeAStar& other) const {
    return board[RED] == other.board[RED] && 
           board[BLUE] == other.board[BLUE] &&
           board[YELLOW] == other.board[YELLOW];
}

// operador de comparación mayor NodeAStar
bool NodeAStar::operator>(const NodeAStar& other) const {
    return (depth + heuristic) > (other.depth + other.heuristic);
}

// operador de hash NodeAStar
size_t std::hash<NodeAStar>::operator()(const NodeAStar& node) const {
    size_t h1 = std::hash<U64>{}(node.board[RED]);
    size_t h2 = std::hash<U64>{}(node.board[BLUE]);
    size_t h3 = std::hash<U64>{}(node.board[YELLOW]);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

// heurística para A*
int heuristic(const NodeAStar& node) {
    int remainingPieces = 0;
    for (int i = 0; i < 3; i++) {
        remainingPieces += __builtin_popcountll(node.board[i]);
    }
    return remainingPieces;
}

bool bfs(Board& board) {
    queue<Node> frontier;
    unordered_set<Node> explored;
    
    Node startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, {}};
    frontier.push(startNode);
    explored.insert(startNode);

    int nodesExplored = 0;

    while (!frontier.empty()) {
        Node currentNode = frontier.front();
        frontier.pop();
        nodesExplored++;

        // Verificar si hemos alcanzado la solución (tablero vacío)
        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) {
            cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
            break;
        }

        // Generar los movimientos posibles y agregarlos a la cola
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                    Node newNode = currentNode;
                    // Eliminar las fichas en las posiciones
                    for (int i = 0; i < 3; i++) {
                        if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) {
                            newNode.board[i] &= ~(oneMask << pos1);
                            newNode.board[i] &= ~(oneMask << pos2);
                        }
                    }
                    newNode.moves.push_back({pos1, pos2});
                    newNode.depth++;

                    if (explored.find(newNode) == explored.end()) {
                        frontier.push(newNode);
                        explored.insert(newNode);
                    }
                }
            }
        }
    }

    double timeElapsed = get_execution_time();
    cout << "Tiempo total: " << timeElapsed << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / timeElapsed << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}

bool astar(Board& board) {
    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> frontier;
    unordered_set<NodeAStar> explored;

    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}};
    frontier.push(startNode);
    explored.insert(startNode);

    int nodesExplored = 0;

    while (!frontier.empty()) {
        NodeAStar currentNode = frontier.top();
        frontier.pop();
        nodesExplored++;

        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) {
            cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
            break;
        }

        // Generar los movimientos posibles y agregarlos a la cola
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                    NodeAStar newNode = currentNode;
                    // Eliminar las fichas en las posiciones
                    for (int i = 0; i < 3; i++) {
                        if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) {
                            newNode.board[i] &= ~(oneMask << pos1);
                            newNode.board[i] &= ~(oneMask << pos2);
                        }
                    }
                    newNode.moves.push_back({pos1, pos2});
                    newNode.depth++;
                    newNode.heuristic = heuristic(newNode);

                    if (explored.find(newNode) == explored.end()) {
                        frontier.push(newNode);
                        explored.insert(newNode);
                    }
                }
            }
        }
    }

    double timeElapsed = get_execution_time();
    cout << "Tiempo total: " << timeElapsed << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / timeElapsed << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}

int search_ida(Board& board, NodeAStar startNode, int depth, int limit, int& nodesExplored) {
    // Si la profundidad actual es mayor que el límite, retornamos el valor de la profundidad como el nuevo límite
    if (depth > limit) return depth;

    // Si hemos encontrado la solución (tablero vacío), retornamos FOUND
    if ((startNode.board[RED] | startNode.board[BLUE] | startNode.board[YELLOW]) == 0) {
        return FOUND;
    }

    int minCost = INT_MAX;  // Este valor llevará el mínimo costo encontrado

    // Generar todos los movimientos posibles (de manera similar a lo que ya haces en A*)
    for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) {
        for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) {
            // Verificamos si las fichas en las posiciones son del mismo color y si el camino está despejado
            if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                NodeAStar newNode = startNode;  // Usa NodeAStar en lugar de Node

                // Realizamos el movimiento: eliminamos las piezas de las posiciones
                for (int i = 0; i < 3; i++) {
                    if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) {
                        newNode.board[i] &= ~(oneMask << pos1);
                        newNode.board[i] &= ~(oneMask << pos2);
                    }
                }

                // Actualizamos los movimientos y la profundidad
                newNode.moves.push_back({pos1, pos2});
                newNode.depth = depth + 1;

                // Actualizamos el número de nodos explorados
                nodesExplored++;

                // Llamamos recursivamente a search_ida con la nueva configuración
                int result = search_ida(board, newNode, depth + 1, limit, nodesExplored);

                if (result == FOUND) {
                    return FOUND;  // Si encontramos la solución, terminamos
                }

                if (result < minCost) {
                    minCost = result;  // Actualizamos el menor costo encontrado
                }
            }
        }
    }

    return minCost;  // Retornamos el menor costo encontrado en este nivel
}

bool ida_star(Board& board) {
    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}}; 
    int limit = heuristic(startNode);  
    int nodesExplored = 0;

    while (true) {
        int temp = search_ida(board, startNode, 0, limit, nodesExplored);  
        if (temp == FOUND) {
            cout << "Solución encontrada." << endl;
            break;
        }
        if (temp == INT_MAX) {
            cout << "No se encontró solución." << endl;
            return false;
        }
        limit = temp;  // Incrementa el límite de profundidad
    }

    double timeElapsed = get_execution_time();
    cout << "Tiempo total: " << timeElapsed << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / timeElapsed << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}
