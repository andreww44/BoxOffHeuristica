#ifndef ALGORITMOS_HPP
#define ALGORITMOS_HPP

#include <queue>
#include <unordered_set>
#include <vector>
#include <iostream>
#include "Board.hpp"  // Asegúrate de que Board.hpp esté bien definido

// Definición de un nodo en la búsqueda
struct Node {
    U64 board[3];  // Representación del tablero con bitboards
    int depth;      // Profundidad (número de movimientos realizados)
    std::vector<std::pair<int, int>> moves; // Lista de movimientos realizados (pares eliminados)

    bool operator==(const Node& other) const;
};

namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const;
    };
}

// Definición de un nodo para A* (con heurística)
struct NodeAStar {
    U64 board[3];  // Representación del tablero con bitboards
    int depth;      // Profundidad (número de movimientos realizados)
    int heuristic;  // Heurística para la función A*
    std::vector<std::pair<int, int>> moves; // Lista de movimientos realizados

    bool operator==(const NodeAStar& other) const;
    bool operator>(const NodeAStar& other) const;
};

namespace std {
    template <>
    struct hash<NodeAStar> {
        size_t operator()(const NodeAStar& node) const;
    };
}

// Declaraciones de funciones de búsqueda
// Funciones de búsqueda y algoritmos
bool bfs(Board& board);   // Búsqueda por amplitud (Breadth-First Search)
int heuristic(const NodeAStar& node); // Heurística para A*
bool astar(Board& board); // A* Search
bool ida_star(Board& board); // IDA* Search

int search_ida(Board& board, NodeAStar startNode, int depth, int limit, int& nodesExplored);


#endif // ALGORITMOS_HPP
