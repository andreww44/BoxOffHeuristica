#include "Algoritmos.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <climits>
#include <chrono> 
#include <stack>
#include <unordered_map>
#include <sys/resource.h>  
#include <string>

using namespace std;
using namespace std::chrono;

const int FOUND = 1;
const U64 oneMask = 1;
const int MAX_COST = 1000000;

auto start_time = chrono::high_resolution_clock::now();  // Tiempo de inicio global
auto start_memory = rusage();  // Uso de memoria

// Función para obtener el tiempo de ejecución en segundos
double get_execution_time() 
{
    auto end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end_time - start_time;
    return duration.count();
}

// Función para obtener el consumo de memoria (en bytes)
size_t get_memory_usage() 
{
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss;  // Retorna la memoria máxima usada (en kilobytes)
}

// operador de comparación de Nodo
bool Node::operator==(const Node& other) const 
{
    return board[RED] == other.board[RED] && 
           board[BLUE] == other.board[BLUE] &&
           board[YELLOW] == other.board[YELLOW];
}

// operador de hash para Node
size_t std::hash<Node>::operator()(const Node& node) const 
{
    size_t h1 = std::hash<U64>{}(node.board[RED]);
    size_t h2 = std::hash<U64>{}(node.board[BLUE]);
    size_t h3 = std::hash<U64>{}(node.board[YELLOW]);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

// operador de comparación NodeAStar
bool NodeAStar::operator==(const NodeAStar& other) const 
{
    return board[RED] == other.board[RED] && 
           board[BLUE] == other.board[BLUE] &&
           board[YELLOW] == other.board[YELLOW];
}

// operador de comparación mayor NodeAStar
bool NodeAStar::operator>(const NodeAStar& other) const 
{
    return (depth + heuristic) > (other.depth + other.heuristic);
}

void NodeAStar::print(){
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

void NodeDijkstra::print(){
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

// operador de hash NodeAStar
size_t std::hash<NodeAStar>::operator()(const NodeAStar& node) const 
{
    size_t h1 = std::hash<U64>{}(node.board[RED]);
    size_t h2 = std::hash<U64>{}(node.board[BLUE]);
    size_t h3 = std::hash<U64>{}(node.board[YELLOW]);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

// heurística para A*
int heuristic(const NodeAStar& node) 
{
    int remainingPieces = 0;
    for (int i = 0; i < 3; i++) 
    {
        remainingPieces += __builtin_popcountll(node.board[i]);
    }
    return remainingPieces;
}

//Node Dijsktra
bool NodeDijkstra::operator>(const NodeDijkstra& other) const 
{
    return cost > other.cost;
}

bool NodeDijkstra::operator==(const NodeDijkstra& other) const 
{
    return board[RED] == other.board[RED] && 
           board[BLUE] == other.board[BLUE] &&
           board[YELLOW] == other.board[YELLOW];
}

//Funcionando
bool bfs(Board& board) 
{
    queue<Node> frontier;
    unordered_set<Node> explored;
    
    Node startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, {}};
    frontier.push(startNode);
    explored.insert(startNode);

    int nodesExplored = 0;
    auto start_time = chrono::high_resolution_clock::now();
    while (!frontier.empty()) 
    {
        Node currentNode = frontier.front();
        frontier.pop();
        nodesExplored++;

        // Verificar si hemos alcanzado la solución (tablero vacío)
        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
        {
            cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
            break;
        }

        // Generar los movimientos posibles y agregarlos a la cola
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
        {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
            {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) 
                {
                    Node newNode = currentNode;
                    // Eliminar las fichas en las posiciones
                    for (int i = 0; i < 3; i++) 
                    {
                        if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) 
                        {
                            newNode.board[i] &= ~(oneMask << pos1);
                            newNode.board[i] &= ~(oneMask << pos2);
                        }
                    }
                    newNode.moves.push_back({pos1, pos2});
                    newNode.depth++;

                    if (explored.find(newNode) == explored.end()) 
                    {
                        frontier.push(newNode);
                        explored.insert(newNode);
                    }
                }
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time); // Tiempo en milisegundos
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}

//Funcionando
bool astar(Board& board) 
{
    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> frontier;
    unordered_set<NodeAStar> explored;

    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}};
    frontier.push(startNode);
    explored.insert(startNode);

    int nodesExplored = 0;
    auto start_time = chrono::high_resolution_clock::now();
    while (!frontier.empty()) 
    {
        NodeAStar currentNode = frontier.top();
        frontier.pop();
        nodesExplored++;

        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
        {
            cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
            break;
        }

        // Generar los movimientos posibles y agregarlos a la cola
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
        {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
            {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                    NodeAStar newNode = currentNode;
                    // Eliminar las fichas en las posiciones
                    for (int i = 0; i < 3; i++) 
                    {
                        if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) 
                        {
                            newNode.board[i] &= ~(oneMask << pos1);
                            newNode.board[i] &= ~(oneMask << pos2);
                        }
                    }
                    newNode.moves.push_back({pos1, pos2});
                    newNode.depth++;
                    newNode.heuristic = heuristic(newNode);

                    if (explored.find(newNode) == explored.end()) 
                    {
                        frontier.push(newNode);
                        explored.insert(newNode);
                    }
                }
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time); // Tiempo en milisegundos
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}

//Funcionando
std::unordered_set<NodeAStar> visitedStates;
int search_ida(Board& board, NodeAStar startNode, int depth, int limit, int& nodesExplored) 
{
    if (depth > limit) {
        return depth;
    }
    if ((startNode.board[RED] | startNode.board[BLUE] | startNode.board[YELLOW]) == 0) {
        return FOUND;
    }
    if (visitedStates.count(startNode)) {
        return INT_MAX;  
    }

    visitedStates.insert(startNode);

    int minCost = INT_MAX;  // Este valor llevará el mínimo costo encontrado
    // Generar todos los movimientos posibles
    for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
    {
        for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
        {
            // Verificamos si las fichas en las posiciones son del mismo color y si el camino está despejado
            if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                NodeAStar newNode = startNode;

                // Realizamos el movimiento: eliminamos las piezas de las posiciones
                for (int i = 0; i < 3; i++) 
                {
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
bool ida_star(Board& board) 
{
    auto start_time = chrono::high_resolution_clock::now();
    getrusage(RUSAGE_SELF, &start_memory);                   

    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}}; 
    int limit = heuristic(startNode);
    std::cout<< "Limit = " << limit << std::endl;
    int nodesExplored = 0;

    while (true) 
    {
        int temp = search_ida(board, startNode, 0, limit, nodesExplored); 

        if (temp == FOUND) 
        {
            cout << "Solución encontrada." << endl;
            break;
        }
        if (temp == INT_MAX) {
            cout << "No se encontró solución." << endl;
            return false;
        }

        limit = temp;  // Incrementa el límite de profundidad
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time); // Tiempo en milisegundos
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}

// Implementación de DFS recursivo
bool dfs(Board& board, Node& currentNode, unordered_set<Node>& visitedNodes, int depth, int& nodesExplored) 
{
    // Verificar si hemos alcanzado la solución (tablero vacío)
    if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
    {
        cout << "Solución encontrada en " << depth << " movimientos!" << endl;
        return true;
    }

    // Añadir el nodo actual al conjunto de nodos visitados para evitar ciclos
    visitedNodes.insert(currentNode);
    nodesExplored++;

    // Generar los movimientos posibles
    for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
    {
        for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
        {
            if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) 
            {
                // Crear un nuevo nodo con el estado resultante de aplicar el movimiento
                Node newNode = currentNode;

                // Eliminar las fichas en las posiciones
                for (int i = 0; i < 3; i++) 
                {
                    if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) 
                    {
                        newNode.board[i] &= ~(oneMask << pos1);
                        newNode.board[i] &= ~(oneMask << pos2);
                    }
                }

                // Verificar si el nuevo nodo no ha sido visitado
                if (visitedNodes.find(newNode) == visitedNodes.end()) 
                {
                    // Llamada recursiva a DFS con el nuevo nodo
                    if (dfs(board, newNode, visitedNodes, depth + 1, nodesExplored)) 
                    {
                        return true;
                    }
                }
            }
        }
    }

    return false;  // Si no se encuentra solución en este camino
}

bool start_dfs(Board& board) 
{
    unordered_set<Node> visitedNodes;  // Conjunto de nodos visitados para evitar ciclos
    Node startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, {}};  // Nodo inicial
    int nodesExplored = 0;

    auto start_time = chrono::high_resolution_clock::now();
    
    getrusage(RUSAGE_SELF, &start_memory);

    // Llamada inicial a la función DFS
    bool solutionFound = dfs(board, startNode, visitedNodes, 0, nodesExplored);

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time); // Tiempo en milisegundos
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return solutionFound;
}

// Función de Dijkstra para encontrar el camino de menor costo
bool Dijkstra(Board& board) 
{
    // Cola de prioridad para los nodos a explorar (min-heap)
    std::priority_queue<NodeDijkstra, std::vector<NodeDijkstra>, std::greater<NodeDijkstra>> pq;

    // Almacena el costo mínimo de cada nodo para evitar expansiones innecesarias
    std::unordered_map<NodeDijkstra, int> minCost;
    
    NodeDijkstra startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, {}}; // Nodo inicial
    pq.push(startNode);
    minCost[startNode] = 0;

    int nodesExplored = 0;

    struct rusage start_memory;
    getrusage(RUSAGE_SELF, &start_memory);
    auto start_time = high_resolution_clock::now();
    while (!pq.empty()) 
    {
        // Extraemos el nodo con el menor costo
        
        NodeDijkstra currentNode = pq.top();
        //currentNode.print();
        pq.pop();

        // Comprobamos si hemos alcanzado la solución (tablero vacío)
        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
        {
            std::cout << "Solución encontrada con un costo de " << currentNode.cost << "!" << std::endl;
            std::cout << "Movimientos realizados: " << currentNode.path.size() << std::endl;
            return true;
        }

        nodesExplored++;

        // Generar los movimientos posibles
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
        {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
            {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) 
                {
                    // Crear un nuevo nodo con el estado resultante de aplicar el movimiento
                    NodeDijkstra newNode = currentNode;
                    newNode.cost = currentNode.cost + 1; // Incrementamos el costo en 1 para este movimiento
                    newNode.path.push_back(pos1); // Opcional: guardar el movimiento en el camino
                    newNode.path.push_back(pos2);

                    // Eliminar las fichas en las posiciones
                    for (int i = 0; i < 3; i++) 
                    {
                        if ((newNode.board[i] & (1 << pos1)) && (newNode.board[i] & (1 << pos2))) 
                        {
                            newNode.board[i] &= ~(1 << pos1);
                            newNode.board[i] &= ~(1 << pos2);
                        }
                    }

                    // Si encontramos un camino más corto a este nuevo nodo, lo añadimos a la cola
                    if (minCost.find(newNode) == minCost.end() || newNode.cost < minCost[newNode]) 
                    {
                        pq.push(newNode);
                        minCost[newNode] = newNode.cost;
                    }
                }
            }
        }
    }

    // Mostrar resultados
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time); // Tiempo en milisegundos
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    struct rusage end_memory;
    getrusage(RUSAGE_SELF, &end_memory);
    long memory_used = end_memory.ru_maxrss - start_memory.ru_maxrss;
    std::cout << "Memoria máxima utilizada: " << memory_used << " KB." << std::endl;

    return false;
}

bool WeightedAStar(Board& board, double weight) 
{
    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> frontier;
    unordered_set<NodeAStar> explored;

    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}};
    frontier.push(startNode);
    explored.insert(startNode);

    int nodesExplored = 0;
    auto start_time = high_resolution_clock::now();
    while (!frontier.empty()) 
    {
        NodeAStar currentNode = frontier.top();
        frontier.pop();
        nodesExplored++;

        // Si hemos alcanzado la solución, terminamos
        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
        {
            cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
            break;
        }

        // Generar los movimientos posibles y agregarlos a la cola
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
        {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
            {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                    NodeAStar newNode = currentNode;
                    // Eliminar las fichas en las posiciones
                    for (int i = 0; i < 3; i++) 
                    {
                        if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) 
                        {
                            newNode.board[i] &= ~(oneMask << pos1);
                            newNode.board[i] &= ~(oneMask << pos2);
                        }
                    }
                    newNode.moves.push_back({pos1, pos2});
                    newNode.depth++;  // Incrementar el número de movimientos
                    
                    // Calcular la heurística y ajustar la fórmula
                    newNode.heuristic = heuristic(newNode);

                    //Aplicar el peso a la heurística
                    double weightedF = newNode.depth + weight * newNode.heuristic;

                    //Usamos la fórmula ponderada para la función de costo

                    if (explored.find(newNode) == explored.end()) 
                    //Insertamos el nodo con el valor 
                    //ponderado de f(n) en la cola de prioridad
                    {
                        frontier.push(newNode);
                        explored.insert(newNode);
                    }
                }
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time); //Tiempo en milisegundos
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return true;
}

//Busca una solución usando un número limitado de mejores nodos en cada nivel (ancho de haz)
bool BeamSearch(Board& board, int beamWidth) 
{
    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> frontier; //Cola de prioridad para los mejores nodos
    unordered_set<NodeAStar> explored; //Guarda los nodos ya explorados
    
    //Inicializa el nodo de inicio y lo agrega a la cola y al conjunto de explorados
    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}};
    frontier.push(startNode);
    explored.insert(startNode);

    int nodesExplored = 0; //Contador de nodos explorados
    auto start_time = chrono::high_resolution_clock::now(); //Tiempo de inicio

    while (!frontier.empty()) 
    {
        vector<NodeAStar> levelNodes; //Lista de nodos en el nivel actual

        //Selecciona hasta beamWidth nodos de la cola para el nivel actual
        for (int i = 0; i < beamWidth && !frontier.empty(); i++) 
        {
            levelNodes.push_back(frontier.top());
            frontier.pop();
        }

        for (auto& currentNode : levelNodes) 
        {
            //Si se encuentra una solución, imprime resultados y termina
            if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
            {
                cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
                auto end_time = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(end_time - start_time);
                double duration_in_seconds = duration.count() / 1000.0;

                cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
                cout << "Nodos explorados: " << nodesExplored << endl;
                cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
                cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;
                return true;
            }
            
            nodesExplored++; //Incrementa el contador de nodos explorados

            //Genera nodos sucesores si cumplen con las condiciones
            for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) {
                for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) {
                    if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) {
                        NodeAStar newNode = currentNode;
                        for (int i = 0; i < 3; i++) {
                            if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) {
                                newNode.board[i] &= ~(oneMask << pos1);
                                newNode.board[i] &= ~(oneMask << pos2);
                            }
                        }
                        newNode.moves.push_back({pos1, pos2});
                        newNode.depth++;
                        newNode.heuristic = heuristic(newNode);
                        
                        //Agrega nuevos nodos a la cola y al conjunto de explorados
                        if (explored.find(newNode) == explored.end()) {
                            frontier.push(newNode);
                            explored.insert(newNode);
                        }
                    }
                }
            }
        }
    }

    //Si no se encuentra una solución, imprime resultados y termina
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time);
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    cout << "No se encontró solución." << endl;

    return false;
}

//Conjunto que almacena los estados ya visitados en la búsqueda RBFS
std::unordered_set<NodeAStar> visitedStatesRBFS;
const int ALREADY_VISITED = -1;

int RBFS(Board& board, NodeAStar node, int fLimit, int& nodesExplored) 
{
    if ((node.board[RED] | node.board[BLUE] | node.board[YELLOW]) == 0)
    {
        cout<< "Encontre"<< std::endl;
        return FOUND; 
    }

    if (visitedStatesRBFS.count(node)) 
    {
        return ALREADY_VISITED;  
    }

    visitedStatesRBFS.insert(node);

    vector<NodeAStar> successors;
    int iteration_count = 0;

    for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
    {
        for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
        {
            if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) 
            {
                NodeAStar newNode = node;

                //Sin verificar límites, el índice siempre es válido
                for (int i = 0; i < 3; i++) 
                {
                    if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) 
                    {
                        newNode.board[i] &= ~(oneMask << pos1);
                        newNode.board[i] &= ~(oneMask << pos2);
                    }
                }

                newNode.depth++;
                newNode.heuristic = heuristic(newNode);
                cout << "Nodo creado con profundidad " << newNode.depth << " y heurística " << newNode.heuristic << endl;
                successors.push_back(newNode);
                nodesExplored++;
            }
            iteration_count++; //Incrementar el contador de iteraciones
        }
    }

    if (successors.empty()) return INT_MAX;

    for (auto& s : successors) 
    {
        s.heuristic = max(s.heuristic, node.heuristic);
    }

    while (true) 
    {
        sort(successors.begin(), successors.end(), greater<NodeAStar>());
        NodeAStar best = successors.back();

        if (best.heuristic > fLimit) return best.heuristic;

        int alternative = successors.size() > 1 ? successors[successors.size() - 2].heuristic : INT_MAX;
        int result = RBFS(board, best, min(fLimit, alternative), nodesExplored);

        if (result == FOUND) return FOUND;

        if (result == ALREADY_VISITED) 
        {
            successors.pop_back();
            if (successors.empty()) return INT_MAX;  //Si no quedan más sucesores, retornar INT_MAX
            continue;
        }

        best.heuristic = result;
    }
}

//Función para iniciar la búsqueda RBFS
bool start_rbfs(Board& board) {
    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}};
    int nodesExplored = 0;

    //Obtiene el tiempo de inicio y la memoria utilizada al inicio
    getrusage(RUSAGE_SELF, &start_memory);
    auto start_time = chrono::high_resolution_clock::now();

    //Ejecuta la búsqueda RBFS
    int result = RBFS(board, startNode, INT_MAX, nodesExplored);

    //Calcula la duración total y muestra estadísticas
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    double duration_in_seconds = duration.count() / 1000.0;

    cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
    cout << "Nodos explorados: " << nodesExplored << endl;
    cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
    cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

    return result == FOUND;
}

bool SMAStar(Board& board, int maxMemory) 
{

    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> frontier;
    unordered_map<NodeAStar, int> explored;
    
    NodeAStar startNode = {board.board[RED], board.board[BLUE], board.board[YELLOW], 0, heuristic(startNode), {}};
    frontier.push(startNode);
    explored[startNode] = startNode.heuristic;

    int nodesExplored = 0;
    auto start_time = chrono::high_resolution_clock::now();
    getrusage(RUSAGE_SELF, &start_memory);

    while (!frontier.empty()) 
    {
        if (frontier.size() > maxMemory) 
        {
            auto toRemove = frontier.top();
            frontier.pop();
            explored.erase(toRemove);
        }
        
        NodeAStar currentNode = frontier.top();
        frontier.pop();

        if ((currentNode.board[RED] | currentNode.board[BLUE] | currentNode.board[YELLOW]) == 0) 
        {
            cout << "Solución encontrada en " << currentNode.depth << " movimientos!" << endl;
            auto end_time = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time); //Tiempo en milisegundos
            double duration_in_seconds = duration.count() / 1000.0;

            cout << "Tiempo total: " << duration_in_seconds << " segundos." << endl;
            cout << "Nodos explorados: " << nodesExplored << endl;
            cout << "Nodos procesados por segundo: " << nodesExplored / duration_in_seconds << " nodos/seg." << endl;
            cout << "Memoria máxima utilizada: " << get_memory_usage() << " KB." << endl;

            return true;
        }

        nodesExplored++;
        for (int pos1 = 0; pos1 < BOARD_SIZE; ++pos1) 
        {
            for (int pos2 = pos1 + 1; pos2 < BOARD_SIZE; ++pos2) 
            {
                if (board.areSameColor(pos1, pos2) && board.isPathClear(pos1, pos2) && board.isRectangleClear(pos1, pos2)) 
                {
                    NodeAStar newNode = currentNode;
                    for (int i = 0; i < 3; i++) 
                    {
                        if ((newNode.board[i] & (oneMask << pos1)) && (newNode.board[i] & (oneMask << pos2))) 
                        {
                            newNode.board[i] &= ~(oneMask << pos1);
                            newNode.board[i] &= ~(oneMask << pos2);
                        }
                    }
                    newNode.depth++;
                    newNode.heuristic = heuristic(newNode);

                    if (explored.find(newNode) == explored.end() || explored[newNode] > newNode.heuristic) 
                    {
                        frontier.push(newNode);
                        explored[newNode] = newNode.heuristic;
                    }
                }
            }
        }
    }
    cout << "No se encontró solución." << endl;

    return false;
}
