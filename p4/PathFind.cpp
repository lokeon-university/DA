// ###### Config options ################

#define PRINT_PATHS 1
// valorar celdas segun la f y la heuristica , el camino de los uco es de donde sale(bordes) hsata el centro de extraccion
// targt_node : centro de extraccion OBJETIVO: conseguir menos tiempo -> ahora somos UCOS queremos destruir defensas

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_PATHS
#include "ppm.h"
#endif

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight)
{
    return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0);
}
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight)
{
    i_out = (int)(pos.y * 1.0f / cellHeight);
    j_out = (int)(pos.x * 1.0f / cellWidth);
}

float estimatedDistance(AStarNode *originNode, AStarNode *targetNode, float cellWidth, float cellHeight)
{
    return _distance(originNode->position, targetNode->position);
}

// Son Vector3, asi que distancia a 0 y yata
bool operator==(const AStarNode &o, const AStarNode &t)
{
    return (_distance(o.position, t.position) == 0.0);
}

bool heapMinimum(const AStarNode *o, const AStarNode *t)
{
    return (o->F > t->F);
}

// rellenar la matriz de costes , no tocar , hasta implementar el Algoritmo A*, luego tocar esto pa tener mejor cosas
void DEF_LIB_EXPORTED calculateAdditionalCost(float **additionalCost, int cellsWidth, int cellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, List<Defense *> defenses)
{

    float cellWidth = mapWidth / cellsWidth;
    float cellHeight = mapHeight / cellsHeight;
    float cost = 0.0;
    Vector3 cellPosition;
    List<Object *>::iterator itObs;
    List<Defense *>::iterator itDef;

    for (int i = 0; i < cellsHeight; ++i)
    {
        for (int j = 0; j < cellsWidth; ++j)
        {
            cellPosition = cellCenterToPosition(i, j, cellWidth, cellHeight);

            for (itDef = defenses.begin(); itDef != defenses.end(); ++itDef)
            {
                if ((*itDef)->radio > _distance(cellPosition, (*itDef)->position))
                {
                    cost += _distance(cellPosition, (*itDef)->position) / (*itDef)->radio;
                }
            }

            //Distancia respecto a los obstacles
            for (itObs = obstacles.begin(); itObs != obstacles.end(); ++itObs)
            {
                if ((*itObs)->radio > _distance(cellPosition, (*itObs)->position))
                {
                    cost += _distance(cellPosition, (*itObs)->position);
                }
            }

            additionalCost[i][j] = cost;
        }
    }
}

// meter el camino recorriendo los padres de AsterNode y meterlo en path (push_back) orden de origen->destino
void DEF_LIB_EXPORTED calculatePath(AStarNode *originNode, AStarNode *targetNode, int cellsWidth, int cellsHeight, float mapWidth, float mapHeight, float **additionalCost, std::list<Vector3> &path)
{
    bool target = false;
    std::vector<AStarNode *> open, close;
    List<AStarNode *>::iterator adjacent;
    float distance;
    int row, col;

    AStarNode *current = originNode;
    positionToCell(originNode->position, row, col, cellsWidth, cellsHeight);
    current->G = 0;
    current->H = estimatedDistance(originNode, targetNode, cellsWidth, cellsHeight) + additionalCost[row][col];
    // std::cout << estimatedDistance(originNode, targetNode, cellsWidth, cellsHeight) + additionalCost[row][col] << std::endl;
    current->F = current->G + current->H;
    open.push_back(current);
    std::make_heap(open.begin(), open.end());

    while (!target && !open.empty())
    { // @todo ensure current and target are connected
        current = open.front();
        std::pop_heap(open.begin(), open.end(), heapMinimum);
        open.pop_back();
        close.push_back(current);
        //comprobar posiciones, si son la misma
        if (*current == *targetNode)
        {
            target = true;
        }
        else
        {
            // miramos los adyacentes
            for (adjacent = current->adjacents.begin(); adjacent != current->adjacents.end(); adjacent++)
            {
                // que no este cerrado
                if (std::find(close.begin(), close.end(), (*adjacent)) == close.end())
                {
                    //que no este en los abiertos
                    if (std::find(open.begin(), open.end(), (*adjacent)) == open.end())
                    {
                        (*adjacent)->parent = current;
                        positionToCell((*adjacent)->position, row, col, cellsWidth, cellsHeight);
                        (*adjacent)->G = current->G + _distance(current->position, (*adjacent)->position);
                        (*adjacent)->H = estimatedDistance((*adjacent), targetNode, cellsWidth, cellsHeight) + additionalCost[row][col];
                        // std::cout << estimatedDistance(originNode, targetNode, cellsWidth, cellsHeight) + additionalCost[row][col] << std::endl;
                        (*adjacent)->F = (*adjacent)->H + (*adjacent)->G;
                        open.push_back((*adjacent));
                        std::push_heap(open.begin(), open.end(), heapMinimum);
                    }
                    else
                    {
                        distance = _distance(current->position, (*adjacent)->position);
                        if ((*adjacent)->G > current->G + distance)
                        {
                            (*adjacent)->parent = current;
                            (*adjacent)->G = current->G + distance;
                            (*adjacent)->F = (*adjacent)->H + (*adjacent)->G;
                            std::sort_heap(open.begin(), open.end(), heapMinimum);
                        }
                    }
                }
            }
        }
    }
    //recuperar
    while (current->parent != originNode)
    {
        current = current->parent;
        path.push_front(current->position);
    }
}
