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

float heuristic(AStarNode *originNode, AStarNode *targetNode, float **additionalCost, float cellWidth, float cellHeight)
{
}

// rellenar la matriz de costes , no tocar , hasta implementar el Algoritmo A*, luego tocar esto pa tener mejor cosas
void DEF_LIB_EXPORTED calculateAdditionalCost(float **additionalCost, int cellsWidth, int cellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, List<Defense *> defenses)
{

    float cellWidth = mapWidth / cellsWidth;
    float cellHeight = mapHeight / cellsHeight;

    for (int i = 0; i < cellsHeight; ++i)
    {
        for (int j = 0; j < cellsWidth; ++j)
        {
            Vector3 cellPosition = cellCenterToPosition(i, j, cellWidth, cellHeight);
            float cost = 0;
            if ((i + j) % 2 == 0)
            {
                cost = cellWidth * 100;
            }

            additionalCost[i][j] = cost;
        }
    }
}
// AStarNODE* tiene to la informacion pa implementar el Algoritmo A*
// meter el camino recorriendo los padres de AsterNode y meterlo en path (push_back) orden de origen->destino
void DEF_LIB_EXPORTED calculatePath(AStarNode *originNode, AStarNode *targetNode, int cellsWidth, int cellsHeight, float mapWidth, float mapHeight, float **additionalCost, std::list<Vector3> &path)
{

    int maxIter = 100;
    bool target = false, opened = false, closed = false;
    std::vector<AStarNode *> opened, closed;

    AStarNode *current = originNode;
    current->G = 0;
    current->H = heuristic(originNode, targetNode, additionalCost, cellsWidth, cellsHeight);
    current->F = current->G + current->H;

    while (current != targetNode && maxIter > 0)
    { // @todo ensure current and target are connected
        float min = INF_F;
        AStarNode *o = NULL;
        for (List<AStarNode *>::iterator it = current->adjacents.begin(); it != current->adjacents.end(); ++it)
        {
            float dist = _sdistance((*it)->position, targetNode->position);
            if (additionalCost != NULL)
            {
                dist += additionalCost[(int)((*it)->position.y / cellsHeight)][(int)((*it)->position.x / cellsWidth)];
            }
            //std::cout << (*it)->position.y << ", " << (*it)->position.x << std::endl;
            if (dist < min)
            {
                min = dist;
                o = (*it);
            }
        }

        current = o;

        if (current == NULL)
        {
            break;
        }

        path.push_back(current->position);
        --maxIter;
    }
}
