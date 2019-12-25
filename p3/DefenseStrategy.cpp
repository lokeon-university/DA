// ###### Config options ################

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight) { return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight)
{
    i_out = (int)(pos.y * 1.0f / cellHeight);
    j_out = (int)(pos.x * 1.0f / cellWidth);
}

float defaultCellValue(int row, int col, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, Defense *defense)
{
    float distanceToObs = INF_F;
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    float distanceUp, distanceDown, distanceCenter, distanceToFirstDefense;
    List<Object *>::iterator itObs;
    Vector3 position = cellCenterToPosition(row, col, cellWidth, cellHeight);

    for (itObs = obstacles.begin(); itObs != obstacles.end(); ++itObs)
    {
        if (distanceToObs > _distance(position, (*itObs)->position))
        {
            distanceToObs = _distance(position, (*itObs)->position);
        }
    }

    distanceToFirstDefense = _distance((defense)->position, position);
    distanceCenter = abs(row - cellWidth / 2) + abs(col - cellHeight / 2);
    distanceDown = _distance(position, Vector3(position.x, mapWidth, 0));
    distanceUp = _distance(position, Vector3(position.x, 0, 0));

    return (distanceDown + distanceUp) - (0.5 * distanceToObs + distanceCenter + 2 * distanceToFirstDefense);
}

bool factibilidad(int row, int col, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, int idDef, List<Object *> obstacles, List<Defense *> defenses)
{
    Vector3 walls = cellCenterToPosition(row, col, mapWidth / nCellsWidth, mapHeight / nCellsHeight);
    std::list<Defense *>::iterator itWall, itDefe;
    std::list<Object *>::iterator itObs;
    bool factible = true;

    for (itWall = defenses.begin(); (*itWall)->id != idDef; ++itWall)
    {
        if (itWall == defenses.end())
        {
            factible = false;
        }
    }

    for (itDefe = defenses.begin(); itDefe != defenses.end() && factible; ++itDefe)
    {
        if (_distance(walls, (*itDefe)->position) < (*itWall)->radio + (*itDefe)->radio)
        {
            factible = false;
        }
    }

    for (itObs = obstacles.begin(); itObs != obstacles.end() && factible; ++itObs)
    {
        if (_distance(walls, (*itObs)->position) < (*itWall)->radio + (*itObs)->radio)
        {
            factible = false;
        }
    }

    //Tamaño
    if (row < 0 || row > nCellsHeight || col < 0 || col > nCellsWidth)
    {
        factible = false;
    }

    //Esquinas
    if (_distance(Vector3(0.0, walls.y, 0.0), walls) < (*itWall)->radio || _distance(Vector3(walls.x, 0.0, 0.0), walls) < (*itWall)->radio || _distance(Vector3(mapWidth, walls.y, 0.0), walls) < (*itWall)->radio || _distance(Vector3(walls.x, mapHeight, 0.0), walls) < (*itWall)->radio)
    {
        factible = false;
    }

    return factible;
}

void DEF_LIB_EXPORTED placeDefenses3(bool **freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, List<Defense *> defenses)
{

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    std::vector<std::vector<float>> defaultValues(nCellsHeight, std::vector<float>(nCellsWidth));
    std::vector<std::vector<float>> fusionValues, heapValues, quickValues, noorderValues;
    int row, col;

    cronometro cFusion, cMonticulo, cRapido, cNOrden;
    long int r = 0;

    for (int i = 0; i < nCellsHeight; ++i)
    {
        for (int j = 0; j < nCellsWidth; ++j)
        {
            defaultValues[i][j] = defaultCellValue(i, j, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, *defenses.begin());
        }
    }

    do
    {
        List<Defense *>::iterator currentDefense = defenses.begin();

        //----------- FUSION -----------//
        fusionValues = defaultValues;
        cFusion.activar();
        
        while (currentDefense != defenses.end())
        {
            Vector3 positionSelect = (*currentDefense)->position;
            positionToCell(positionSelect, row, col, cellWidth, cellHeight);

            if (factibilidad(row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
            {
                (*currentDefense)->position = positionSelect;
                ++currentDefense;
            }
        }
        cFusion.parar();

        //----------- MONTICULO -----------//
        heapValues = defaultValues;
        currentDefense = defenses.begin();
        cMonticulo.activar();
        List<Defense *>::iterator currentDefense = defenses.begin();
        while (currentDefense != defenses.end())
        {
            Vector3 positionSelect = (*currentDefense)->position;
            positionToCell(positionSelect, row, col, cellWidth, cellHeight);

            if (factibilidad(row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
            {
                (*currentDefense)->position = positionSelect;
                ++currentDefense;
            }
        }
        cMonticulo.parar();

        //----------- RAPIDO -----------//
        quickValues = defaultValues;
        currentDefense = defenses.begin();
        cRapido.activar();
        List<Defense *>::iterator currentDefense = defenses.begin();
        while (currentDefense != defenses.end())
        {
            Vector3 positionSelect = (*currentDefense)->position;
            positionToCell(positionSelect, row, col, cellWidth, cellHeight);

            if (factibilidad(row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
            {
                (*currentDefense)->position = positionSelect;
                ++currentDefense;
            }
        }
        cRapido.parar();

        //----------- SIN ORDEN -----------//
        noorderValues = defaultValues;
        currentDefense = defenses.begin();
        cNOrden.activar();
        List<Defense *>::iterator currentDefense = defenses.begin();
        while (currentDefense != defenses.end())
        {
            Vector3 positionSelect = (*currentDefense)->position;
            positionToCell(positionSelect, row, col, cellWidth, cellHeight);

            if (factibilidad(row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
            {
                (*currentDefense)->position = positionSelect;
                ++currentDefense;
            }
        }
        cNOrden.parar();

    } while (cFusion.tiempo() + cRapido.tiempo() + cNOrden.tiempo() + cMonticulo.tiempo() < 1.0);

    std::cout << (nCellsWidth * nCellsHeight)
              << '\t' << "Fusion: " << cFusion.tiempo()
              << '\t' << "Rapido: " << cRapido.tiempo()
              << '\t' << "Sin orden: " << cNOrden.tiempo()
              << '\t' << "Monticulo: " << cMonticulo.tiempo()
              << std::endl;
}
