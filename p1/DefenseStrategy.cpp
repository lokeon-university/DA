// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1  // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
//RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight) { return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight)
{
    i_out = (int)(pos.y * 1.0f / cellHeight);
    j_out = (int)(pos.x * 1.0f / cellWidth);
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

Vector3 SelectionFirstDefense(std::vector<std::vector<float>> &FirstDefenseValue, float cellWidht, float cellHeight, int nCellsWidth, int nCellsHeight, std::list<Defense *> defenses)
{
    float maxValue = -INF_F;
    int posH, posW;

    for (int i = 0; i < nCellsHeight; ++i)
    {
        for (int j = 0; j < nCellsWidth; ++j)
        {

            if (maxValue < FirstDefenseValue[i][j])
            {
                maxValue = FirstDefenseValue[i][j];
                posH = i;
                posW = j;
            }
        }
    }

    FirstDefenseValue[posH][posW] = -INF_F;

    return cellCenterToPosition(posH, posW, cellWidht, cellHeight);
}

Vector3 SelectionDefense(std::vector<std::vector<float>> &DefenseValue, float cellWidht, float cellHeight, int nCellsWidth, int nCellsHeight, int idDef, std::list<Defense *> defenses)
{
    float maxValue = -INF_F;
    int posH, posW;

    for (int i = 0; i < nCellsHeight; ++i)
    {
        for (int j = 0; j < nCellsWidth; ++j)
        {
            if (idDef != (*defenses.begin())->id)
            {
                if (maxValue < DefenseValue[i][j])
                {
                    maxValue = DefenseValue[i][j];
                    posH = i;
                    posW = j;
                }
            }
        }
    }

    DefenseValue[posH][posW] = -INF_F;

    return cellCenterToPosition(posH, posW, cellWidht, cellHeight);
}

float cellFirstDefenseValue(int row, int col, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, List<Defense *> defenses)
{
    float distanceToObs = INF_F;
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    float distanceUp, distanceDown, distanceCenter;
    List<Object *>::iterator itObs;
    Vector3 position = cellCenterToPosition(row, col, cellWidth, cellHeight);

    for (itObs = obstacles.begin(); itObs != obstacles.end(); ++itObs)
    {
        if (distanceToObs > _distance(position, (*itObs)->position))
        {
            distanceToObs = _distance(position, (*itObs)->position);
        }
    }

    distanceCenter = abs(row - nCellsWidth / 2) + abs(col - nCellsHeight / 2);
    distanceDown = _distance(position, Vector3(position.x, mapWidth, 0));
    distanceUp = _distance(position, Vector3(position.x, 0, 0));

    return (distanceDown + distanceUp) - (distanceToObs + distanceCenter);
}

float cellDefenseValue(int row, int col, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, Defense *defense)
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

bool sortByDamage(Defense *d1, Defense *d2)
{
    return (d1->damage * d1->attacksPerSecond) > (d2->damage * d2->attacksPerSecond);
}

void DEF_LIB_EXPORTED placeDefenses(bool **freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, std::list<Object *> obstacles, std::list<Defense *> defenses)
{

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    int maxAttemps = 1000;
    int row, col;

    List<Defense *>::iterator currentDefense = defenses.begin();
    List<Defense *> DefenseAux = defenses;
    std::vector<std::vector<float>> FirstDefenseValue(nCellsHeight, std::vector<float>(nCellsWidth));
    std::vector<std::vector<float>> DefenseValue(nCellsHeight, std::vector<float>(nCellsWidth));
    std::vector<std::vector<float>> DefenseValueAux(nCellsHeight, std::vector<float>(nCellsWidth));
    Defense *defenseAux = *defenses.begin();

    for (int i = 0; i < nCellsHeight; ++i)
    {
        for (int j = 0; j < nCellsWidth; ++j)
        {
            FirstDefenseValue[i][j] = cellFirstDefenseValue(i, j, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
        }
    }

    while (currentDefense == defenses.begin() && maxAttemps > 0)
    {
        --maxAttemps;

        Vector3 positionSelect = SelectionFirstDefense(FirstDefenseValue, cellWidth, cellHeight, nCellsWidth, nCellsHeight, defenses);
        positionToCell(positionSelect, row, col, cellWidth, cellHeight);

        if (factibilidad(row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
        {
            (*currentDefense)->position = positionSelect;
            ++currentDefense;
            maxAttemps = 1000;
        }
    }

    for (int i = 0; i < nCellsHeight; ++i)
    {
        for (int j = 0; j < nCellsWidth; ++j)
        {
            DefenseValue[i][j] = cellDefenseValue(i, j, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, *defenses.begin());
        }
    }

    DefenseValueAux = DefenseValue;

    DefenseAux.pop_front();
    DefenseAux.sort(sortByDamage);
    DefenseAux.push_front(defenseAux);

    defenses = DefenseAux;

    while (currentDefense != defenses.end() && maxAttemps > 0)
    {
        --maxAttemps;

        Vector3 positionSelect = SelectionDefense(DefenseValueAux, cellWidth, cellHeight, nCellsWidth, nCellsHeight, (**currentDefense).id, defenses);
        positionToCell(positionSelect, row, col, cellWidth, cellHeight);

        if (factibilidad(row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
        {
            (*currentDefense)->position = positionSelect;
            ++currentDefense;
            maxAttemps = 1000;
            DefenseValueAux = DefenseValue;
        }
    }

#ifdef PRINT_DEFENSE_STRATEGY

    float **cellValues = new float *[nCellsHeight];
    for (int i = 0; i < nCellsHeight; ++i)
    {
        cellValues[i] = new float[nCellsWidth];
        for (int j = 0; j < nCellsWidth; ++j)
        {
            cellValues[i][j] = ((int)(cellValue(i, j))) % 256;
        }
    }
    dPrintMap("strategy.ppm", nCellsHeight, nCellsWidth, cellHeight, cellWidth, freeCells, cellValues, std::list<Defense *>(), true);

    for (int i = 0; i < nCellsHeight; ++i)
        delete[] cellValues[i];
    delete[] cellValues;
    cellValues = NULL;

#endif
}
