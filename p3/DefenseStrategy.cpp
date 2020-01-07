// ###### Config options ################

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"

using namespace Asedio;

struct defensePosition
{

    defensePosition(int x = 0, int y = 0, float value = 0.0) : x_(x), y_(y), value_(value) {}

    bool operator<(const defensePosition &d)
    {
        return value_ < d.value_;
    }
    bool operator>(const defensePosition &d)
    {
        return value_ > d.value_;
    }
    bool operator<=(const defensePosition &d)
    {
        return (value_ <= d.value_ || value_ == d.value_);
    }

    int x_, y_;
    float value_;
};

std::ostream &operator<<(std::ostream &os, const defensePosition &v)
{
    os << v.value_;
    return os;
}

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight) { return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight)
{
    i_out = (int)(pos.y * 1.0f / cellHeight);
    j_out = (int)(pos.x * 1.0f / cellWidth);
}

Vector3 SelectionMaxDefense(std::vector<defensePosition> &DefenseValue, float cellWidht, float cellHeight, int nCellsWidth, int nCellsHeight, std::list<Defense *> defenses)
{
    defensePosition maxValue;
    int posH;

    for (int j = 0; j < DefenseValue.size(); ++j)
    {
        if (maxValue < DefenseValue[j])
        {
            maxValue = DefenseValue[j];
            posH = j;
        }
    }

    DefenseValue.erase(DefenseValue.begin() + posH);

    return cellCenterToPosition(maxValue.x_, maxValue.y_, cellWidht, cellHeight);
}

float defaultCellValue(int row, int col, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, Defense *defense)
{
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    float value = 0;
    Vector3 position((col * cellWidth) + cellWidth * 0.5f, (row * cellHeight) + cellHeight * 0.5f, 0);

    for (List<Object *>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
    {
        value += _distance(position, (*it)->position);
    }

    return value;
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

void insertionSort(std::vector<defensePosition> &v, int i, int j)
{
    int m;
    defensePosition def;

    for (int l = i + 1; l <= j; l++)
    {
        def = v[l];
        m = l - 1;

        while (m >= i && v[m] > def)
        {
            v[m + 1] = v[m];
            m--;
        }
        v[m + 1] = def;
    }
}

void fusion(std::vector<defensePosition> &v, int i, int j, int k)
{
    int right, left, l = 0, m = 0, n = i;
    left = k - i + 1;
    right = j - k;

    std::vector<defensePosition> aLeft(left), aRight(right);

    for (int x = 0; x < left; ++x)
    {
        aLeft[x] = v[x + i];
    }

    for (int r = 0; r < right; ++r)
    {
        aRight[r] = v[k + 1 + r];
    }

    while (l < left && m < right)
    {
        if (aLeft[l] <= aRight[m])
        {
            v[n] = aLeft[l];
            l++;
        }
        else
        {
            v[n] = aRight[m];
            m++;
        }

        n++;
    }

    while (l < left)
    {
        v[n] = aLeft[l];
        l++;
        n++;
    }

    while (m < right)
    {
        v[n] = aRight[m];
        m++;
        n++;
    }
}

void fusionSort(std::vector<defensePosition> &v, int i, int j)
{
    int n = j - i + 1;

    if (n <= 5)
    {
        insertionSort(v, i, j);
    }
    else
    {
        int k = i - 1 + (n) / 2;
        fusionSort(v, i, k);
        fusionSort(v, k + 1, j);
        fusion(v, i, j, k);
    }
}

int partition(std::vector<defensePosition> &v, int i, int j)
{
    int left, right;
    defensePosition pivot = v[i];

    left = i;
    right = j;

    while (left < right)
    {
        while (v[right] > pivot)
        {
            right--;
        }
        while ((left < right) && (v[left] <= pivot))
        {
            left++;
        }

        if (left < right)
        {
            std::swap(v[left], v[right]);
        }
    }

    std::swap(v[right], v[i]);

    return right;
}

void quickSort(std::vector<defensePosition> &v, int i, int j)
{
    int n = j - i + 1;

    if (n <= 5)
    {
        insertionSort(v, i, j);
    }
    else
    {
        int pivot = partition(v, i, j);

        quickSort(v, i, pivot - 1);
        quickSort(v, pivot + 1, j);
    }
}

void heapSort(std::vector<defensePosition> &v)
{
    std::make_heap(v.begin(), v.end());
    std::sort_heap(v.begin(), v.end());
}

void DEF_LIB_EXPORTED placeDefenses3(bool **freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object *> obstacles, List<Defense *> defenses)
{

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    std::vector<defensePosition> fusionValues;
    List<Defense *>::iterator currentDefense = defenses.begin();
    long int rHeap = 0;
    int maxAttemps = 1000;
    cronometro cFusion;

    //----------- HEAP -----------//

    for (int i = 0; i < nCellsHeight; ++i)
    {
        for (int j = 0; j < nCellsWidth; ++j)
        {
            fusionValues.push_back(defensePosition(i, j, defaultCellValue(i, j, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, *defenses.begin())));
        }
    }

    std::vector<defensePosition>::iterator itFusion = fusionValues.end() - 1;
    fusionSort(fusionValues, 0, fusionValues.size() - 1);
    while (currentDefense != defenses.end() && maxAttemps > 0)
    {
        Vector3 positionSelect = cellCenterToPosition((itFusion)->x_, (itFusion)->y_, cellWidth, cellHeight);
        if (factibilidad((itFusion)->x_, (itFusion)->y_, nCellsWidth, nCellsHeight, mapWidth, mapHeight, (*currentDefense)->id, obstacles, defenses))
        {
            maxAttemps = 1000;
            (*currentDefense)->position = positionSelect;
            ++currentDefense;
            fusionValues.erase(itFusion);
            itFusion = fusionValues.end() - 1;
        }
        itFusion--;
        maxAttemps--;
    }
}
