// ###### Config options ################

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

int defenseValue(Defense *d)
{
    return (((d->damage * d->attacksPerSecond * d->range) + d->health));
}

void changeValueDefense(std::vector<std::vector<int>> &matriz, std::list<Defense *> defenses, unsigned int ases)
{
    std::list<Defense *>::iterator itDefe = defenses.begin();

    for (int i = 0; i <= ases; i++)
    {
        if (i < (*itDefe)->cost)
        {
            matriz[0][i] = 0;
        }
        else
        {
            matriz[0][i] = defenseValue(*itDefe);
        }
    }

    for (int j = 1; j < defenses.size(); j++, itDefe++)
    {
        for (int k = 0; k <= ases; k++)
        {
            if (k >= (*itDefe)->cost)
            {
                matriz[j][k] = std::max(matriz[j - 1][k], matriz[j - 1][k - (*itDefe)->cost] + defenseValue(*itDefe));
            }
            else
            {
                matriz[j][k] = matriz[j - 1][k];
            }
        }
    }
}

void bestCombination(std::vector<std::vector<int>> &matriz, std::list<int> &selectedIDs, std::list<Defense *> defenses, std::list<Defense *>::iterator itDefe, int idDefense, unsigned int ases)
{
    int j = ases;

    for (int i = defenses.size() - 1; i > 0; --i, --itDefe)
    {
        if (matriz[i][j] != matriz[i - 1][j])
        {
            selectedIDs.push_front((*itDefe)->id);
            ases -= (*itDefe)->cost;
        }
    }

    selectedIDs.push_front(idDefense);
}

void DEF_LIB_EXPORTED selectDefenses(std::list<Defense *> defenses, unsigned int ases, std::list<int> &selectedIDs, float mapWidth, float mapHeight, std::list<Object *> obstacles)
{

    std::list<Defense *>::iterator itDefe = defenses.end();
    --itDefe;

    // Primera defensa
    ases -= (*defenses.begin())->cost;
    int idDefense = (*defenses.begin())->id;

    defenses.pop_front();
    std::vector<std::vector<int>> matriz(defenses.size(), std::vector<int>(ases + 1));

    changeValueDefense(matriz, defenses, ases);

    bestCombination(matriz, selectedIDs, defenses, itDefe, idDefense, ases);
}
