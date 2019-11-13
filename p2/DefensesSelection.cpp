// ###### Config options ################

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

int defenseValue(Defense *d)
{
    return (((d->damage * d->attacksPerSecond * d->range) + d->health) / d->dispersion);
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

    for (int j = 1; j < defenses.size(); j++)
    {
        for (int k = 0; k <= ases; k++,itDefe++)
        {
            if (k < (*itDefe)->cost)
            {
                matriz[j][k] = matriz[j - 1][k];
            }
            else
            {
                matriz[j][k] = std::max(matriz[j - 1][k], matriz[j - 1][k - (*itDefe)->cost] + defenseValue(*itDefe));
            }
        }
    }
}

void DEF_LIB_EXPORTED selectDefenses(std::list<Defense *> defenses, unsigned int ases, std::list<int> &selectedIDs, float mapWidth, float mapHeight, std::list<Object *> obstacles)
{
    //std::list<Defense*>::iterator itDefe = defenses.begin();
   
}
