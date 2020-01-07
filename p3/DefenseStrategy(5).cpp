// ###### Config options ################



// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"

using namespace Asedio;   


struct compareDefenses {
    bool operator()(Defense * d1, Defense * d2) {return ((d1->health*1/5 + d1->attacksPerSecond*1/5 + d1->range*1/5 + d1->damage*1/5
                                                        + d1-> dispersion*1/5) > (d2->health*1/5 + d2->attacksPerSecond*1/5 + d2->range*1/5 + d2->damage*1/5
                                                        + d2-> dispersion*1/5));}
};


struct cellValueStruct{

    cellValueStruct(int xx = 0, int yy = 0, float vv = 0.0) : x(xx), y(yy), value(vv) {}

    bool operator<(const cellValueStruct &d){
        return value < d.value;
    }

    bool operator>(const cellValueStruct &d){
        return value > d.value;
    }

    bool operator<=(const cellValueStruct &d){
        return (value <= d.value) || (value == d.value);
    }

    int x, y;
    float value;
};
struct compare {
    bool operator()(const cellValueStruct &d1, cellValueStruct &d2) {return d1.value < d2.value;}
};

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight){ i_out = (int)(pos.y * 1.0f/cellHeight); j_out = (int)(pos.x * 1.0f/cellWidth); }

Vector3 selection(std::vector<cellValueStruct> &v, float cellWidth, float cellHeight) {
    float r= -INF_F;
    int max = 0;
    for(int i = 0; i < v.size(); i++){
            if(/* v[i].value != -INF_F &&*/ v[i].value > r){
                max=i;
                r=v[i].value;
            }
    }
    int ii = v[max].x;
    int jj = v[max].y;
    v.erase(v.begin()+max);
    //v[max].value = -INF_F; //Simbólico para marcarlo como seleccionado
    //Devuelve el valor máximo
    return cellCenterToPosition(ii, jj, cellWidth, cellHeight);
}


bool feasibility(int id, int row, int col, int nCellsWidth, int nCellsHeight, 
                float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses){
    
    bool b=true; //Hasta que no se demuestre lo contrario, puede colocarse
    Vector3 defense=cellCenterToPosition(row, col, mapWidth / nCellsWidth, mapHeight / nCellsHeight);
   
    List<Defense*>::iterator dfP;
    for(dfP = defenses.begin(); dfP != defenses.end() && (*dfP)->id != id; dfP++ ){}
    //Lo localiza en la lista para poder acceder a sus características

    //Comprobación de si se sale del mapa:
    if( defense.x - (*dfP)->radio < 0 ||
        defense.x + (*dfP)->radio > mapWidth ||
        defense.y - (*dfP)->radio < 0 ||
        defense.y + (*dfP)->radio > mapHeight ||
        row < 0 || row > nCellsHeight || col < 0 || col > nCellsWidth) b = false; 


    //Comprueba si colisiona con el resto de defensas
    for(List<Defense*>::iterator dfIt = defenses.begin(); dfIt != defenses.end() && b!= false; dfIt++){
            //Si comprueba que colisiona con alguno, deja de mirar
        if( _distance(defense, (*dfIt)->position) - (*dfP)->radio  - (*dfIt)->radio < 0) b = false;
    }
    
    for(List<Object*>::iterator m = obstacles.begin(); m != obstacles.end() && b!= false; m++ ){
    //Compruebo si colisiona con alguna montaña
        if((_distance(defense, (*m)->position) - (*dfP)->radio  - (*m)->radio < 0)) b = false;
    }

    return b;

}


// ##### FUSIÓN ####

void fusion(std::vector<cellValueStruct> &v, int i, int j, int k){
    int right = j - k, left= k - i + 1, iaux = 0, jaux = 0, kaux = i;

    std::vector<cellValueStruct> L(left), R(right);

    for (int x = 0; x < left; ++x)
        L[x] = v[x + i];

    for (int r = 0; r < right; ++r)
        R[r] = v[k + 1 + r];

    while (iaux < left && jaux < right)
    {
        if (L[iaux] <= R[jaux]){
            v[kaux] = L[iaux];
            iaux++;
        }
        else{
            v[kaux] = R[jaux];
            jaux++;
        }

        kaux++;
    }

    while (iaux < left)
    {
        v[kaux] = L[iaux];
        iaux++;
        kaux++;
    }

    while (jaux < right)
    {
        v[kaux] = R[jaux];
        jaux++;
        kaux++;
    }
}
/* i is for left index and j is right index of the 
   sub-array of v to be sorted */
void sortFusion(std::vector<cellValueStruct> &v, int i, int j) 
{ 
    if (i < j) 
    { 
        // Same as (i+j)/2, but avoids overflow for 
        // large i and h 
        int k = i+(j-i)/2; 
  
        // Sort first and second halves 
        sortFusion(v, i, k); 
        sortFusion(v, k+1, j); 
  
        fusion(v, i, j, k); 
    } 
} 

// #### QUICKSORT ####


int pivote(std::vector<cellValueStruct> &v, int start, int end)  {  
    cellValueStruct pivot = v[end]; 
    int min = (start - 1);  
  
    for (int i = start; i < end; i++){  
        if (v[i] <= pivot){  
            min++; 
            std::swap(v[min], v[i]);  
        }  
    }  
    std::swap(v[min + 1], v[end]);  
    return (min + 1);  
}  

void quickSort(std::vector<cellValueStruct> &v, int i, int j){

    if (i < j) {
        int pivot = pivote(v, i, j);
        quickSort(v, i, pivot - 1);
        quickSort(v, pivot + 1, j);
    }
}


// #### MONTICULO ####

void heap(std::vector<cellValueStruct> &v){
    std::make_heap(v.begin(), v.end());
    std::sort_heap(v.begin(), v.end());
}

float defaultCellValue(int row, int col, int nCellsWidth, int nCellsHeight
    , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses) {
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight; 

    Vector3 cell = cellCenterToPosition(row, col, cellWidth, cellHeight);
    Vector3 center(mapWidth/2, mapHeight/2, 0);

	return  -_distance(cell, center); 
}

void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , List<Object*> obstacles, List<Defense*> defenses) {

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight; 
	cronometro cNoOrder, cQuickSort, cFusion, cHeap;
    int rNoOrder = 0, rFusion = 0, rQuickSort = 0, rMonticulo = 0; 

    for(int it = 0; it<4; it++){
    
        //inicializo el mapa de valores
        std::vector<std::vector<float>> MapValue(nCellsHeight, std::vector<float>(nCellsWidth));
        int maxAttemps = 1000;
        //defensescopy.sort(compare());
        std::vector<cellValueStruct> cells;
        std::list<cellValueStruct> cellList;


        switch(it){
            case 0:
                cNoOrder.activar();
            case 1: 
                cFusion.activar();
            break;
            case 2:   
                cQuickSort.activar();
            break;
            case 3: 
                cHeap.activar();
            break;
        }

        for(int i = 0; i < nCellsHeight; i++){
            for(int j = 0; j < nCellsWidth; j++){
                MapValue[i][j] = defaultCellValue(i, j, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
            }
        }

      
        for(int i = 0; i < nCellsHeight; i++){
            for(int j = 0; j < nCellsWidth; j++){
                cells.push_back(cellValueStruct(i, j, MapValue[i][j]));
            }
        }
    

        switch(it){
            case 0 :
            //cells.sort(compare());
            sort(cells.begin(), cells.end(), compare());
                break;
            case 1: 
                sortFusion(cells, 0, cells.size()-1); 
            break;
            case 2:   
                quickSort(cells, 0, cells.size()-1); 
            break;
            case 3: 
                heap(cells); 
            break;
        }
        
        //for(int i = 0; i < cells.size(); i++) cellList.push_back(cells[i]);
            
        Vector3 v;
        List<cellValueStruct>::iterator currentCell = (cellList.end());
        currentCell--;
        int i = cells.size()-1;
        int row, col;
        //do{
            for( List<Defense*>::iterator currentDefense = defenses.begin(); currentDefense != defenses.end() && maxAttemps > 0; maxAttemps--, i--){
                
                switch(it){
                    case 0 :
                        rNoOrder++;
                        break;
                    case 1: 
                        rFusion++;
                        break;
                    case 2:   
                        rQuickSort++;
                        break;
                    case 3: 
                        rMonticulo++;
                        break;
                }
            
                if(it!=0) v = cellCenterToPosition(cells[i].x, cells[i].y, cellWidth, cellHeight);
                else v = selection(cells, cellWidth, cellHeight);
                positionToCell(v, row, col, cellWidth, cellHeight);
                if(feasibility((*currentDefense)->id, row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses)){
                        maxAttemps = 1000;
                        (*currentDefense)->position = v;
                        cells.erase(cells.begin()+i);
                        i=cells.size()-1;
                        currentDefense++;
                    } 
            }
        //} while (cNOrden.tiempo() < (0.01 / (0.001 + 0.01)));

        
/*
int i = v.size()-1;
int row, col;
 for( List<Defense*>::iterator currentDefense = defensesCopy.begin(); defenseCopy.empty()==true && maxAttemps > 0; maxAttemps--, currentDefense--){
               if(it!=0) v = cellCenterToPosition(cells[i]->x, cells[i]->y, cellWidth, cellHeight);
               else v = selection(cells, cellWidth, cellHeight);
               positionToCell(v, row, col, cellWidth, cellHeight);
            
               if(feasibility((*currentDefense)->id, row, col, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses)){
                maxAttemps = 1000;
                (*currentDefense)->position = v;
                defensesCopy.erase(currentDefense); 
                if(it!=0) i--;
                currentDefense=defensesCopy.begin();
            } 
        }

*/
    
        switch(it){
            case 0:
                cNoOrder.parar();
            case 1: 
                cFusion.parar();
            break;
            case 2:   
                cQuickSort.parar();
            break;
            case 3: 
                cHeap.parar();
            break;
        }

        if(it!=0) cells.clear();
    }

     std::cout << (nCellsWidth * nCellsHeight)
              << '\t' << cNoOrder.tiempo() / rNoOrder
              << '\t' << cFusion.tiempo() / rFusion
              << '\t' << cQuickSort.tiempo() / rQuickSort
              << '\t' << cHeap.tiempo() / rMonticulo
              << std::endl;
}
