#include "DataStructure.h"
#include "Window.h"


int main() {

    std::vector<std::string> lasFiles{R"(3dm_32_389_5705_1_nw.las)" };
    std::string shpFile = R"(gis_osm_buildings_a_free_1.shp)";



    // read tree
    try
    {
        auto lol = DataStructure(lasFiles, shpFile);

        std::cout << "main\t" << "DataStructure finished" << std::endl;

        Window windi = Window(lol);
    }

    catch (std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }



    return 0;
}
