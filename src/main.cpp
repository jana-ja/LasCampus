#include "DataStructure.h"
#include "Window.h"

extern "C" {
#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif //STB_IMAGE_IMPLEMENTATION
}

int main() {

    // in las folder
    std::vector<std::string> lasFiles{R"(3dm_32_389_5705_1_nw.las)" };
    // in shp folder
    std::string shpFile = R"(gis_osm_buildings_a_free_1.shp)";
    // in gml folder
    std::string gmlFile = R"(LoD2_32_389_5705_1_NW.gml)";
    // in img folder
    std::string imgFile = R"(dop10rgbi_32_389_5705_1_nw_2021.jpg)";



    // read tree
    try
    {
        auto lol = DataStructure(lasFiles, shpFile, gmlFile, imgFile);

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
