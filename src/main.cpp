#include "PointCloud.h"
#include "DummyData.h"
#include "Window.h"


int main() {

//    std::vector<std::string> files{ R"(bdom50_32389_5705_1_nw_2021.las)" };
    std::vector<std::string> files{ R"(3dm_32_389_5705_1_nw.las)" };



    // read tree
    try
    {
        auto lol = PointCloud(files);

        std::cout << "main\t" << "PointCloud finished" << std::endl;

        // for now get dummy tree
        //auto lol = DummyData();


        Window windi = Window(lol);
        //windi.setVertices(vertices);
    }

    catch (std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }



    return 0;
}
