#include "pcl_view.h"

int main(int argc, char** argv){

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () != 1){
        cout<< "Usage: " << argv[0] << " " << "pcdfile.pcd" <<endl;
        return -1;
    }

    std::string pcd_file = argv[filenames[0]];
    show_pcd(pcd_file);
}

